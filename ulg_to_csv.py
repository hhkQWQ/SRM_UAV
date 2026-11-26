import argparse
import base64
import csv
import json
import math
import os
import re
import struct
import sys
from pathlib import Path


MAGIC = bytes.fromhex("55 4c 6f 67 01 12 35")
SYNC = bytes.fromhex("2f 73 13 20 25 0c bb 12")
SYNC_FRAME = b"\x08\x00S" + SYNC

FIELD = re.compile(r"([\w/-]+)(?:\[(\d+)\])?\s+([\w/-]+)", re.ASCII)
TYPES = {
    "int8_t": "b", "uint8_t": "B",
    "int16_t": "h", "uint16_t": "H",
    "int32_t": "i", "uint32_t": "I",
    "int64_t": "q", "uint64_t": "Q",
    "float": "f", "double": "d", "bool": "?",
}
LEVELS = (
    "EMERGENCY", "ALERT", "CRITICAL", "ERROR",
    "WARNING", "NOTICE", "INFO", "DEBUG",
)
COLUMNS = (
    "offset", "section", "record_type", "name", "instance", "message_id",
    "timestamp_us", "time_since_log_start_s", "status", "data_json", "raw_base64",
)


def read_text(data):
    return data.decode("utf-8", errors="backslashreplace")


def parse_field(declaration):
    match = FIELD.fullmatch(declaration.strip())
    if not match:
        raise ValueError(f"无效的字段定义：{declaration}")
    kind, count, name = match.groups()
    return kind, int(count) if count is not None else None, name


def relative_seconds(timestamp, start):
    delta = timestamp - start
    whole, fraction = divmod(abs(delta), 1_000_000)
    sign = "-" if delta < 0 else ""
    return f"{sign}{whole}.{fraction:06d}"


def json_float(value):
    if math.isfinite(value):
        return value
    if math.isnan(value):
        return "NaN"
    return "Infinity" if value > 0 else "-Infinity"


class Decoder:
    def __init__(self):
        self.formats = {}
        self.subscriptions = {}

    def read_value(self, kind, count, data, pos):
        size = 1 if count is None else count

        if kind == "char":
            end = pos + size
            if end > len(data):
                raise ValueError("字符字段不完整")
            return read_text(data[pos:end]), end

        if kind in TYPES:
            layout = struct.Struct(f"<{size}{TYPES[kind]}")
            values = layout.unpack_from(data, pos)
            if kind in ("float", "double"):
                values = tuple(json_float(value) for value in values)
            return (values[0] if count is None else values), pos + layout.size

        if size > 65535:
            raise ValueError("嵌套数组超过 ULog 单条记录的长度范围")

        schema = self.formats[kind]
        values = []
        for _ in range(size):
            value, pos = self.read_fields(schema, data, pos)
            values.append(value)
        return (values[0] if count is None else values), pos

    def read_fields(self, schema, data, pos):
        fields = {}
        for kind, count, name in schema:
            value, pos = self.read_value(kind, count, data, pos)
            if not name.startswith("_padding"):
                fields[name] = value
        return fields, pos

    def decode(self, tag, payload, row):
        if tag == "F":
            definition = read_text(payload)
            name, declarations = definition.rstrip("\0").split(":", 1)
            self.formats[name] = [
                parse_field(part)
                for part in declarations.split(";")
                if part.strip()
            ]
            row["name"] = name
            return {"format": definition}

        if tag == "A":
            instance, message_id = struct.unpack_from("<BH", payload)
            name = read_text(payload[3:]).rstrip("\0")
            self.subscriptions[message_id] = (name, instance)
            row.update(name=name, instance=instance, message_id=message_id)
            return {}

        if tag in ("D", "R"):
            message_id, = struct.unpack_from("<H", payload)
            row["message_id"] = message_id

            if tag == "R":
                subscription = self.subscriptions.pop(message_id, None)
                if subscription is not None:
                    row["name"], row["instance"] = subscription
                return {}

            name, instance = self.subscriptions[message_id]
            row.update(name=name, instance=instance)
            schema = self.formats[name]

            # 顶层末尾的填充可以省略。嵌套结构中的填充仍参与定位。
            end = len(schema)
            while end and schema[end - 1][2].startswith("_padding"):
                end -= 1

            values, _ = self.read_fields(schema[:end], payload, 2)
            row["timestamp_us"] = values["timestamp"]
            return values

        if tag in ("I", "M", "P", "Q"):
            key_start = 2 if tag in ("M", "Q") else 1
            value_start = key_start + payload[key_start - 1]
            if value_start > len(payload):
                raise ValueError("元数据键不完整")

            kind, count, name = parse_field(read_text(payload[key_start:value_start]))
            row["name"] = name
            value, end = self.read_value(kind, count, payload, value_start)
            if end != len(payload):
                raise ValueError("元数据值的长度与声明类型不符")

            result = {"type": kind, "array_length": count, "value": value}
            if tag == "M":
                result["continued"] = bool(payload[0])
            if tag == "Q":
                result["default_types_mask"] = payload[0]
            return result

        if tag in ("L", "C"):
            if tag == "C":
                level, log_tag, timestamp = struct.unpack_from("<BHQ", payload)
                message_start = 11
            else:
                level, timestamp = struct.unpack_from("<BQ", payload)
                log_tag = None
                message_start = 9

            row["timestamp_us"] = timestamp
            return {
                "level": LEVELS[level - 48] if 48 <= level <= 55 else None,
                "level_byte": level,
                "tag": log_tag,
                "message": read_text(payload[message_start:]),
            }

        if tag == "S":
            if payload != SYNC:
                raise ValueError("同步标记无效")
            return {"sync_magic_hex": payload.hex()}

        if tag == "O":
            duration, = struct.unpack("<H", payload)
            return {"duration_ms": duration}

        if tag == "B":
            return {
                "compatible_flags_hex": payload[:8].hex(),
                "incompatible_flags_hex": payload[8:16].hex(),
                "appended_offsets": struct.unpack_from("<3Q", payload, 16),
            }

        row["status"] = "unknown_type"
        return {}


def export_records(source, writer, header, size):
    decoder = Decoder()
    start, = struct.unpack_from("<Q", header, 8)
    section = "definitions"
    boundaries = []
    raw_only = False
    counts = {}

    def emit(row, decoded, raw):
        if "timestamp_us" in row:
            row["time_since_log_start_s"] = relative_seconds(
                row["timestamp_us"], start
            )
        row["data_json"] = json.dumps(
            decoded, ensure_ascii=False, allow_nan=False
        )
        row["raw_base64"] = base64.b64encode(raw).decode("ascii")
        writer.writerow(row)
        status = row["status"]
        counts[status] = counts.get(status, 0) + 1

    emit(
        {
            "offset": 0, "section": "header", "record_type": "HEADER",
            "timestamp_us": start,
            "status": "ok" if header[7] == 1 else "unknown_version",
        },
        {"version": header[7], "magic_hex": header[:7].hex(), "file_size_bytes": size},
        header,
    )

    while source.tell() < size:
        offset = source.tell()

        if raw_only:
            raw = source.read(min(16384, size - offset))
            if not raw:
                raise OSError("导出期间源文件提前结束")
            emit(
                {"offset": offset, "section": "raw",
                 "record_type": "RAW", "status": "raw_only"},
                {}, raw,
            )
            continue

        if boundaries and offset == boundaries[0]:
            boundaries.pop(0)
            section = "appended"

        limit = boundaries[0] if boundaries else size
        frame_header = source.read(min(3, limit - offset))
        if not frame_header:
            raise OSError("导出期间源文件提前结束")

        row = {"offset": offset, "section": section, "status": "ok"}
        if len(frame_header) < 3:
            row.update(record_type="RAW", status="truncated")
            emit(row, {"reason": "记录头不完整"}, frame_header)
            continue

        length, code = struct.unpack("<HB", frame_header)
        tag = chr(code) if 32 <= code <= 126 else f"0x{code:02x}"
        row["record_type"] = tag
        payload = source.read(min(length, limit - source.tell()))
        raw = frame_header + payload

        if len(payload) != length:
            row["status"] = "truncated"
            sync_at = raw.find(SYNC_FRAME, 1)
            if sync_at >= 0:
                source.seek(offset + sync_at)
                raw = raw[:sync_at]
                row["status"] = "resynced"
            emit(row, {"declared_payload_bytes": length}, raw)
            continue

        if section == "definitions" and tag in ("A", "L", "C"):
            section = "data"
            row["section"] = section

        try:
            decoded = decoder.decode(tag, payload, row)

            if tag == "B" and offset == len(header):
                incompatible_flags = int.from_bytes(payload[8:16], "little")
                if incompatible_flags & ~1:
                    row["status"] = "unsupported_incompatibility_flags"
                    raw_only = True
                elif incompatible_flags & 1:
                    offsets = [
                        value for value in decoded["appended_offsets"] if value
                    ]
                    boundaries = sorted({
                        value for value in offsets if source.tell() <= value <= size
                    })
                    if not offsets or offsets != boundaries:
                        row["status"] = "invalid_appended_offsets"

        except (KeyError, IndexError, ValueError, OverflowError,
                RecursionError, struct.error) as exc:
            row["status"] = "decode_error"
            decoded = {"error": str(exc)}

        emit(row, decoded, raw)

    return counts


def main():
    parser = argparse.ArgumentParser(description="将一份 PX4 ULog 导出为一份 CSV")
    parser.add_argument("ulg", type=Path, help="输入 ULog 文件")
    parser.add_argument("-o", "--output", type=Path, help="指定输出 CSV 路径")
    args = parser.parse_args()
    destination = args.output or args.ulg.with_suffix(".csv")

    try:
        if destination.suffix.lower() != ".csv":
            raise ValueError("输出路径必须以 .csv 结尾")

        with args.ulg.open("rb") as source:
            size = os.fstat(source.fileno()).st_size
            header = source.read(16)
            if len(header) != 16 or not header.startswith(MAGIC):
                raise ValueError("文件没有有效的明文 PX4 ULog 文件头")

            output = destination.open("x", encoding="utf-8-sig", newline="")
            try:
                with output:
                    writer = csv.DictWriter(output, fieldnames=COLUMNS)
                    writer.writeheader()
                    counts = export_records(source, writer, header, size)
            except BaseException:
                destination.unlink(missing_ok=True)
                raise

    except (OSError, ValueError) as exc:
        parser.exit(1, f"导出失败：{exc}\n")

    issues = {status: count for status, count in counts.items() if status != "ok"}
    print(f"已生成 {destination}，共 {sum(counts.values()):,} 条记录")
    if issues:
        print(f"需核对的记录：{issues}。原始字节已保存在 CSV 中。", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())