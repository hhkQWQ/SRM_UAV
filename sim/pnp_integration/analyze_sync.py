#!/usr/bin/env python3
"""Summarize TIMESYNC observations without assigning an unobserved root cause."""
import argparse
from collections import Counter
import json
import math
from pathlib import Path


def distribution(values):
    values = sorted(v for v in values if isinstance(v,(int,float)) and math.isfinite(v))
    return {'count':len(values),'max':values[-1] if values else None,
            'p95':values[round((len(values)-1)*.95)] if values else None}


def summarize(path):
    records,invalid = [],[]
    with Path(path).open() as stream:
        for line_number,line in enumerate(stream,1):
            try:
                record = json.loads(line)
                if not isinstance(record,dict):
                    raise ValueError('record is not an object')
                records.append(record)
            except (ValueError,TypeError):
                invalid.append(line_number)
    config = next((r for r in records if r.get('record_type') == 'config'),{})
    events = [r for r in records if r.get('record_type') == 'event']
    snapshots = [r for r in records if r.get('record_type') == 'snapshot']
    contexts = [r.get('context') for r in snapshots if isinstance(r.get('context'),dict)]
    outcomes = Counter(r.get('event') for r in events)
    limits = config.get('limits',{})
    requests = {}
    reply_samples,queue_waits,timeouts = [],[],[]
    since_accept = Counter()
    recent = []
    last_accept = None
    observations = []
    for record in events:
        event,token = record.get('event'),record.get('token_ns')
        if event in ('REQUEST_QUEUED','REQUEST_QUEUE_FULL'):
            requests[token] = {'queued':event == 'REQUEST_QUEUED','tx':None,'matched_reply':False}
        elif event == 'REQUEST_TX':
            request = requests.setdefault(token,{'queued':None,'tx':None,'matched_reply':False})
            if record.get('bytes_before') == 0:
                request['tx'] = record
                queue_waits.append(record.get('queue_wait_us'))
        if event in ('ACCEPT_INITIAL','ACCEPT_FILTER','ACCEPT_RESET','REJECT_RTT','REJECT_JITTER'):
            request = requests.get(token)
            if request is not None:
                request['matched_reply'] = True
            item = {key:record.get(key) for key in (
                'event','event_sequence','local_us','wall_us','rtt_us','sample_offset_us','innovation_us','poll_gap_us')}
            tx = request.get('tx') if request else None
            item['queue_wait_us'] = tx.get('queue_wait_us') if tx else None
            item['elapsed_after_write_us'] = (record['local_us']-tx['local_us'] if tx else None)
            reply_samples.append(item)
            recent.append(item)
            recent = recent[-8:]
        if event and event.startswith('ACCEPT_'):
            last_accept = record
            since_accept.clear()
        elif event in ('REJECT_RTT','REJECT_JITTER','REPLY_NO_PENDING','REPLY_TOKEN_MISMATCH',
                       'REQUEST_QUEUE_FULL','REQUEST_REPLACED'):
            since_accept[event] += 1
        if event == 'SYNC_STATUS' and record.get('reason') == 'ACCEPT_TIMEOUT':
            timeouts.append({'local_us':record.get('local_us'),'wall_us':record.get('wall_us'),
                'event_sequence':record.get('event_sequence'),'sync_age_us':record.get('sync_age_us'),
                'last_accept_event_sequence':last_accept.get('event_sequence') if last_accept else None,
                'outcomes_since_last_accept':dict(since_accept),'recent_reply_outcomes':list(recent),
                'drain_context':record.get('drain_context')})
    if outcomes['REJECT_JITTER']:
        observations.append('已观测到偏移创新量超过滤波阈值而被拒收；是否导致每次超时请检查timeouts中的时间顺序。')
    if outcomes['REJECT_RTT']:
        observations.append('已观测到请求创建至解析使用时间戳的往返间隔被判超限；该间隔包含发送排队和接收轮询延迟。')
    delayed = sum(v > limits.get('max_rtt_us',math.inf) for v in queue_waits if v is not None)
    if delayed:
        observations.append('有%d个请求仅等待首次write的耗时已超过最大RTT；有发送排队延迟的直接证据。' % delayed)
    if outcomes['REPLY_TOKEN_MISMATCH'] or outcomes['REPLY_NO_PENDING']:
        observations.append('已观测到无法匹配当前请求的回复；需按token核对旧请求、延迟回复与请求替换。')
    if outcomes['ACCEPT_RESET'] or outcomes['BOOT_BACKWARD'] or outcomes['LOCAL_CLOCK_BACKWARD']:
        observations.append('已记录偏移重置或时间回退；事件本身不能区分真实时钟变化与时间戳处理偏差。')
    dropped = max((r.get('events_dropped',0) for r in records),default=0)
    if dropped or invalid:
        observations.append('日志存在缓冲丢事件或无效行，不能把缺少事件直接解释为缺少通信。')
    if not events:
        observations.append('没有同步事件，无法从此文件判定失败原因。')
    return {'file':str(Path(path).resolve()),'diagnostic_only':True,
        'record_count':len(records),'invalid_line_count':len(invalid),'invalid_line_examples':invalid[:10],
        'events_dropped':dropped,'limits':limits,'event_counts':dict(outcomes),
        'rtt_us':distribution([r.get('rtt_us') for r in reply_samples]),
        'queue_wait_us':distribution(queue_waits),'queue_wait_exceeds_max_rtt_count':delayed,
        'requests_without_write_observation_at_file_end':sum(
            r['queued'] is True and r['tx'] is None for r in requests.values()),
        'written_requests_without_matched_reply_at_file_end':sum(
            r['tx'] is not None and not r['matched_reply'] for r in requests.values()),
        'latest_snapshot':snapshots[-1] if snapshots else None,'timeouts':timeouts,
        'nav_states':dict(Counter(c.get('nav',c.get('control_state')) for c in contexts)),
        'flight_reasons':dict(Counter(c.get('flight',c.get('flight_reason')) for c in contexts)),
        'latest_context':contexts[-1] if contexts else None,
        'observations':observations,
        'limitation':'REQUEST_TX observes the write API; replies observe parser processing. Unfinished requests at file end may still be in flight. Missing replies cannot by themselves distinguish link loss, peer silence, or delayed processing. This report does not declare a unique root cause.'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('log',nargs='?')
    parser.add_argument('--latest',action='store_true')
    parser.add_argument('--log-dir',default=str(Path(__file__).resolve().parents[1]/'logs'))
    parser.add_argument('--output')
    args = parser.parse_args()
    if args.latest:
        candidates = list(Path(args.log_dir).glob('pnp_sim_*_sync.jsonl'))
        candidates += list(Path(args.log_dir).glob('pnp_sync_*.jsonl'))
        if not candidates:
            parser.error('No synchronization diagnostic logs found; restart the instrumented simulation first')
        path = max(candidates,key=lambda p:p.stat().st_mtime_ns)
    elif args.log:
        path = Path(args.log)
    else:
        parser.error('Supply a diagnostic JSONL path or --latest')
    report = json.dumps(summarize(path),indent=2,ensure_ascii=False)
    if args.output:
        output = Path(args.output)
        if output.resolve() == path.resolve():
            parser.error('--output cannot overwrite the input log')
        output.write_text(report+'\n')
    print(report)


if __name__ == '__main__':
    main()
