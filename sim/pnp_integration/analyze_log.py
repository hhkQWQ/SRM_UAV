#!/usr/bin/env python3
"""Read PnP fields and summarize measured pose accuracy; no flight control."""
import argparse
import ast
from collections import Counter
import csv
import json
import math
from pathlib import Path
import runpy


def number(row,key):
    try:
        value=float(row[key])
        return value if math.isfinite(value) else None
    except (KeyError,TypeError,ValueError):
        return None


def flag(row,key):
    return str(row.get(key,'')).lower() in ('1','true')


def vector(row,prefix):
    values=tuple(number(row,f'{prefix}_{axis}_m') for axis in ('x','y','z'))
    return None if any(v is None for v in values) else values


def percentile(values,fraction=.95):
    if not values:
        return None
    ordered=sorted(values)
    index=(len(ordered)-1)*fraction
    lower=int(index)
    upper=min(lower+1,len(ordered)-1)
    return ordered[lower]+(ordered[upper]-ordered[lower])*(index-lower)


def stats(values):
    values=[v for v in values if v is not None and math.isfinite(v)]
    if not values:
        return {'samples':0}
    return {'samples':len(values),'mean':sum(values)/len(values),
            'rmse':math.sqrt(sum(v*v for v in values)/len(values)),
            'p95':percentile(values),'max':max(values)}


def counts(rows,key):
    return dict(Counter(row.get(key,'') for row in rows))


def read_rows(path):
    """Normalize historical names only when reading; never rewrite logs."""
    aliases={'frame_id':'frame','track':'track_state','nav':'control_state',
             'flight':'flight_reason','setpoint_submitted':'control_submitted'}
    for axis,local in zip(('x','y','z'),('n','e','d')):
        aliases.update({f'p_C_{axis}_m':f'p_camera_{axis}_m',
                        f'p_local_raw_{axis}_m':f'relative_{local}_m',
                        f'true_p_C_{axis}_m':f'true_camera_{axis}_m',
                        f'true_p_local_{axis}_m':f'true_relative_{local}_m',
                        f'command_v{local}_mps':f'control_v{local}_mps'})
    rows=[]
    skipped=resets=duplicates=0
    previous=None
    with Path(path).open(newline='') as stream:
        reader=csv.DictReader(stream)
        fields=reader.fieldnames or []
        schema=('pnp' if 'capture_time_us' in fields else 'legacy_sim')
        for row in reader:
            if None in row or any(value is None for value in row.values()):
                skipped+=1
                continue
            for new,old in aliases.items():
                if new not in row and old in row:
                    row[new]=row[old]
            for key in ('track','control_track'):
                if row.get(key)=='CENTER_ONLY':row[key]='ACQUIRING'
            if 'capture_time_us' not in row:
                seconds=number(row,'sim_time_s')
                if seconds is not None:
                    row['capture_time_us']=seconds*1e6
            stamp=number(row,'capture_time_us')
            if stamp is None or stamp<0:
                skipped+=1
                continue
            if 'vision_enabled' not in row:
                row['vision_enabled']='1'  # Old sim always ran detection.
            if previous is not None:
                resets+=stamp<previous
                duplicates+=stamp==previous
            previous=stamp
            rows.append(row)
    return rows,{'schema':schema,'skipped_rows':skipped,'clock_resets':resets,
                 'duplicate_timestamps':duplicates,
                 'control_input_recorded':'control_track' in fields}


def last_epoch(rows):
    start=0
    for i in range(1,len(rows)):
        if number(rows[i],'capture_time_us')<number(rows[i-1],'capture_time_us'):
            start=i
    return rows[start:]


def select_csv(path=None,latest=False,log_dir=None):
    if latest and path:
        raise ValueError('Choose CSV or --latest, not both')
    if latest:
        directory=Path(log_dir) if log_dir else Path(__file__).resolve().parents[1]/'logs'
        candidates=list(directory.glob('pnp_sim_*.csv'))
        if not candidates:
            raise ValueError('No simulation CSV logs found')
        return max(candidates,key=lambda item:item.stat().st_mtime_ns)
    if path:
        return Path(path)
    raise ValueError('Supply CSV or --latest')


def run_settings(path):
    """Use recorded PnP values; old logs fall back to the current source."""
    directory=Path(__file__).resolve().parents[2]/'PnP'
    current=runpy.run_path(str(directory/'config.py'))
    metadata_path=Path(path).with_suffix('.json')
    try:
        metadata=json.loads(metadata_path.read_text())
    except (OSError,ValueError):
        metadata={}
    if not isinstance(metadata,dict):
        metadata={}
    recorded=metadata.get('control_config',{})
    if not isinstance(recorded,dict):
        recorded={}
    names=('TARGET_HEIGHT_M','ARRIVAL_XY_M','ARRIVAL_Z_M','ARRIVAL_DWELL_MS')
    values={key:recorded.get(key,current.get(key)) for key in names}
    interval=metadata.get('log_interval_ms')
    if interval is None:
        tree=ast.parse((directory/'main.py').read_text())
        assignment=next(node for node in tree.body if isinstance(node,ast.Assign)
                        and any(isinstance(t,ast.Name) and t.id=='POSE_LOG_INTERVAL_MS' for t in node.targets))
        interval=ast.literal_eval(assignment.value)
    values['log_interval_ms']=interval
    for key in (*names,'log_interval_ms'):
        try:
            values[key]=float(values[key])
            if not math.isfinite(values[key]):values[key]=None
        except (TypeError,ValueError):
            values[key]=None
    values['source']=str(metadata_path) if all(key in recorded for key in names) else 'current_PnP_config_fallback'
    return values


def error_vectors(rows,estimate,truth,validity):
    errors=[]
    for row in rows:
        actual,expected=vector(row,estimate),vector(row,truth)
        if (validity is None or validity not in row or flag(row,validity)) and actual is not None and expected is not None:
            errors.append(tuple(a-b for a,b in zip(actual,expected)))
    return errors


def error_report(errors):
    result=stats([math.sqrt(sum(v*v for v in item)) for item in errors])
    if errors:
        result['bias_xyz_m']=[sum(item[i] for item in errors)/len(errors) for i in range(3)]
        result['rmse_xyz_m']=[math.sqrt(sum(item[i]**2 for item in errors)/len(errors)) for i in range(3)]
    return result


def summarize(path,last_seconds=None):
    rows,quality=read_rows(path)
    if last_seconds is not None and rows:
        rows=last_epoch(rows)
        cutoff=number(rows[-1],'capture_time_us')-last_seconds*1e6
        rows=[r for r in rows if number(r,'capture_time_us')>=cutoff]
    vision=[r for r in rows if flag(r,'vision_enabled')]
    valid=sum(flag(r,'pose_valid') for r in vision)
    camera=error_report(error_vectors(vision,'p_C','true_p_C','pose_valid'))
    raw=error_report(error_vectors(vision,'p_local_raw','true_p_local','relative_valid'))
    filtered=error_report(error_vectors([r for r in vision if r.get('track') in ('MEASURED','PREDICTED')],
                                        'p_local','true_p_local',None))
    notes=[]
    if not rows:
        notes.append('没有完整的有效时间戳记录。')
    if not vision:
        notes.append('记录处于待命，没有识别样本；IDLE不计入识别失败率。')
    if not camera['samples']:
        notes.append('没有有效PnP与对应真值，不能将缺少误差样本视为零误差。')
    if quality['schema']!='pnp':
        notes.append('历史字段仅在读取时转换；未记录的控制端状态不补造。')
    if quality['clock_resets']:
        notes.append('存在时钟回退；--last-seconds只选择最后一个时间段。')
    return {'file':str(Path(path).resolve()),'log_quality':quality,'sampled_rows':len(rows),
            'vision_rows':len(vision),'valid_pose_rows':valid,
            'valid_pose_rate':valid/len(vision) if vision else None,
            'capture_time_range_s':([number(rows[0],'capture_time_us')*1e-6,
                                      number(rows[-1],'capture_time_us')*1e-6] if rows else None),
            'track_states':counts(rows,'track'),'nav_states':counts(rows,'nav'),
            'flight_reasons':counts(rows,'flight'),
            'camera_position_error_m':camera,'raw_relative_error_m':raw,
            'filtered_relative_error_m':filtered,
            'reprojection_px':stats([number(r,'reprojection_px') for r in vision if flag(r,'pose_valid')]),
            'solve_ms':stats([number(r,'solve_ms') for r in vision]),
            'observation_age_ms':stats([number(r,'observation_age_ms') for r in vision]),
            'absolute_truth_dt_us':stats([abs(number(r,'truth_dt_us')) for r in vision if number(r,'truth_dt_us') is not None]),
            'notes':notes}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv',nargs='?')
    parser.add_argument('--latest',action='store_true')
    parser.add_argument('--last-seconds',type=float)
    args=parser.parse_args()
    if args.last_seconds is not None and (not math.isfinite(args.last_seconds) or args.last_seconds<=0):
        parser.error('--last-seconds must be positive')
    try:
        path=select_csv(args.csv,args.latest)
        report=summarize(path,args.last_seconds)
    except (OSError,ValueError) as error:
        parser.error(str(error))
    print(json.dumps(report,indent=2,ensure_ascii=False,allow_nan=False))


if __name__=='__main__':
    main()
