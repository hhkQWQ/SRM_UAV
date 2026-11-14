#!/usr/bin/env python3
"""Summarize time-aligned PnP position accuracy from existing observation logs."""
import argparse
import csv
import json
from pathlib import Path

import numpy as np


def values(rows, key):
    result=[]
    for row in rows:
        if row.get(key) in ('',None):
            continue
        try:value=float(row[key])
        except (TypeError,ValueError):continue
        if np.isfinite(value):result.append(value)
    return np.asarray(result,dtype=float)


def stats(rows, key):
    data=values(rows,key)
    if not len(data):return {'samples':0}
    return {'samples':len(data),'mean_m':float(data.mean()),
            'rmse_m':float(np.sqrt(np.mean(data**2))),
            'p95_m':float(np.percentile(data,95)),'max_m':float(data.max())}


def camera_axes(rows):
    errors=[]
    for row in rows:
        try:
            item=[float(row[f'p_camera_{axis}_m'])-float(row[f'true_camera_{axis}_m'])
                  for axis in ('x','y','z')]
        except (KeyError,TypeError,ValueError):continue
        if np.all(np.isfinite(item)):errors.append(item)
    if not errors:return {'samples':0}
    data=np.asarray(errors)
    return {'samples':len(data),'axes':'camera right/down/forward',
            'bias_xyz_m':data.mean(axis=0).tolist(),
            'rmse_xyz_m':np.sqrt(np.mean(data**2,axis=0)).tolist(),
            'error_std_xyz_m':data.std(axis=0).tolist()}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv',type=Path,nargs='?')
    parser.add_argument('--latest',action='store_true',help='Use newest pnp_sim_*.csv in sim/logs')
    parser.add_argument('--last-seconds',type=float,default=None,help='Last N simulation seconds in the log')
    args=parser.parse_args()
    if args.latest and args.csv:parser.error('Use a CSV path or --latest, not both')
    if args.last_seconds is not None and (not np.isfinite(args.last_seconds) or args.last_seconds<=0):
        parser.error('--last-seconds must be positive')
    if args.latest:
        logs=Path(__file__).resolve().parent.parent/'logs'
        candidates=list(logs.glob('pnp_sim_*.csv'))
        if not candidates:parser.error(f'No observation CSV found in {logs}')
        args.csv=max(candidates,key=lambda p:p.stat().st_mtime_ns)
    if args.csv is None:parser.error('Provide a CSV path or --latest')
    with args.csv.open(newline='') as stream:rows=list(csv.DictReader(stream))
    if args.last_seconds is not None:
        times=values(rows,'sim_time_s')
        if len(times):
            cutoff=float(times.max())-args.last_seconds
            selected=[]
            for row in rows:
                try:
                    if cutoff<=float(row['sim_time_s'])<=float(times.max()):selected.append(row)
                except (KeyError,ValueError,TypeError):continue
            rows=selected
    valid=sum(str(row.get('pose_valid','')).lower() in ('true','1') for row in rows)
    dt=values(rows,'truth_dt_us')
    camera=stats(rows,'camera_error_m')
    reprojection=values(rows,'reprojection_px')
    times=values(rows,'sim_time_s')
    report={'file':str(args.csv),'sampled_rows':len(rows),'valid_pose_rows':valid,
        'valid_pose_rate':valid/len(rows) if rows else None,
        'time_range_s':[float(times.min()),float(times.max())] if len(times) else None,
        'camera_position_error':camera,'camera_axis_error':camera_axes(rows),
        'relative_ned_error':stats(rows,'relative_error_m'),
        'max_abs_truth_time_difference_ms':float(np.max(np.abs(dt)))/1000 if len(dt) else None,
        'reprojection_p95_px':float(np.percentile(reprojection,95)) if len(reprojection) else None,
        'rotation_accuracy':'not measured by current CSV; requires rotation/normal ground-truth comparison'}
    print(json.dumps(report,indent=2,ensure_ascii=False))
    if not rows:print('没有可统计的记录；先在标靶可见的位置悬停并等待日志写入。')
    elif camera.get('samples',0)==0:
        print('没有有效的相机真值误差样本；检查检测结果和 Gazebo 位姿时间对齐，不能视为零误差。')


if __name__=='__main__':main()
