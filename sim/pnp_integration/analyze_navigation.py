#!/usr/bin/env python3
"""Describe actual Offboard runs and check sampled arrival using PnP tolerances."""
import argparse
import json
import math
from pathlib import Path

from analyze_log import counts,flag,number,read_rows,run_settings,select_csv,stats,vector


def navigating(row):
    return (flag(row,'armed') and flag(row,'offboard') and flag(row,'configured')
            and row.get('flight')=='OK' and flag(row,'setpoint_submitted')
            and row.get('control_track') in ('MEASURED','PREDICTED')
            and row.get('nav') in ('CENTERING','ALIGN','PREDICT','HOLD','TARGET_CHANGED'))


def offboard_runs(rows):
    runs=[]
    current=[]
    for row in rows:
        if current and number(row,'capture_time_us')<number(current[-1],'capture_time_us'):
            runs.append(current)
            current=[]
        if flag(row,'armed') and flag(row,'offboard'):
            current.append(row)
        elif current:
            runs.append(current)
            current=[]
    if current:
        runs.append(current)
    return runs


def truth_error(row,target_height):
    position=vector(row,'true_p_local')
    if position is None or target_height is None:
        return None,None
    return math.hypot(position[0],position[1]),position[2]-target_height


def summarize(path,final_seconds=10.):
    rows,quality=read_rows(path)
    report={'file':str(Path(path).resolve()),'passed':False,'log_quality':quality,
            'sampled_rows':len(rows),'track_states':counts(rows,'track'),'nav_states':counts(rows,'nav')}
    if not rows:
        return dict(report,reason='NO_SAMPLES')
    parameters=run_settings(path)
    report['PnP_parameters']=parameters
    runs=offboard_runs(rows)
    report['offboard_runs']=[{'start_s':number(run[0],'capture_time_us')*1e-6,
                              'end_s':number(run[-1],'capture_time_us')*1e-6,
                              'samples':len(run)} for run in runs]
    report['current_offboard']=flag(rows[-1],'offboard')
    if not runs:
        return dict(report,reason='NO_OFFBOARD_SAMPLES')
    run=runs[-1]
    end=number(run[-1],'capture_time_us')
    final=[r for r in run if number(r,'capture_time_us')>=end-final_seconds*1e6]
    errors=[truth_error(row,parameters['TARGET_HEIGHT_M']) for row in final]
    stamps=[number(row,'capture_time_us') for row in final]
    gaps=[(b-a)*1e-6 for a,b in zip(stamps,stamps[1:]) if b>a]
    report.update(evaluated_run=report['offboard_runs'][-1],
                  evaluated_window_s=[stamps[0]*1e-6,stamps[-1]*1e-6],
                  final_sample_count=len(final),max_sample_gap_s=max(gaps) if gaps else None,
                  navigation_rows=sum(navigating(r) for r in final),
                  navigation_rate=sum(navigating(r) for r in final)/len(final),
                  final_track_states=counts(final,'track'),final_nav_states=counts(final,'nav'),
                  final_flight_reasons=counts(final,'flight'),
                  final_horizontal_error_m=stats([h for h,z in errors]),
                  final_absolute_height_error_m=stats([abs(z) for h,z in errors if z is not None]))
    required=('TARGET_HEIGHT_M','ARRIVAL_XY_M','ARRIVAL_Z_M','ARRIVAL_DWELL_MS','log_interval_ms')
    if any(parameters[k] is None or not math.isfinite(parameters[k]) or parameters[k]<=0 for k in required):
        return dict(report,reason='INVALID_PNP_PARAMETERS')
    # Sampling continuity is derived from the recorded logging interval, not a new flight parameter.
    maximum_gap=2*parameters['log_interval_ms']*1000
    start=None
    previous=None
    for row,(horizontal,vertical) in zip(final,errors):
        stamp=number(row,'capture_time_us')
        arrived=(navigating(row) and row.get('control_track')=='MEASURED' and row.get('nav')=='HOLD'
                 and horizontal is not None and vertical is not None
                 and horizontal<=parameters['ARRIVAL_XY_M'] and abs(vertical)<=parameters['ARRIVAL_Z_M'])
        if not arrived:
            start=None
        elif start is None or previous is None or not 0<stamp-previous<=maximum_gap:
            start=stamp
        previous=stamp
    duration=0. if start is None else (stamps[-1]-start)*.001
    report['sampled_truth_hold_ms']=duration
    report['passed']=bool(duration>=parameters['ARRIVAL_DWELL_MS'])
    report['reason']='SAMPLED_TRUTH_HOLD' if report['passed'] else 'TRUTH_HOLD_NOT_CONFIRMED'
    if not quality['control_input_recorded']:
        report['reason']='CONTROL_INPUT_NOT_RECORDED'
        report['passed']=False
    report['criterion']='Last recorded Offboard run: consecutive sampled HOLD, MEASURED control input and truth within PnP ARRIVAL_XY_M/ARRIVAL_Z_M for ARRIVAL_DWELL_MS; gaps <=2 logging intervals.'
    report['notes']=['PREDICTED is valid navigation input; arrival confirmation follows PnP and needs MEASURED.',
                     'This evaluates recorded samples at the end of the selected run, not flight-controller execution ACK or unobserved motion.']
    if parameters['source']=='current_PnP_config_fallback':
        report['notes'].append('历史参数未完整记录，分析使用当前PnP配置；结论须结合当时配置核对。')
    if not report['final_horizontal_error_m']['samples']:
        report['notes'].append('缺少对齐真值，不作到位成功判断。')
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv',nargs='?')
    parser.add_argument('--latest',action='store_true')
    parser.add_argument('--log-dir',default=str(Path(__file__).resolve().parents[1]/'logs'))
    parser.add_argument('--final-seconds',type=float,default=10.)
    parser.add_argument('--output')
    args=parser.parse_args()
    if not math.isfinite(args.final_seconds) or args.final_seconds<=0:
        parser.error('--final-seconds must be positive')
    try:
        path=select_csv(args.csv,args.latest,args.log_dir)
        if args.output and Path(args.output).resolve() in (path.resolve(),path.with_suffix('.json').resolve()):
            parser.error('--output cannot overwrite the input or its run metadata')
        report=summarize(path,args.final_seconds)
    except (OSError,ValueError) as error:
        parser.error(str(error))
    text=json.dumps(report,indent=2,ensure_ascii=False,allow_nan=False)
    if args.output:
        Path(args.output).write_text(text+'\n')
    print(text)


if __name__=='__main__':
    main()
