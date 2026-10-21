#!/usr/bin/env python3
"""Plot all GNSS odometry samples in the three preserved brake recordings."""
import hashlib
import json
import os
from pathlib import Path

os.environ.setdefault('MPLCONFIGDIR','/tmp/brake-full-route-matplotlib')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
import numpy as np

HERE=Path(__file__).resolve().parent
SESSIONS=[(10,'20261021T123628-29c20b2f'),(20,'20261021T124850-ec2f774f'),(30,'20261021T130403-80ac952d')]
font_manager.fontManager.addfont('/usr/share/fonts/truetype/nanum/NanumGothic.ttf')
plt.rcParams.update({'font.family':'NanumGothic','axes.unicode_minus':False,'font.size':10})


def ns(stamp):
    return stamp['sec']*10**9+stamp['nanosec']


def read_recording(speed,name):
    path=HERE/name/'messages.jsonl'
    raw=path.read_bytes()
    rows=[json.loads(line) for line in raw.splitlines()]
    brake=next(json.loads(row['data']['data']) for row in rows
               if row['topic']=='/vehicle/brake_test'
               and json.loads(row['data']['data']).get('event')=='brake'
               and json.loads(row['data']['data']).get('reason')=='target_reached')
    command_ns=brake['command_ns']
    stop=next(row['data'] for row in rows if row['topic']=='/vehicle/stop_status'
              and row['data']['stopped'] and ns(row['data']['stopped_since'])>command_ns
              and abs(ns(row['data']['command_stamp'])-command_ns)<50_000_000)
    odometry=sorted([row for row in rows if row['topic']=='/gnss/odom'],key=lambda row:row['source_stamp_ns'])
    stamps=np.array([row['source_stamp_ns'] for row in odometry],dtype=np.int64)
    assert np.all(np.diff(stamps)>0)
    t=(stamps-command_ns)/1e9
    xy=np.array([[row['data']['pose']['pose']['position'][axis] for axis in ['x','y']] for row in odometry])
    xy-=xy[0].copy()  # Translation only; keep east/north orientation and metric scale.
    stop_s=(ns(stop['stopped_since'])-command_ns)/1e9
    def interpolate(time):
        return np.array([np.interp(time,t,xy[:,axis]) for axis in [0,1]])
    return {'target_kph':speed,'recording':name,'source_sha256':hashlib.sha256(raw).hexdigest(),
            't':t,'xy':xy,'command_xy':interpolate(0),'stop_xy':interpolate(stop_s),
            'stop_s':stop_s,'point_count':len(odometry),'duration_s':float(t[-1]-t[0])}


def main():
    records=[read_recording(*session) for session in SESSIONS]
    fig,axes=plt.subplots(2,2,figsize=(14,12),layout='constrained')
    overview=axes.flat[0]
    for record,color in zip(records,['#2476a2','#d78a00','#8f49a7']):
        xy=record['xy']
        overview.plot(*xy.T,'.-',color=color,lw=1.4,ms=1.4,alpha=.8,
                      label=f"{record['target_kph']} km/h 시험 전체")
        overview.scatter(*record['command_xy'],marker='D',s=55,facecolors='white',edgecolors=color,zorder=4)
        overview.scatter(*record['stop_xy'],marker='X',s=65,color=color,zorder=5)
    overview.scatter(0,0,marker='^',s=75,color='#208750',zorder=6,label='각 기록 시작점')
    overview.plot([],[],marker='D',mfc='white',mec='#555555',ls='none',label='제동 명령 위치')
    overview.plot([],[],marker='X',color='#555555',ls='none',label='정지 구간 시작 위치')
    overview.set_title('전체 궤적 겹쳐 비교 — 시작점만 원점으로 이동')
    overview.legend(fontsize=9,loc='upper left')
    for ax,record in zip(list(axes.flat)[1:],records):
        xy,t=record['xy'],record['t']
        before=np.vstack([xy[t<0],record['command_xy']])
        braking=np.vstack([record['command_xy'],xy[(t>=0)&(t<=record['stop_s'])],record['stop_xy']])
        after=np.vstack([record['stop_xy'],xy[t>record['stop_s']]])
        ax.plot(*before.T,'.-',color='#2476a2',ms=2,lw=1.5,label='기록 시작 ~ 제동 명령')
        ax.plot(*braking.T,'.-',color='#d36932',ms=3,lw=2,label='제동 명령 ~ 정지')
        ax.plot(*after.T,'.-',color='#90969b',ms=2,lw=1,alpha=.7,label='정지 이후 기록')
        ax.scatter(*xy[0],marker='^',s=65,color='#208750',zorder=4,label='기록 시작')
        ax.scatter(*xy[-1],marker='s',s=100,facecolors='none',edgecolors='#222222',lw=1.4,zorder=5,label='기록 끝')
        ax.scatter(*record['command_xy'],marker='D',s=55,color='#d78a00',edgecolors='white',zorder=6,label='제동 명령')
        ax.scatter(*record['stop_xy'],marker='X',s=55,color='#b83238',zorder=7,label='정지 구간 시작')
        ax.set_title(f"{record['target_kph']} km/h — 전체 {record['duration_s']:.1f}초 / {record['point_count']:,}점")
        ax.legend(fontsize=8,loc='best')
    for ax in axes.flat:
        ax.set_aspect('equal',adjustable='datalim')
        ax.set_xlabel('기록 시작점 대비 동쪽 거리 (m)')
        ax.set_ylabel('기록 시작점 대비 북쪽 거리 (m)')
        ax.grid(alpha=.2)
    fig.suptitle('10·20·30 km/h 시험 — 전체 주행 GPS 궤적 비교',fontsize=18,weight='bold')
    fig.supxlabel('각 기록의 모든 /gnss/odom 위치 사용: 대기·주행·제동·정지 이후 포함\n'
                  '기록별 시작점을 (0,0)으로 평행 이동 · 동서/남북 방향과 거리 비율 유지 · 실제 출발 위치가 같다는 의미는 아님',fontsize=10)
    output=HERE/'braking_full_routes_comparison.png'
    fig.savefig(output,dpi=170,facecolor='white')
    plt.close(fig)
    inputs=[{key:value for key,value in record.items() if key not in ['xy','t','command_xy','stop_xy']}
            for record in records]
    images=[]
    for path in [HERE/'braking_comparison.png',HERE/'braking_waypoints.png',output]:
        images.append({'path':path.name,'sha256':hashlib.sha256(path.read_bytes()).hexdigest()})
    (HERE/'figure_manifest.json').write_text(json.dumps({
        'full_route_inputs':inputs,'coordinate_method':'Translate each recording first odom XY to zero; preserve ENU direction, scale and all samples.',
        'copied_analysis_images_source':'test_runs/braking-distance-analysis-20261022/out',
        'images':images,
    },ensure_ascii=False,indent=2)+'\n')
    print(output)


if __name__=='__main__':
    main()
