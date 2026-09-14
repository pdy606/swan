#!/usr/bin/env python3
"""Plot recorded simulation driving evidence (requires matplotlib)."""
import argparse,csv,json,math
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import Polygon


def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('run',type=Path);args=ap.parse_args()
    root=Path(__file__).resolve().parents[1];run=args.run
    report=json.loads((run/'result.json').read_text())
    rows=list(csv.DictReader((run/'trajectory.csv').open()))
    obstacles=json.loads((run/'obstacles.json').read_text())
    route=json.loads((run/'layout.json').read_text())['route']
    font=Path('/System/Library/Fonts/AppleSDGothicNeo.ttc')
    if font.exists():
        font_manager.fontManager.addfont(str(font));plt.rcParams['font.family']=font_manager.FontProperties(fname=str(font)).get_name()
    plt.rcParams['axes.unicode_minus']=False
    x=[float(r['x']) for r in rows];y=[float(r['y']) for r in rows]
    t=[float(r['sim_s'])-float(rows[0]['sim_s']) for r in rows]
    fig,(ax,speed,clear)=plt.subplots(3,1,figsize=(15,8),gridspec_kw={'height_ratios':[1.6,1,1]},facecolor='#faf7ef')
    for o in obstacles:ax.add_patch(Polygon(o['vertices'],facecolor='#b8b2a6',edgecolor='#797367',lw=.3))
    px,py=zip(*route);ax.plot(px,py,'--',color='#348582',label='계획 경로',lw=3)
    ax.plot(x,y,color='#d0662d',label='Gazebo 실제 위치',lw=1.8)
    ax.scatter([x[0],x[-1]],[y[0],y[-1]],c=['#348582','#d0662d'],s=60,zorder=5)
    for start,end,label in [(9.2,10.8,'좌판 병목'),(15.5,18.5,'쇼핑객 구간'),(18.5,20.8,'오토바이 하역')]:
        ax.axvspan(start,end,color='#348582',alpha=.07)
        ax.text((start+end)/2,3.3,label,ha='center',fontsize=10)
    ax.set(xlim=(-8,35),ylim=(-3.5,4),xlabel='X (m)',ylabel='Y (m)');ax.set_aspect('equal');ax.legend(loc='upper left')
    speed.plot(t,[float(r['odom_v']) for r in rows],color='#348582',label='선속도 /odom (m/s)')
    speed.plot(t,[float(r['odom_w']) for r in rows],color='#d0662d',label='각속도 /odom (rad/s)')
    speed.set(ylabel='속도');speed.legend(loc='upper left',ncol=2)
    margins=[float(r['margin_m']) for r in rows]
    clear.plot(t,[m if math.isfinite(m) else float('nan') for m in margins],color='#348582')
    clear.axhline(.04,color='#be593e',ls='--',label='시험 중단 기준 0.04m')
    clear.set(xlabel='기록 시작 후 시뮬레이션 시간 (s)',ylabel='검사 원 여유 (m)',ylim=(0,2));clear.legend(loc='upper right')
    for axis in (ax,speed,clear):
        axis.set_facecolor('#faf7ef');axis.spines[['top','right']].set_visible(False)
        axis.grid(alpha=.15)
    fig.suptitle(f"휠체어 실제 주행 검증 · {report['result'].upper()}",x=.08,ha='left',fontsize=22)
    fig.text(.08,.925,'Gazebo 실제 위치로 경로 추종 · 고정 장애물 · 접촉 센서 및 Nav2 검증 아님',fontsize=12,color='#66645e')
    fig.tight_layout(rect=(0,0,1,.90));fig.savefig(run/'driving_report.png',dpi=150);plt.close(fig)
    print('Saved',run/'driving_report.png')


if __name__=='__main__':main()
