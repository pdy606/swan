#!/usr/bin/env python3
"""Static clearance check and previews from the generated SDF (not a Gazebo render).

Authoring dependencies: numpy, matplotlib, shapely. No ROS required.
"""
import ast
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import Polygon as PatchPolygon, Circle
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np
from shapely.geometry import MultiPoint, LineString, Point

ROOT = Path(__file__).resolve().parents[1]


def transform(pose):
    x,y,z,r,p,yaw = [float(v) for v in (pose or '0 0 0 0 0 0').split()]
    cr,sr,cp,sp,cy,sy=math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(yaw),math.sin(yaw)
    t=np.eye(4)
    t[:3,:3]=np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],
                       [sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]])
    t[:3,3]=[x,y,z]
    return t


def vertices(geom):
    b=geom.find('box')
    if b is not None:
        sx,sy,sz=[float(x)/2 for x in b.findtext('size').split()]
        v=np.array([[x,y,z] for x in (-sx,sx) for y in (-sy,sy) for z in (-sz,sz)])
        faces=[[0,1,3,2],[4,6,7,5],[0,4,5,1],[2,3,7,6],[0,2,6,4],[1,5,7,3]]
        return v,faces
    c=geom.find('cylinder')
    if c is not None:
        radius=float(c.findtext('radius'));h=float(c.findtext('length'))/2
        n=12
        v=np.array([[radius*math.cos(a),radius*math.sin(a),z] for z in (-h,h) for a in np.linspace(0,2*math.pi,n,endpoint=False)])
        faces=[list(range(n)),list(range(n,2*n))]+[[i,(i+1)%n,(i+1)%n+n,i+n] for i in range(n)]
        return v,faces
    s=geom.find('sphere')
    if s is not None:
        radius=float(s.findtext('radius'));n=10;layers=6
        v=np.array([[radius*math.cos(a)*math.sin(p),radius*math.sin(a)*math.sin(p),radius*math.cos(p)]
                    for p in np.linspace(0,math.pi,layers) for a in np.linspace(0,2*math.pi,n,endpoint=False)])
        faces=[[j*n+i,j*n+(i+1)%n,(j+1)*n+(i+1)%n,(j+1)*n+i] for j in range(layers-1) for i in range(n)]
        return v,faces
    return None,None


def shapes(world,kind):
    for m in world.findall('model'):
        mt=transform(m.findtext('pose'))
        for link in m.findall('link'):
            lt=mt@transform(link.findtext('pose'))
            for obj in link.findall(kind):
                v,faces=vertices(obj.find('geometry'))
                if v is None:continue
                t=lt@transform(obj.findtext('pose'))
                vv=(np.column_stack((v,np.ones(len(v))))@t.T)[:,:3]
                color=[float(x) for x in obj.findtext('material/diffuse','.55 .55 .55 1').split()][:3]
                yield m.get('name'),obj.get('name'),vv,faces,color


def main():
    world=ET.parse(ROOT.parent/'wheelchair_gazebo/worlds/market_shopping.world').getroot().find('world')
    config=json.loads((ROOT/'config/market_layout.json').read_text())
    preview=ROOT/'preview';preview.mkdir(exist_ok=True)
    for p in ROOT.rglob('*.py'):ast.parse(p.read_text())
    for p in ROOT.rglob('*'):
        if p.suffix in ('.sdf','.dae','.xml','.config'):ET.parse(p)
    names=[m.get('name') for m in world.findall('model')]
    assert len(names)==len(set(names)), 'Duplicate world model names'
    for link in world.findall('.//link'):
        children=link.findall('collision')+link.findall('visual')
        child_names=[child.get('name') for child in children]
        assert len(child_names)==len(set(child_names)), f'Duplicate collision/visual names in {link.get("name")}'
    for texture in world.findall('.//albedo_map'):
        asset=ROOT/'models'/texture.text.removeprefix('model://')
        assert asset.is_file(),asset
    for uri in world.findall('.//mesh/uri'):
        asset=ROOT/'models'/uri.text.removeprefix('model://')
        assert asset.is_file(),asset
        mesh=ET.parse(asset)
        for init in mesh.findall('.//{*}image/{*}init_from'):
            assert (asset.parent/init.text).is_file(),init.text
    # Enclose the local wheelchair, including the protruding LiDAR collision.
    # This is a static geometric check, not a dynamics/Nav2 certification.
    route=LineString(config['route']);radius=.75
    robot_path=ROOT.parent/'wheelchair_gazebo/models/wheelchair/model.sdf'
    robot_world=ET.Element('world')
    robot_world.append(ET.parse(robot_path).getroot().find('model'))
    robot_radius=max(float(np.linalg.norm(v[:,:2],axis=1).max())
                     for _,_,v,_,_ in shapes(robot_world,'collision'))
    assert robot_radius <= radius, f'Wheelchair radius {robot_radius} exceeds probe {radius}'
    moving=set(config.get('moving_models', []))
    obstacles=[]
    for name,part,v,_,_ in shapes(world,'collision'):
        if name=='ground' or name in moving or v[:,2].min()>1.25:continue
        obstacles.append((name,MultiPoint(v[:,:2]).convex_hull))
    minimum=min((route.distance(poly),name) for name,poly in obstacles)
    hits=sorted(set(name for name,p in obstacles if route.buffer(radius).intersects(p)))
    stop_hits={s['name']:sorted(set(name for name,p in obstacles if Point(s['x'],s['y']).buffer(radius).intersects(p))) for s in config['shopping_stops']}
    # Start and shopping goals must also be free; these do not prove reachable turns.
    assert not hits, f'Example route intersects: {hits}'
    assert all(not hit for hit in stop_hits.values()),stop_hits
    report=dict(xml_and_python_syntax='pass',local_mesh_and_texture_paths='pass',
                model_count=len(names),static_person_count=sum(n.startswith(('shopper_', 'vendor_', 'crowd_')) for n in names),
                static_motorcycle_count=sum(n.startswith('motorcycle_') for n in names),
                moving_person_count=len(config.get('traffic',{}).get('pedestrian_names',[])),
                moving_motorcycle_count=len(config.get('traffic',{}).get('motorcycle_names',[])),
                clearance_scope='STATIC obstacles only; crossing traffic can block the route',
                enclosing_robot_radius_m=radius,measured_local_robot_radius_m=round(robot_radius,3),
                minimum_route_center_clearance_m=round(minimum[0],3),
                nearest_model=minimum[1],remaining_radial_margin_m=round(minimum[0]-radius,3),
                shopping_stop_collisions=stop_hits,
                runtime_evidence='v5 is offline geometry validation; see docs/market for historical v4 runtime evidence')
    (preview/'validation.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(report,ensure_ascii=False,indent=2))
    font='/System/Library/Fonts/AppleSDGothicNeo.ttc'
    if Path(font).exists():
        font_manager.fontManager.addfont(font)
        plt.rcParams['font.family']=font_manager.FontProperties(fname=font).get_name()
    plt.rcParams['axes.unicode_minus']=False
    fig,ax=plt.subplots(figsize=(16,7.5),facecolor='#f6f2e9')
    ax.set_facecolor('#f6f2e9')
    # Floor first, then collision footprints for a readable engineering plan.
    for name,part,v,_,color in shapes(world,'visual'):
        if name.startswith('market_paving'):
            poly=MultiPoint(v[:,:2]).convex_hull
            ax.add_patch(PatchPolygon(np.array(poly.exterior.coords),facecolor='#ded7c7',edgecolor='none'))
    for name,part,v,_,color in shapes(world,'visual'):
        if name=='ground' or name.startswith(('paving','market_paving','sign_','entrance_header','stop_')):continue
        if v[:,2].min()>2.0:continue
        poly=MultiPoint(v[:,:2]).convex_hull
        if poly.geom_type=='Polygon':
            ax.add_patch(PatchPolygon(np.array(poly.exterior.coords),facecolor=color,edgecolor='#4c4a41',linewidth=.3))
    ax.annotate('입구 횡단보도',(-2.7,0),xytext=(-3.5,6.5),ha='center',fontsize=11,
                arrowprops=dict(arrowstyle='-',color='#465e7e'),color='#465e7e')
    xs,ys=zip(*config['route']);ax.plot(xs,ys,color='#19796f',lw=2.6,ls='--',zorder=8)
    ax.scatter([config['spawn']['x'],33],[0,0],s=85,color=['#19796f','#c16431'],edgecolors='white',zorder=10)
    ax.text(config['spawn']['x'],-1.9,'횡단보도 앞\n출발',ha='center',va='top',fontsize=11,color='#245349')
    ax.text(33,-.75,'출구',ha='center',va='top',fontsize=11,color='#8e492c')
    for s in config['stalls']:
        ax.text(s['x'],s['y'],s['name'],ha='center',va='center',fontsize=12,
                bbox=dict(boxstyle='round,pad=.4',fc='#fbf7ed',ec='none',alpha=.96))
    for i,s in enumerate(config['shopping_stops'][:2]):
        ax.add_patch(Circle((s['x'],s['y']),.55,fill=False,edgecolor='#19796f',lw=1.7,zorder=9))
        ax.annotate(f"쇼핑 정차 {i+1}",(s['x'],s['y']),xytext=(s['x']+.4,1.9),fontsize=9,color='#16594f')
    ax.annotate('배달 오토바이',(config['encounters'][1]['x'],config['encounters'][1]['y']),xytext=(16,-6.5),ha='center',fontsize=11,
                arrowprops=dict(arrowstyle='-',color='#a7562e'),color='#a7562e')
    ax.annotate('쇼핑객',(config['encounters'][0]['x'],config['encounters'][0]['y']),xytext=(17,6.3),ha='center',fontsize=11,
                arrowprops=dict(arrowstyle='-',color='#465e7e'),color='#465e7e')
    ax.annotate('주차 오토바이',(config['encounters'][2]['x'],config['encounters'][2]['y']),xytext=(25.4,7),ha='center',fontsize=11,
                arrowprops=dict(arrowstyle='-',color='#a7562e'),color='#a7562e')
    ax.annotate('좌판 병목 2.10m',(10,config['congestion_zones'][0]['center_y']),xytext=(9,-7.5),ha='center',fontsize=11,
                arrowprops=dict(arrowstyle='-',color='#a7562e'),color='#a7562e')
    ax.annotate('하역 오토바이 + 손수레',(19.6,config['congestion_zones'][2]['center_y']-1.4),xytext=(20,-8),ha='center',fontsize=11,
                arrowprops=dict(arrowstyle='-',color='#a7562e'),color='#a7562e')
    ax.set(xlim=(-8,36),ylim=(-11,11),xlabel='진행 방향 X (m)',ylabel='Y (m)')
    ax.set_aspect('equal');ax.spines[['top','right']].set_visible(False)
    fig.suptitle('SWAN · 굽은 시장 통로 + 보행신호등',x=.08,ha='left',fontsize=24,fontweight='bold',color='#253e36')
    ax.set_title(f"가게 8개  /  사람 {report['static_person_count']}명  /  오토바이 {report['static_motorcycle_count']}대  ·  추가 이동: 보행자 2명 + 오토바이 2대  ·  점선은 정적 예시 경로",loc='left',fontsize=11,pad=20,color='#6b6b5d')
    fig.tight_layout(rect=(0,0,1,.92));fig.savefig(preview/'market_plan.png',dpi=160);plt.close(fig)
    fig=plt.figure(figsize=(16,9),facecolor='#f6f2e9');ax=fig.add_subplot(111,projection='3d',computed_zorder=False)
    ax.set_facecolor('#f6f2e9')
    faces_all=[];colors=[]
    for name,part,v,faces,color in shapes(world,'visual'):
        if name=='ground':continue
        for face in faces:
            faces_all.append(v[face]);colors.append(color)
    ax.add_collection3d(Poly3DCollection(faces_all,facecolors=colors,edgecolors=(.2,.2,.16,.10),linewidths=.1,zsort='average'))
    ax.set(xlim=(-8,35),ylim=(-11,11),zlim=(0,4.3));ax.set_box_aspect((43,17,7))
    ax.view_init(elev=43,azim=-118);ax.set_axis_off()
    fig.suptitle('SWAN MARKET  /  시장 공간 초안',x=.08,ha='left',fontsize=24,color='#253e36')
    fig.text(.08,.89,'SDF 형상 기반 미리보기 · Gazebo 화면 아님 · 간판 텍스처는 실제 맵에 포함',fontsize=12,color='#6b6b5d')
    fig.subplots_adjust(left=0,right=1,bottom=0,top=.9);fig.savefig(preview/'market_isometric.png',dpi=150);plt.close(fig)
    # Face-on projection of the actual SDF housing/icon geometry (not a Gazebo capture).
    import copy
    fig,axes=plt.subplots(1,3,figsize=(9,6),facecolor='#f6f2e9')
    for ax,(state,label) in zip(axes,[('red','빨간불 · 대기'),('green','초록불 · 횡단'),('off','초록 점멸 · 꺼진 순간')]):
        names=['ped_signal_east_housing']+([] if state=='off' else ['ped_signal_east_'+state])
        local=ET.Element('world')
        for name in names:
            item=copy.deepcopy(world.find(f"model[@name='{name}']"));item.find('pose').text='0 0 0 0 0 0';local.append(item)
        for name,part,v,_,color in shapes(local,'visual'):
            poly=MultiPoint(v[:,[0,2]]).convex_hull
            if poly.geom_type=='Polygon':
                ax.add_patch(PatchPolygon(np.array(poly.exterior.coords),facecolor=color,edgecolor='none'))
        ax.set(xlim=(-.30,.30),ylim=(1.52,2.65));ax.set_aspect('equal');ax.set_axis_off();ax.set_title(label,fontsize=12,pad=15)
    fig.suptitle('사람 모양 보행신호등',fontsize=19,color='#253e36')
    fig.text(.5,.03,'실제 SDF 형상의 정면 투영 · Gazebo 실행 화면 아님 · 초록 점멸 0.5초 켜짐 / 0.5초 꺼짐',ha='center',fontsize=10)
    fig.savefig(preview/'pedestrian_signals.png',dpi=160);plt.close(fig)



if __name__=='__main__':main()
