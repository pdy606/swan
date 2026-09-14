#!/usr/bin/env python3
"""Sample one outbound and one return traffic cycle against SDF footprints.
Authoring dependencies match inspect_market.py. This excludes the wheelchair.
"""
import copy,json,math,sys
from pathlib import Path
import xml.etree.ElementTree as ET
from shapely.geometry import MultiPoint
from shapely.ops import unary_union
from shapely.affinity import rotate,translate
from inspect_market import shapes
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from animate_crosswalk import traffic_poses

root=Path(__file__).resolve().parents[1]
layout=json.loads((root/'config/market_layout.json').read_text());cfg=layout['traffic'];names=layout['moving_models']
w=ET.parse(root.parent/'wheelchair_gazebo/worlds/market_shopping.world').getroot().find('world')
static=[]
for name,part,v,_,_ in shapes(w,'collision'):
    if name=='ground' or name in names or v[:,2].min()>1.8:continue
    static.append(MultiPoint(v[:,:2]).convex_hull)
fixed=unary_union(static);footprints={}
for name in names:
    m=copy.deepcopy(w.find(f"model[@name='{name}']"));m.find('pose').text='0 0 0 0 0 0'
    local=ET.Element('world');local.append(m)
    footprints[name]=unary_union([MultiPoint(v[:,:2]).convex_hull for _,_,v,_,_ in shapes(local,'collision')])
minimum_static=math.inf;minimum_pair=math.inf;max_jump=0.;previous=None
for i in range(1281):
    t=i*.05;poses,stage,phase=traffic_poses(t,cfg);bodies={}
    for name,(x,y,a) in poses.items():
        body=translate(rotate(footprints[name],a,origin=(0,0),use_radians=True),xoff=x,yoff=y)
        assert not body.intersects(fixed),f'Static overlap at {t}: {name}'
        minimum_static=min(minimum_static,body.distance(fixed));bodies[name]=body
        if previous:max_jump=max(max_jump,math.hypot(x-previous[name][0],y-previous[name][1]))
    for j,name in enumerate(names):
        for other in names[j+1:]:
            assert not bodies[name].intersects(bodies[other]),f'Actor overlap at {t}: {name}/{other}'
            minimum_pair=min(minimum_pair,bodies[name].distance(bodies[other]))
    previous=poses
assert max_jump<.06,f'Teleport between successive trajectory positions: {max_jump}'
report={'scope':'2 cycles at 0.05 simulation second intervals; 2D collision footprints; excludes wheelchair and contact physics',
        'sample_count':1281,'result':'pass','minimum_actor_static_clearance_m':minimum_static,
        'minimum_actor_pair_clearance_m':minimum_pair,'maximum_position_step_m':max_jump}
(root/'preview').mkdir(exist_ok=True)
(root/'preview/traffic_geometry_validation.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
