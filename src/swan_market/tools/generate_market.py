#!/usr/bin/env python3
"""Generate a deterministic, static market world and locally bundled signs.

Authoring dependency: Pillow. Runtime: Gazebo Harmonic, no remote assets.
Run with --font /path/to/KoreanFont.ttf when regenerating on Linux.
"""
import argparse
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
ASSET = ROOT / 'models' / 'swan_market_assets'
PALETTE = [(0.72, .24, .16), (.23, .46, .29), (.20, .40, .57), (.72, .49, .15)]


def element(parent, tag, value=None, **attrs):
    node = ET.SubElement(parent, tag, attrs)
    if value is not None:
        node.text = str(value)
    return node


def vector(values):
    return ' '.join(f'{v:.5g}' for v in values)


def model(world, name, pose):
    m = element(world, 'model', name=name)
    element(m, 'static', 'true')
    element(m, 'pose', vector(pose))
    return element(m, 'link', name='body')


def shape(link, name, kind, size, pose, color, collision=True):
    for tag in (['collision', 'visual'] if collision else ['visual']):
        obj = element(link, tag, name=f'{name}_{tag}')
        element(obj, 'pose', vector(pose))
        geom = element(element(obj, 'geometry'), kind)
        if kind == 'box':
            element(geom, 'size', vector(size))
        elif kind == 'sphere':
            element(geom, 'radius', size[0])
        elif kind == 'cylinder':
            element(geom, 'radius', size[0])
            element(geom, 'length', size[1])
        if tag == 'visual':
            mat = element(obj, 'material')
            element(mat, 'ambient', vector((*color, 1)))
            element(mat, 'diffuse', vector((*color, 1)))


def box(world, name, xyz, size, color, collision=True, yaw=0):
    link = model(world, name, (*xyz, 0, 0, yaw))
    shape(link, 'box', 'box', size, (0, 0, 0, 0, 0, 0), color, collision)
    return link


def sign(world, key, text, xyz, width, yaw, color, font):
    """Textured vertical panel, local face normal -Y, bundled COLLADA texture."""
    tex = ASSET / 'materials' / 'textures' / f'{key}.png'
    tex.parent.mkdir(parents=True, exist_ok=True)
    im = Image.new('RGB', (1024, 192), tuple(int(c * 255) for c in color))
    draw = ImageDraw.Draw(im)
    ft = ImageFont.truetype(font, 88)
    draw.rounded_rectangle((12, 12, 1012, 180), radius=8, outline='#f6e9ce', width=3)
    draw.text((512, 92), text, font=ft, fill='#fff5df', anchor='mm')
    im.save(tex)
    mesh = ASSET / 'meshes' / f'{key}.dae'
    mesh.parent.mkdir(parents=True, exist_ok=True)
    mesh.write_text(f'''<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
 <asset><created>2026-09-12T00:00:00Z</created><modified>2026-09-12T00:00:00Z</modified><unit meter="1"/><up_axis>Z_UP</up_axis></asset>
 <library_images><image id="image"><init_from>../materials/textures/{key}.png</init_from></image></library_images>
 <library_effects><effect id="effect"><profile_COMMON>
  <newparam sid="surface"><surface type="2D"><init_from>image</init_from></surface></newparam>
  <newparam sid="sampler"><sampler2D><source>surface</source></sampler2D></newparam>
  <technique sid="common"><lambert><emission><color>0 0 0 1</color></emission><ambient><color>1 1 1 1</color></ambient><diffuse><texture texture="sampler" texcoord="UVSET0"/></diffuse></lambert></technique>
 </profile_COMMON></effect></library_effects>
 <library_materials><material id="material"><instance_effect url="#effect"/></material></library_materials>
 <library_geometries><geometry id="panel"><mesh>
  <source id="positions"><float_array id="posarray" count="12">-.5 0 -.5 .5 0 -.5 .5 0 .5 -.5 0 .5</float_array><technique_common><accessor source="#posarray" count="4" stride="3"><param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/></accessor></technique_common></source>
  <source id="uv"><float_array id="uvarray" count="8">0 0 1 0 1 1 0 1</float_array><technique_common><accessor source="#uvarray" count="4" stride="2"><param name="S" type="float"/><param name="T" type="float"/></accessor></technique_common></source>
  <source id="normals"><float_array id="normalarray" count="3">0 -1 0</float_array><technique_common><accessor source="#normalarray" count="1" stride="3"><param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/></accessor></technique_common></source>
  <vertices id="vertices"><input semantic="POSITION" source="#positions"/></vertices>
  <triangles count="2" material="mat"><input semantic="VERTEX" source="#vertices" offset="0"/><input semantic="TEXCOORD" source="#uv" offset="1" set="0"/><input semantic="NORMAL" source="#normals" offset="2"/><p>0 0 0 1 1 0 2 2 0 0 0 0 2 2 0 3 3 0</p></triangles>
 </mesh></geometry></library_geometries>
 <library_visual_scenes><visual_scene id="scene"><node id="panelnode"><instance_geometry url="#panel"><bind_material><technique_common><instance_material symbol="mat" target="#material"><bind_vertex_input semantic="UVSET0" input_semantic="TEXCOORD" input_set="0"/></instance_material></technique_common></bind_material></instance_geometry></node></visual_scene></library_visual_scenes>
 <scene><instance_visual_scene url="#scene"/></scene>
</COLLADA>''')
    link = model(world, f'sign_{key}', (*xyz, 0, 0, yaw))
    visual = element(link, 'visual', name='lettering')
    geo = element(element(visual, 'geometry'), 'mesh')
    element(geo, 'uri', f'model://swan_market_assets/meshes/{key}.dae')
    element(geo, 'scale', f'{width} 1 {width * 192 / 1024}')
    material = element(visual, 'material')
    element(material, 'ambient', '1 1 1 1')
    element(material, 'diffuse', '1 1 1 1')
    element(material, 'emissive', '0 0 0 1')
    metal = element(element(material, 'pbr'), 'metal')
    element(metal, 'albedo_map', f'model://swan_market_assets/materials/textures/{key}.png')
    element(metal, 'metalness', '0')
    element(metal, 'roughness', '1')


def stall(world, idx, x, side, title, font):
    color = PALETTE[idx % 4]
    y = side * 4.45
    body = model(world, f'stall_{idx:02d}', (x, y, 0, 0, 0, 0))
    # Open storefront: no wall across the front.
    shape(body, 'rear', 'box', (4.8, .15, 2.8), (0, side*1.65, 1.4, 0, 0, 0), (.78,.74,.65))
    for dx in (-2.35, 2.35):
        shape(body, f'side_{dx}', 'box', (.12, 3.3, 2.8), (dx, 0, 1.4, 0, 0, 0), (.81,.77,.69))
    shape(body, 'counter', 'box', (3.5, .85, .8), (0, -side*1.22, .4, 0, 0, 0), (.47,.32,.21))
    shape(body, 'counter_top', 'box', (3.6,.95,.06), (0,-side*1.22,.83,0,0,0), (.86,.77,.59))
    for j in range(10):
        tint = color if j % 2 else (.92,.86,.68)
        shape(body, f'awning_{j}', 'box', (.48, 3.85, .10), (-2.16+j*.48,-side*.10,2.78,0,0,0), tint)
    # Low front stock crates: explicit collisions at LiDAR height.
    for j in (-1, 0, 1):
        shape(body, f'crate_{j}', 'box', (.78,.60,.30), (j*1.04,-side*1.77,.15,0,0,0), (.40,.28,.15))
        for k in range(6):
            produce = [( .84,.18,.11),(.31,.58,.16),(.84,.59,.10),(.74,.58,.39)][idx % 4]
            shape(body, f'produce_{j}_{k}', 'sphere', (.105,),
                  (j*1.04+(k%3-1)*.21,-side*1.77+(k//3-.5)*.22,.39,0,0,0), produce, False)
    # Sign slightly in front of an opaque backing panel.
    box(world, f'signboard_{idx}', (x,side*2.69,2.24), (3.65,.08,.72), color)
    sign(world, f'shop_{idx}', title, (x,side*2.642,2.24),3.6,0 if side>0 else math.pi,color,font)


def person(world, name, x, y, yaw, color, bag=False):
    link = model(world, name, (x,y,0,0,0,yaw))
    skin=(.77,.56,.40); dark=(.12,.15,.18)
    for side in (-1,1):
        shape(link, f'leg_{side}', 'cylinder', (.085,.68), (0,side*.12,.40,0,0,0), dark)
        shape(link, f'shoe_{side}', 'box', (.28,.17,.11), (.06,side*.12,.055,0,0,0), (.08,.07,.06))
        shape(link, f'arm_{side}', 'cylinder', (.068,.48), (.02,side*.27,1.02,0,side*.12,0), color)
    shape(link,'torso','box',(.30,.43,.58),(0,0,1.02,0,0,0),color)
    shape(link,'neck','cylinder',(.07,.13),(0,0,1.37,0,0,0),skin,False)
    shape(link,'head','sphere',(.16,),(0,0,1.57,0,0,0),skin)
    shape(link,'hair','sphere',(.155,),(-.025,0,1.63,0,0,0),(.10,.08,.06),False)
    if bag:
        shape(link,'shopping_bag','box',(.24,.17,.33),(.04,-.38,.60,0,0,0),(.78,.65,.39))


def motorcycle(world, name, x, y, yaw, color, rider=False):
    link=model(world,name,(x,y,0,0,0,yaw))
    for i,dx in enumerate((-.61,.62)):
        shape(link,f'tire_{i}','cylinder',(.25,.14),(dx,0,.25,math.pi/2,0,0),(.075,.075,.07))
        shape(link,f'hub_{i}','cylinder',(.13,.145),(dx,0,.25,math.pi/2,0,0),(.57,.59,.60),False)
    shape(link,'deck','box',(1.20,.43,.17),(0,0,.33,0,0,0),(.18,.18,.18))
    shape(link,'rear_body','box',(.65,.46,.40),(-.36,0,.59,0,0,0),color)
    shape(link,'seat','box',(.80,.40,.12),(-.2,0,.83,0,0,0),(.10,.10,.10))
    shape(link,'front_fairing','box',(.23,.43,.69),(.53,0,.69,0,-.20,0),color)
    shape(link,'handlebar','cylinder',(.035,.72),(.56,0,1.12,math.pi/2,0,0),(.18,.18,.18))
    shape(link,'headlight','sphere',(.095,),(.70,0,.94,0,0,0),(.96,.92,.66),False)
    shape(link,'delivery_box','box',(.51,.55,.48),(-.65,0,1.01,0,0,0),(.20,.23,.25))
    if rider:
        shape(link,'rider_torso','box',(.33,.39,.43),(-.12,0,1.12,0,-.15,0),(.22,.36,.52))
        shape(link,'helmet','sphere',(.17,),(-.07,0,1.51,0,0,0),(.91,.79,.20))
        shape(link,'visor','box',(.045,.23,.10),(.087,0,1.51,0,0,0),(.13,.17,.20),False)
        for side in (-1,1):
            shape(link,f'rider_leg_{side}','box',(.44,.14,.18),(-.12,side*.28,.75,0,.40,0),(.16,.18,.23))
            shape(link,f'rider_arm_{side}','box',(.48,.11,.12),(.20,side*.23,1.12,0,.10,0),(.22,.36,.52))
    else:
        shape(link,'stand','box',(.12,.50,.20),(-.2,0,.10,0,0,0),(.12,.12,.12))


def handcart(world, name, x, y, yaw=0):
    link=model(world,name,(x,y,0,0,0,yaw))
    shape(link,'bed','box',(1.15,.8,.10),(0,0,.20,0,0,0),(.25,.33,.31))
    for dx in (-.4,.4):
        for dy in (-.34,.34):
            shape(link,f'wheel_{dx}_{dy}','cylinder',(.13,.10),(dx,dy,.13,math.pi/2,0,0),(.09,.09,.08))
    for z in (.41,.73):
        shape(link,f'cargo_{z}','box',(.95,.68,.30),(0,0,z,0,0,0),(.57,.40,.24))
    for dy in (-.34,.34):
        shape(link,f'handle_{dy}','cylinder',(.025,.75),(-.55,dy,.58,0,0,0),(.30,.34,.32))
    shape(link,'grip','cylinder',(.03,.70),(-.55,0,.95,math.pi/2,0,0),(.15,.16,.15))


def crowded_details(world):
    # Two low display tables create a 2.10 m pinch point on the curved aisle.
    for side in (-1,1):
        link=box(world,f'aisle_display_{side}',(10,side*1.65,.3),(1.6,1.2,.6),(.44,.30,.19))
        for dx in (-.5,0,.5):
            shape(link,f'basket_{dx}','box',(.42,.65,.12),(dx,0,.36,0,0,0),(.66,.51,.29))
            for j in (-1,0,1):
                shape(link,f'goods_{dx}_{j}','sphere',(.10,),(dx,j*.18,.47,0,0,0),(.76,.32,.10),False)
    handcart(world,'handcart_unloading',19.6,1.45)
    handcart(world,'handcart_waiting',30.8,-1.75,.12)
    # Cardboard boxes and tall bundled goods fill the storefront gaps.
    for i,(x,y) in enumerate(((8.7,-2.1),(14.8,2.2),(23.5,-2.15),(26.0,-2.15))):
        for j in range(2):
            box(world,f'stock_stack_{i}_{j}',(x,y,.2+j*.4),(.65,.6,.4),(.58,.43,.28))
    # Extra shoppers gather in queues and at the crossing; collision bodies remain explicit.
    crowd=[(3.8,1.7,1.4),(4.0,-1.65,-1.2),(7.2,-1.7,-1.5),
           (11.5,1.65,1.5),(12.3,1.65,1.4),(15.3,-1.75,.6),
           (17.8,1.45,2.8),(18.0,-1.45,.2),(23.1,1.65,1.5),
           (24.0,-1.65,-1.5),(28.9,1.7,1.5),(31.4,1.65,.2)]
    for i,(x,y,yaw) in enumerate(crowd):
        person(world,f'crowd_{i:02d}',x,y,yaw,PALETTE[i%4],i%3==0)
    motorcycle(world,'motorcycle_unloading',19.6,-1.40,0,(.64,.43,.10))


def entrance_crosswalk(world, font):
    # The physical ground stays flat; asphalt, paint and tactile areas are visuals.
    # Curbs end outside the 3.2 m wide step-free opening.
    box(world,'entrance_road',(-2.7,0,.010),(4.2,16,.004),(.20,.22,.23),False)
    for name,x,width in [('west',-6.4,3.2),('market',.25,1.7)]:
        box(world,f'entrance_sidewalk_{name}',(x,0,.014),(width,16,.004),(.67,.66,.61),False)
    for i in range(7):
        box(world,f'crosswalk_stripe_{i}',(-4.57+i*.60,0,.016),(.38,2.8,.003),(.95,.95,.89),False)
    for side in (-1,1):
        for x in (-4.83,-.57):
            box(world,f'entrance_curb_{x}_{side}',(x,side*4.8,.07),(.14,6.4,.14),(.77,.76,.70))
        # The stop line is confined to its approach lane.
        box(world,f'road_stop_line_{side}',(-2.7-side*1.05,side*2.0,.016),(1.7,.16,.003),(.95,.95,.89),False)
        for j in range(4):
            box(world,f'road_center_line_{side}_{j}',(-2.7,side*(3.2+j*1.25),.016),(.10,.65,.003),(.91,.70,.16),False)
    for i,x in enumerate((-5.2,-.2)):
        box(world,f'crosswalk_tactile_{i}',(x,0,.019),(.50,2.8,.003),(.85,.64,.14),False)
        for j in range(12):
            box(world,f'tactile_mark_{i}_{j}',(x,-1.3+j*.235,.022),(.30,.055,.002),(.69,.47,.10),False)
    pole=model(world,'crosswalk_sign_pole',(0,2.1,0,0,0,0))
    shape(pole,'post','cylinder',(.045,2.2),(0,0,1.1,0,0,0),(.35,.38,.40))
    box(world,'crosswalk_sign_back',(0,2.1,2.15),(.08,1.25,.30),(.10,.29,.60))
    sign(world,'crosswalk','횡단보도',(-.048,2.1,2.15),1.2,-math.pi/2,(.10,.29,.60),font)


def aisle_center(x):
    """Two smooth opposing bends, with a straight crosswalk/entrance."""
    if x <= 3 or x >= 33:
        return 0.0, 0.0
    a = 2 * math.pi * (x - 3) / 30
    y = 2.2 * math.sin(a) ** 3
    slope = 2.2 * 3 * math.sin(a) ** 2 * math.cos(a) * 2 * math.pi / 30
    return y, math.atan(slope)


def bend_point(x, y, yaw=0):
    center, angle = aisle_center(x)
    return x - y * math.sin(angle), center + y * math.cos(angle), yaw + angle


def bend_market(world):
    """Move shop groups and obstacles along the same centerline as the paving."""
    fixed = ('ground', 'boundary_', 'entrance_', 'crosswalk_', 'road_', 'tactile_',
             'crossing_', 'traffic_', 'ped_signal_', 'sign_crosswalk', 'sign_entrance')
    for item in world.findall('model'):
        if item.get('name').startswith(fixed):
            continue
        pose = [float(v) for v in item.findtext('pose').split()]
        if item.get('name').startswith(('stall_', 'signboard_', 'sign_shop')):
            # Preserve spacing between rigid shop buildings on the inside of bends.
            pose[1] += aisle_center(pose[0])[0]
        else:
            pose[0], pose[1], pose[5] = bend_point(pose[0], pose[1], pose[5])
        item.find('pose').text = vector(pose)


def pedestrian_signals(world):
    """Opaque two-lens housings with emissive standing/walking human silhouettes.

    Each lit icon is a separate collision-free model. The animation puts an unlit
    icon below the ground; the opaque dark lens remains in the housing.
    """
    signals = []
    for side, x, y, yaw in [('west', -5.35, 1.95, math.pi/2),
                            ('east', .10, -1.95, -math.pi/2)]:
        base = model(world, f'ped_signal_{side}_housing', (x,y,0,0,0,yaw))
        shape(base,'pole','cylinder',(.045,2.05),(0,0,1.025,0,0,0),(.23,.25,.27))
        shape(base,'housing','box',(.48,.20,.99),(0,0,2.08,0,0,0),(.055,.065,.075))
        for color,z in [('red',2.32),('green',1.84)]:
            shape(base,f'{color}_lens','cylinder',(.205,.022),(0,-.113,z,math.pi/2,0,0),(.013,.018,.022),False)
            shape(base,f'{color}_hood','box',(.47,.25,.04),(0,-.13,z+.225,0,0,0),(.07,.08,.09),False)
            name=f'ped_signal_{side}_{color}'
            icon=model(world,name,(x,y,0 if color=='red' else -5,0,0,yaw))
            tint=(1.0,.025,.015) if color=='red' else (.025,1.0,.18)
            head=(-.012,.137) if color=='red' else (.025,.137)
            shape(icon,'head','cylinder',(.035,.010),(head[0],-.132,z+head[1],math.pi/2,0,0),tint,False)
            # Rectangular strokes in the face's X/Z plane, thick enough for the camera.
            if color=='red':
                strokes=[((0,.095),(0,-.015),.064), ((-.046,.071),(-.066,-.035),.026),
                         ((.046,.071),(.066,-.035),.026), ((-.021,-.01),(-.030,-.145),.032),
                         ((.021,-.01),(.030,-.145),.032)]
            else:
                strokes=[((.012,.092),(-.020,-.020),.054), ((0,.068),(-.064,.018),.026),
                         ((-.064,.018),(-.112,.035),.026), ((.020,.060),(.075,.004),.026),
                         ((.075,.004),(.119,.013),.026), ((-.020,-.018),(-.084,-.142),.031),
                         ((-.016,-.025),(.055,-.074),.031), ((.055,-.074),(.086,-.147),.031)]
            for i,(a,b,width) in enumerate(strokes):
                dx,dz=b[0]-a[0],b[1]-a[1]
                shape(icon,f'stroke_{i}','box',(width,.010,math.hypot(dx,dz)+.012),
                      ((a[0]+b[0])/2,-.132,z+(a[1]+b[1])/2,0,math.atan2(dx,dz),0),tint,False)
            for visual in icon.findall('visual'):
                element(visual.find('material'),'emissive',vector((*tint,1)))
                element(visual,'cast_shadows','false')
        signals.append(dict(red=f'ped_signal_{side}_red',green=f'ped_signal_{side}_green',x=x,y=y,yaw=yaw))
    return signals


def build(font):
    sdf=ET.Element('sdf',version='1.10'); w=element(sdf,'world',name='swan_market')
    physics=element(w,'physics',name='market_physics',type='ignored')
    element(physics,'max_step_size',.001);element(physics,'real_time_factor',1)
    for short,cls in [('physics','Physics'),('user-commands','UserCommands'),('scene-broadcaster','SceneBroadcaster'),('sensors','Sensors')]:
        plugin=element(w,'plugin',filename=f'gz-sim-{short}-system',name=f'gz::sim::systems::{cls}')
        if short=='sensors':element(plugin,'render_engine','ogre2')
    scene=element(w,'scene');element(scene,'ambient','0.65 0.65 0.65 1');element(scene,'background','0.74 0.82 0.86 1');element(scene,'shadows','true')
    light=element(w,'light',name='sun',type='directional');element(light,'pose','0 0 15 0 0 0');element(light,'direction','-.3 -.5 -1');element(light,'diffuse','.85 .82 .75 1');element(light,'specular','.1 .1 .1 1');element(light,'cast_shadows','true')
    box(w,'ground',(15,0,-.10),(46,24,.2),(.64,.61,.55))
    # Short overlapping tiles follow the bends without physical seams or steps.
    for i in range(72):
        x=-1.75+i*.5
        box(w,f'market_paving_{i}',(x,0,.002),(.75,5.2,.004),(.79,.76,.68),False)
        if i%2==0:
            box(w,f'paving_joint_{i}',(x,0,.0045),(.025,5.2,.001),(.67,.64,.57),False)
    entrance_crosswalk(w,font)
    signals=pedestrian_signals(w)
    box(w,'cross_aisle',(17,0,.006),(3.0,15,.002),(.72,.70,.65),False)
    for side in (-1,1):
        box(w,f'boundary_{side}',(16,side*10.5,.5),(38,.15,1),(.65,.59,.49))
        box(w,f'entrance_post_{side}',(1.5,side*3.25,1.8),(.28,.28,3.6),(.20,.29,.25))
    box(w,'entrance_header',(1.5,0,3.6),(.24,6.8,.82),(.20,.34,.28))
    sign(w,'entrance','스완 전통시장',(1.365,0,3.6),6.6,-math.pi/2,(.20,.34,.28),font)
    titles=['싱싱 과일','우리 채소','바다 생선','엄마 반찬','고소한 떡집','시장 분식','생활 잡화','동네 꽃집']
    stalls=[]
    for side in (1,-1):
        for x in (6,12,22,28):
            idx=len(stalls);stall(w,idx,x,side,titles[idx],font);stalls.append(dict(name=titles[idx],x=x,y=side*4.45))
    people=[('shopper_fruit',6,1.65,math.pi/2,(.29,.43,.63),True),
            ('shopper_crossing',16.6,1.0,math.pi,(.74,.38,.22),True),
            ('shopper_grocery',21,-1.65,-math.pi/2,(.36,.48,.31),True),
            ('shopper_flowers',29,-1.5,-math.pi/2,(.64,.37,.46),True),
            ('shopper_exit',32,1.7,0,(.28,.40,.49),True)]
    for args in people:person(w,*args)
    for i,(x,side) in enumerate(((6,1),(12,-1),(22,1),(28,-1))):
        person(w,f'vendor_{i}',x,side*4.45,-side*math.pi/2,(.79,.69,.48))
    motorcycle(w,'motorcycle_delivery',13.7,-1.65,.18,(.65,.17,.12))
    motorcycle(w,'motorcycle_parked',25.4,1.65,math.pi-.15,(.17,.30,.48))
    crowded_details(w)
    person(w,'crossing_pedestrian_1',-5.7,-.75,0,(.73,.29,.17),True)
    person(w,'crossing_pedestrian_2',.5,.75,math.pi,(.23,.47,.64),True)
    motorcycle(w,'traffic_motorcycle_1',-1.65,-5.8,math.pi/2,(.76,.19,.12),rider=True)
    motorcycle(w,'traffic_motorcycle_2',-3.75,5.8,-math.pi/2,(.19,.39,.70),rider=True)
    # Shopping stop and turning area markings are visual only.
    stops=[dict(name='과일 구매',x=8.2,y=.6,yaw=math.pi/2),dict(name='반찬 구매',x=27.6,y=.7,yaw=math.pi/2),dict(name='출구 대기',x=33,y=0,yaw=0)]
    for i,s in enumerate(stops):
        for dx,dy,sx,sy in [(-.7,0,.04,1.2),(.7,0,.04,1.2),(0,-.6,1.4,.04),(0,.6,1.4,.04)]:
            box(w,f'stop_{i}_{dx}_{dy}',(s['x']+dx,s['y']+dy,.009),(sx,sy,.002),(.22,.52,.47),False)
    metadata=dict(world='swan_market',version=5,units='metres',dynamic_actors=True,
                  spawn=dict(x=-6.2,y=0,z=.03,yaw=0),main_aisle_nominal_width=4.76,
                  shopping_stops=stops,stalls=stalls,
                  route=[[-6.2,0],[-5.6,0],[0.8,0],[8,0],[10,0],[12,0],[14,0],[16,-.4],[18,0],[20,0],[23,0],[25.5,-.25],[28,0],[33,0]],
                  crosswalk=dict(road_x_min=-4.8,road_x_max=-.6,crossing_length_m=4.2,walking_width_m=2.8,curb_opening_width_m=3.2,step_free=True,traffic_signals=True,moving_traffic=True),
                  traffic=dict(signals=signals,green_start=17.0,green_flash_start=24.0,green_end=27.0,blink_period=1.0,cycle_seconds=32.0,bike_move_seconds=15.0,pedestrian_start=17.0,pedestrian_duration=10.0,
                               road_center_x=-2.7,lane_radius=1.05,road_straight_half_length=5.8,
                               pedestrian_west_x=-5.7,pedestrian_east_x=.5,
                               pedestrian_names=['crossing_pedestrian_1','crossing_pedestrian_2'],
                               motorcycle_names=['traffic_motorcycle_1','traffic_motorcycle_2'],
                               motion='scripted kinematic poses; not vehicle dynamics or robot avoidance'),
                  moving_models=['crossing_pedestrian_1','crossing_pedestrian_2','traffic_motorcycle_1','traffic_motorcycle_2'],
                  congestion_zones=[dict(name='돌출 좌판 병목',x_min=9.2,x_max=10.8,measured_gap_m=2.1),
                                    dict(name='쇼핑객 교차 구간',x_min=15.5,x_max=18.5),
                                    dict(name='오토바이 하역 구간',x_min=18.5,x_max=20.8)],
                  encounters=[dict(name='쇼핑객',x=16.6,y=1.0),dict(name='배달 오토바이',x=13.7,y=-1.65),dict(name='주차 오토바이',x=25.4,y=1.65)])
    bend_market(w)
    for collection in (metadata['stalls'],metadata['shopping_stops'],metadata['encounters']):
        for item in collection:
            if collection is metadata['stalls']:
                x,y,yaw=item['x'],item['y']+aisle_center(item['x'])[0],0
            else:
                x,y,yaw=bend_point(item['x'],item['y'],item.get('yaw',0))
            item.update(x=x,y=y)
            if 'yaw' in item:item['yaw']=yaw
    old_route=metadata['route']
    route=[]
    for a,b in zip(old_route,old_route[1:]):
        steps=max(1,math.ceil((b[0]-a[0])/.2))
        for i in range(steps):
            t=i/steps;x=a[0]+(b[0]-a[0])*t;y=a[1]+(b[1]-a[1])*t
            route.append(list(bend_point(x,y)[:2]))
    route.append(list(bend_point(*old_route[-1])[:2]))
    metadata['route']=route
    metadata['aisle_centerline']=[list(bend_point(i*.25,0)[:2]) for i in range(137)]
    metadata['aisle_shape']='S-shaped, smooth opposite bends with +/-2.2 m centerline offset'
    for zone in metadata['congestion_zones']:
        zone['center_y']=aisle_center((zone['x_min']+zone['x_max'])/2)[0]
    metadata['pedestrian_signal_sequence']='red 0-17 s; green 17-24 s; flashing green 24-27 s (1 Hz); red 27-32 s'
    world_dir = ROOT.parent/'wheelchair_gazebo/worlds'
    world_dir.mkdir(parents=True,exist_ok=True);(ROOT/'config').mkdir(exist_ok=True)
    ET.indent(sdf,space='  ');ET.ElementTree(sdf).write(world_dir/'market_shopping.world',encoding='utf-8',xml_declaration=True)
    (ROOT/'config/market_layout.json').write_text(json.dumps(metadata,ensure_ascii=False,indent=2)+'\n')
    (world_dir/'market_shopping.launch.json').write_text(json.dumps(
        {'spawn':metadata['spawn'], 'scenario_launch':'launch/traffic.launch.py', 'scenario_package':'swan_market'},indent=2)+'\n')
    (ASSET/'model.config').write_text('<?xml version="1.0"?><model><name>swan_market_assets</name><version>1.0</version><sdf version="1.10">model.sdf</sdf><description>Local Korean market signage assets</description></model>')
    (ASSET/'model.sdf').write_text('<?xml version="1.0"?><sdf version="1.10"><model name="swan_market_assets"><static>true</static><link name="assets"/></model></sdf>')
    print(f'Generated {len(w.findall("model"))} models: {world_dir / "market_shopping.world"}')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--font',default='/System/Library/Fonts/AppleSDGothicNeo.ttc')
    build(parser.parse_args().font)
