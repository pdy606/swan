"""Physical contact fixtures in a separate world; source wheelchair is unchanged."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import xml.etree.ElementTree as E
from wheelchair_ai_driver.derived_world import generate
from wheelchair_ai_driver.evaluation_world import instrument

ROOT=Path(__file__).resolve().parents[1]
parser=argparse.ArgumentParser()
parser.add_argument('--output',default='results/provenance/contact_validation_'+str(time.time_ns()))
OUT=(ROOT/parser.parse_args().output).resolve()
if not OUT.is_relative_to(ROOT) or OUT.exists():raise ValueError('Choose a new directory inside AI package')
OUT.mkdir(parents=True)
base=generate(ROOT)
src=E.parse(base).getroot()
world=E.Element('world',name='contact_validation')
E.SubElement(world,'physics',name='default',type='ignored')
E.SubElement(world.find('physics'),'max_step_size').text='0.001'
E.SubElement(world.find('physics'),'real_time_factor').text='1'
for name,lib in [('Physics','physics'),('UserCommands','user-commands'),('SceneBroadcaster','scene-broadcaster')]:
 E.SubElement(world,'plugin',filename=f'gz-sim-{lib}-system',name=f'gz::sim::systems::{name}')
inc=E.SubElement(world,'include'); E.SubElement(inc,'uri').text=src.find("./world/include[name='wheelchair']/uri").text
E.SubElement(inc,'name').text='wheelchair';E.SubElement(inc,'pose').text='0 0 .05 0 0 0'
world.append(E.fromstring('''<model name="ground_plane"><static>true</static><link name="link"><collision name="collision"><geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry></collision></link></model>'''))
world.append(E.fromstring('''<model name="fixture"><static>true</static><pose>0 10 1 0 0 0</pose><link name="link"><collision name="sphere"><geometry><sphere><radius>.025</radius></sphere></geometry></collision></link></model>'''))
world.append(E.fromstring('''<model name="integration_city_branch"><static>true</static><pose>1.5 0 -.05 0 0 0</pose><link name="body"><collision name="integration_city_branch_collision"><geometry><box><size>1.5 3 .1</size></box></geometry></collision></link></model>'''))
world.append(E.fromstring('''<model name="road"><static>true</static><pose>0 10 .2 0 0 0</pose><link name="link"><collision name="collision"><geometry><box><size>.2 .2 .4</size></box></geometry></collision></link></model>'''))
sdf=E.Element('sdf',version='1.9');sdf.append(world)
p=OUT/'base.sdf';E.ElementTree(sdf).write(p)
p=instrument(p,ROOT,OUT)
tree=E.parse(p);plugin=tree.find("./world/plugin[@name='swan_eval::Contacts']");E.SubElement(plugin,'validation').text='true';tree.write(p)
env=dict(os.environ,GZ_PARTITION='swan_contact_validation',GZ_SIM_RESOURCE_PATH=str(ROOT.parent/'wheelchair_gazebo/models'))
log=(OUT/'gazebo.log').open('w')
proc=subprocess.Popen(['gz','sim','-s','-r',str(p)],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
def state():return json.loads((OUT/'contacts_state.json').read_text())
def wait_sim(delta):
 start=state()['sim_time'];deadline=time.monotonic()+max(20,delta*3)
 while state()['sim_time']<start+delta:
  if time.monotonic()>deadline:raise RuntimeError('Simulator not advancing')
  time.sleep(.02)
def move(x,y,z,model="fixture"):
 cmd=['gz','service','-s','/world/contact_validation/set_pose','--reqtype','gz.msgs.Pose','--reptype','gz.msgs.Boolean','--timeout','3000','--req',f'name: "{model}", position: {{x:{x},y:{y},z:{z}}}']
 r=subprocess.run(cmd,env=env,capture_output=True,text=True,timeout=5)
 if r.returncode or 'true' not in r.stdout:raise RuntimeError(r.stdout+r.stderr)
try:
 deadline=time.monotonic()+40
 while not (OUT/'contacts_state.json').exists():
  if proc.poll() is not None or time.monotonic()>deadline:raise RuntimeError('No contact state; inspect gazebo.log')
  time.sleep(.1)
 wait_sim(1)
 (OUT/'armed').touch();wait_sim(.3)
 baseline=state();expected=sorted(json.loads((OUT/'evaluation_manifest.json').read_text())['chair_collisions'])
 assert sorted(baseline['registered_collisions'])==expected,(expected,baseline)
 assert baseline['collision_count']==0,baseline
 results=[{'case':'normal_ground','count':0,'support_pairs':baseline['excluded_support_pairs']}]
 def wheel_command(value):
  subprocess.run(['gz','topic','-t','/model/wheelchair/front_drive/cmd_vel','-m','gz.msgs.Double','-p',f'data: {value}'],env=env,check=True,capture_output=True,timeout=5)
 start_x=baseline['collision_poses']['wheelchair::base_link::seat_collision'][0]
 wheel_command(.6);wait_sim(20);wheel_command(0);wait_sim(.5)
 driven=state()['collision_poses']['wheelchair::base_link::seat_collision'][0]-start_x
 assert driven>1.7 and state()['collision_count']==0,('normal_floor_drive',driven,state())
 results.append({'case':'normal_floor_drive','distance_m':driven,'count':0})
 for name in baseline['registered_collisions']:
  move(0,10,1);wait_sim(.2)
  before=state()['collision_count']
  pose=state()['collision_poses'][name]
  # A small physical sphere inside each existing collision ensures that shape is reported.
  move(*pose[:3]);wait_sim(.15)
  during=state();move(0,10,1);wait_sim(.2)
  events=[json.loads(line) for line in (OUT/'contacts.jsonl').read_text().splitlines()]
  matching=[e for e in events if e['type']=='contact_begin' and e.get('armed') and e['count']>before and name in e['chair_collisions']]
  results.append({'case':name,'matched':bool(matching),'count_delta':state()['collision_count']-before})
  if not matching or state()['collision_count']-before!=1:raise RuntimeError('Missing or duplicated collision '+name)
 # A single fixture touches seat + backrest simultaneously, still one event.
 before=state()['collision_count']
 seat=state()['collision_poses']['wheelchair::base_link::seat_collision']
 move(seat[0]-.205,seat[1],seat[2]+.04);wait_sim(.15)
 matching=[json.loads(line) for line in (OUT/'contacts.jsonl').read_text().splitlines()]
 matching=[e for e in matching if e.get('type')=='contact_begin' and e.get('count',0)>before]
 assert state()['collision_count']-before==1 and any(len(set(e['chair_collisions']))>=2 for e in matching),matching
 results.append({'case':'multiple_parts_same_object','new_events':1,'collisions':matching[0]['chair_collisions']})
 move(0,10,1);wait_sim(.2)
 # Recontact after release creates a distinct event.
 name='wheelchair::base_link::seat_collision'
 before=state()['collision_count'];move(*state()['collision_poses'][name][:3]);wait_sim(.15);move(0,10,1);wait_sim(.2)
 results.append({'case':'release_and_recontact','new_events':state()['collision_count']-before})
 assert results[-1]['new_events']==1
 # A genuine vertical step, even under a whitelisted support model, must count.
 before=state()['collision_count']
 front=state()['collision_poses']['wheelchair::front_drive_wheel::front_drive_wheel_collision']
 move(front[0]+.25,front[1],.2,model='road');wait_sim(.15)
 assert state()['collision_count']>before,('vertical_step_not_detected',state())
 results.append({'case':'real_vertical_support_face','detected':True,'events':state()['collision_count']-before})
 move(0,10,.2,model='road');wait_sim(.2)
 (OUT/'results.json').write_text(json.dumps(results,indent=2)+'\n')
 print(json.dumps(results,indent=2))
finally:
 os.killpg(proc.pid,signal.SIGINT)
 try:proc.wait(timeout=5)
 except subprocess.TimeoutExpired:pass
 try:os.killpg(proc.pid,signal.SIGKILL)
 except ProcessLookupError:pass
 log.close()
