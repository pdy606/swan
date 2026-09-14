#!/usr/bin/env python3
"""Opt-in SIMULATION test, using Gazebo ground truth; not a navigation controller.

Run against this world's ROS bridge after placing the robot at the configured spawn pose.
Requires ROS Jazzy, gz.transport13/gz.msgs10, Pillow and exported static footprints.
The test never teleports the robot. Commands stop on stale data, low geometric
clearance, a close obstacle in the front LiDAR strip, tilt, stall or timeout.
"""
import argparse
import csv
import json
import math
import signal
import subprocess
import time
from pathlib import Path
from datetime import datetime, timezone

import rclpy
from rclpy.node import Node as ROSNode
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import Odometry
from gz.transport13 import Node as GZNode
from gz.msgs10.pose_v_pb2 import Pose_V
from PIL import Image as PILImage


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def point_segment(x, y, a, b):
    dx, dy = b[0]-a[0], b[1]-a[1]
    t = max(0, min(1, ((x-a[0])*dx+(y-a[1])*dy)/(dx*dx+dy*dy or 1)))
    return math.hypot(x-a[0]-t*dx, y-a[1]-t*dy)


def polygon_distance(x, y, vertices):
    inside = False
    distance = math.inf
    for a, b in zip(vertices, vertices[1:]):
        distance = min(distance, point_segment(x,y,a,b))
        if (a[1]>y) != (b[1]>y) and x < (b[0]-a[0])*(y-a[1])/(b[1]-a[1])+a[0]:
            inside = not inside
    return 0 if inside else distance


class Test:
    def __init__(self, args):
        self.out=Path(args.output);self.out.mkdir(parents=True,exist_ok=True)
        self.layout=json.loads(Path(args.layout).read_text())
        self.obstacles=json.loads(Path(args.geometry).read_text())
        (self.out/'layout.json').write_text(json.dumps(self.layout))
        (self.out/'obstacles.json').write_text(json.dumps(self.obstacles))
        self.max_waypoints=args.max_waypoints
        self.ros=ROSNode('market_driving_validation')
        self.gz=GZNode();self.truth=None;self.data={};self.received={}
        self.counts={};self.stage='initializing';self.last_sample=-math.inf
        self.previous=None;self.travelled=0.;self.peak_odom_speed=0.
        self.min_margin=math.inf;self.nearest=None;self.max_tilt=0.;self.last_status=0.
        self.events=[];self.started=time.monotonic();self.max_wall=600
        self.pub=self.ros.create_publisher(Twist,'/model/wheelchair/cmd_vel',10)
        for key,topic,typ in [('scan','/scan',LaserScan),('odom','/odom',Odometry),('camera','/camera',Image)]:
            self.ros.create_subscription(typ,topic,lambda m,k=key:self.receive(k,m),qos_profile_sensor_data)
        self.gz.subscribe(Pose_V,'/world/swan_market/dynamic_pose/info',self.pose)
        self.file=(self.out/'trajectory.csv').open('w',newline='')
        self.csv=csv.writer(self.file)
        self.csv.writerow(['sim_s','stage','x','y','z','yaw','odom_x','odom_y','odom_v','odom_w','margin_m','nearest_model','front_strip_range_m'])

    def receive(self,key,msg):
        self.data[key]=msg;self.received[key]=time.monotonic()
        self.counts[key]=self.counts.get(key,0)+1

    def pose(self,msg):
        for p in msg.pose:
            if p.name=='wheelchair':
                q=p.orientation
                roll=math.atan2(2*(q.w*q.x+q.y*q.z),1-2*(q.x*q.x+q.y*q.y))
                pitch=math.asin(max(-1,min(1,2*(q.w*q.y-q.z*q.x))))
                yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                self.truth=(msg.header.stamp.sec+msg.header.stamp.nsec*1e-9,p.position.x,p.position.y,p.position.z,yaw,roll,pitch,time.monotonic())
                break

    def pump(self, duration=.05):
        end=time.monotonic()+duration
        while time.monotonic()<end:rclpy.spin_once(self.ros,timeout_sec=.005)

    def command(self,v=0.,w=0.):
        m=Twist();m.linear.x=v;m.angular.z=w;self.pub.publish(m)

    def margin(self,x,y):
        best=math.inf;name=None
        for o in self.obstacles:
            x0,y0,x1,y1=o['bounds']
            if max(x0-x,0,x-x1)**2+max(y0-y,0,y-y1)**2>min(best,3)**2:continue
            d=polygon_distance(x,y,o['vertices'])
            if d<best:best=d;name=o['name']
        return best-.75,name

    def front(self):
        s=self.data['scan'];values=[]
        for i,r in enumerate(s.ranges):
            if not math.isfinite(r):continue
            angle=s.angle_min+i*s.angle_increment
            px,py=r*math.cos(angle),r*math.sin(angle)
            # Rear returns include the wheelchair itself. Only the forward strip
            # in front of the protruding LiDAR is used for this stop check.
            if px>0 and abs(py)<.48:values.append(px)
        return min(values,default=math.inf)

    def check(self,forward=False):
        now=time.monotonic();p=self.truth
        if p is None or now-p[7]>3 or any(now-self.received.get(k,0)>3 for k in ('scan','odom','camera')):
            raise RuntimeError('stale ground truth / sensor stream')
        if now-self.started>self.max_wall:raise RuntimeError('wall-clock timeout')
        margin,nearest=self.margin(p[1],p[2]);front=self.front()
        self.max_tilt=max(self.max_tilt,abs(p[5]),abs(p[6]))
        if margin<.04:raise RuntimeError(f'geometric margin too low: {margin:.3f}m at {nearest}')
        if max(abs(p[5]),abs(p[6]))>.20:raise RuntimeError('excessive body tilt')
        if forward and front<.32:raise RuntimeError(f'front LiDAR strip obstacle: {front:.3f}m')
        if p[0]-self.last_sample>=.2:
            od=self.data['odom'];op=od.pose.pose.position
            self.min_margin=min(self.min_margin,margin)
            if margin<=self.min_margin:self.nearest=nearest
            self.peak_odom_speed=max(self.peak_odom_speed,abs(od.twist.twist.linear.x))
            if self.previous:self.travelled+=math.hypot(p[1]-self.previous[1],p[2]-self.previous[2])
            self.previous=p;self.last_sample=p[0]
            self.csv.writerow([round(p[0],3),self.stage,*p[1:5],op.x,op.y,od.twist.twist.linear.x,od.twist.twist.angular.z,margin,nearest,front])
            self.file.flush()
        if now-self.last_status>3:
            status={'stage':self.stage,'world_xy':[p[1],p[2]],'sim_s':p[0],'distance_m':self.travelled,'min_margin_m':self.min_margin,'wall_s':now-self.started}
            temp=self.out/'status.tmp';temp.write_text(json.dumps(status));temp.replace(self.out/'status.json')
            self.last_status=now
        return p

    def capture(self,name):
        m=self.data.get('camera')
        if m and m.encoding=='rgb8':PILImage.frombytes('RGB',(m.width,m.height),bytes(m.data),'raw','RGB',m.step).save(self.out/f'{name}.png')

    def stop(self,name):
        self.stage=name;begin=self.truth[0]
        while self.truth[0]-begin<1.5:
            self.command();self.pump();self.check()
        v=self.data['odom'].twist.twist
        self.events.append({'test':name,'sim_s':self.truth[0],'position':list(self.truth[1:3]),'linear_speed':v.linear.x,'angular_speed':v.angular.z})
        if abs(v.linear.x)>.02 or abs(v.angular.z)>.03:raise RuntimeError('did not stop')
        self.capture(name)

    def waypoint(self,index,target):
        self.stage=f'route_{index}_to_{target[0]}';best=math.inf;progress=self.truth[0]
        while True:
            self.pump();p=self.check(forward=True)
            dx,dy=target[0]-p[1],target[1]-p[2];distance=math.hypot(dx,dy)
            if distance<.10:break
            if distance<best-.03:best=distance;progress=p[0]
            if p[0]-progress>10:raise RuntimeError('stalled / not approaching waypoint')
            heading=wrap(math.atan2(dy,dx)-p[4])
            v=min(.28,max(.07,.7*distance))*max(0,math.cos(heading))
            if abs(heading)>.45:v=0.
            self.command(v,max(-.4,min(.4,1.8*heading)))
        self.events.append({'test':'waypoint','index':index,'target':target,'actual':list(p[1:3]),'sim_s':p[0]})

    def rotate(self,target,name):
        self.stage=name;begin=self.truth[0]
        while True:
            self.pump();p=self.check();err=wrap(target-p[4])
            if abs(err)<.025:break
            if p[0]-begin>20:raise RuntimeError('turn timeout')
            self.command(0.,max(-.35,min(.35,1.5*err)))
        self.stop(name+'_stop')
        self.events[-1]['target_yaw']=target;self.events[-1]['actual_yaw']=self.truth[4]

    def run(self):
        if self.layout.get('dynamic_actors'):
            raise RuntimeError('This static-footprint driving test does not support moving traffic')
        while time.monotonic()-self.started<12:
            self.command();self.pump()
            if self.truth and all(k in self.data for k in ('scan','odom','camera')):break
        self.check()
        spawn=self.layout['spawn']
        if math.hypot(self.truth[1]-spawn['x'],self.truth[2]-spawn['y'])>.15:
            raise RuntimeError('robot must start at configured spawn pose')
        self.start_pose=self.truth;self.capture('start')
        self.stop('initial_stop')
        for i,target in enumerate(self.layout['route'][1:],1):
            self.waypoint(i,target)
            if target[0] in (-5.6,.8,8,12,20,33):self.stop(f'stop_x{target[0]}')
            if target[0] in (10,16,18,25.5):self.capture(f'pass_x{target[0]}')
            if self.max_waypoints and i>=self.max_waypoints:
                self.stop('route_section_end');return
        self.rotate(math.pi/2,'turn_left_90')
        self.rotate(0,'return_forward')
        self.stage='reverse_0.4m';start=self.truth;begin=start[0]
        while math.hypot(self.truth[1]-start[1],self.truth[2]-start[2])<.4:
            self.pump();self.check()
            if self.truth[0]-begin>10:raise RuntimeError('reverse timeout')
            self.command(-.15,0.)
        self.stop('reverse_stop')
        self.events[-1]['reverse_distance_m']=math.hypot(self.truth[1]-start[1],self.truth[2]-start[2])

    def finish(self,error):
        # Stop even on failures and keep the GUI open, with simulation paused.
        end=time.monotonic()+4
        while time.monotonic()<end:
            self.command();self.pump()
        pause=subprocess.run(['gz','service','-s','/world/swan_market/control','--reqtype','gz.msgs.WorldControl','--reptype','gz.msgs.Boolean','--timeout','5000','--req','pause: true'],capture_output=True,text=True,timeout=8)
        report={'tested_at_utc':datetime.now(timezone.utc).isoformat(),'result':'pass' if error is None else 'failed','error':error,'controller':'Gazebo ground-truth waypoint feedback via ROS cmd_vel; NOT Nav2','teleports_during_test':0,'distance_m':self.travelled,'minimum_0_75m_circle_margin_m':self.min_margin,'nearest_model':self.nearest,'peak_odom_linear_speed_m_s':self.peak_odom_speed,'max_abs_roll_pitch_rad':self.max_tilt,'wall_duration_s':time.monotonic()-self.started,'sensor_message_counts':self.counts,'events':self.events,'simulation_paused':'data: true' in pause.stdout,'contact_sensor_used':False,'limits':'Clearance from sampled actual pose and static SDF footprints; not contact sensor evidence or dynamic obstacle avoidance.'}
        if self.truth:report['final_pose']=list(self.truth[:7])
        if 'odom' in self.data:
            v=self.data['odom'].twist.twist
            report['final_linear_speed']=v.linear.x;report['final_angular_speed']=v.angular.z
        (self.out/'result.json').write_text(json.dumps(report,indent=2)+'\n')
        self.file.close();self.ros.destroy_node()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute',action='store_true',required=True)
    parser.add_argument('--layout',required=True);parser.add_argument('--geometry',required=True);parser.add_argument('--output',required=True)
    parser.add_argument('--max-waypoints',type=int,help='Test only the first N route targets; skip exit rotation/reverse')
    args=parser.parse_args()
    if args.max_waypoints is not None and args.max_waypoints<1:parser.error('--max-waypoints must be positive')
    def interrupt(*_):raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM,interrupt)
    rclpy.init();test=Test(args);error=None
    try:test.run()
    except (Exception,KeyboardInterrupt) as exc:error=str(exc) or 'interrupted';print(error,flush=True)
    finally:test.finish(error);rclpy.shutdown()
    if error:raise SystemExit(1)


if __name__=='__main__':main()
