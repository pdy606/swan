"""Evaluation observer and bounded run lifecycle; never supplies obstacle data to control."""
import csv
import json
import math
import os
from pathlib import Path
import statistics
import time

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger
from .experiment_metrics import CourseProgress
from .avoidance_metrics import AvoidanceInterventions
from .experiment_store import save_json
from .observation_metrics import MotionMetrics


class Recorder(Node):
    def __init__(self):
        super().__init__('swan_experiment_recorder')
        for k,v in {'output_dir':'','max_wall_sec':3000.,'prepare_wall_sec':180.,'assistance':False}.items():self.declare_parameter(k,v)
        self.output=Path(self.get_parameter('output_dir').value).resolve()
        root=Path(__file__).resolve().parents[1]
        if not self.output.is_relative_to(root):raise ValueError('Output outside AI package')
        self.output.mkdir(parents=True,exist_ok=True)
        self.status={};self.last_status_wall=time.monotonic();self.created=time.monotonic()
        self.odom=None;self.started=False;self.done=False;self.ending=None;self.start_future=None
        self.start_client=self.create_client(Trigger,'/ai_driver/start')
        self.stop_client=self.create_client(Trigger,'/ai_driver/emergency_stop')
        self.stop_future=None;self.last_stop=-100.;self.last_checkpoint=-100.;self.last_log=-100.
        self.metrics=MotionMetrics();self.progress=None;self.percent=0.;self.last_state=None;self.accepted_state=False
        self.latencies=[];self.last_odom_log=-100.;self.zero_since=None
        self.interventions=AvoidanceInterventions(self.get_parameter('assistance').value)
        self.events=(self.output/'validation.jsonl').open('x')
        self.trajectory=(self.output/'odom.csv').open('x',newline='')
        self.writer=csv.writer(self.trajectory);self.writer.writerow(['sim_time','x','y','vx','wz','progress_percent','route_index'])
        self.create_subscription(String,'/ai_driver/driver_status',self.on_status,30)
        self.create_subscription(String,'/ai_driver/prediction',self.on_prediction,20)
        self.create_subscription(String,'/avoidance_status',self.on_avoidance,100)
        self.create_subscription(Odometry,'/odom',self.on_odom,qos_profile_sensor_data)
        self.create_timer(.1,self.tick,clock=Clock())

    def event(self,kind,**values):
        self.events.write(json.dumps(dict(kind=kind,wall_time=time.time(),**values),allow_nan=False)+'\n');self.events.flush()

    def contacts(self):
        try:return json.loads((self.output/'contacts_state.json').read_text())
        except (FileNotFoundError,json.JSONDecodeError):return {}

    def on_status(self,m):
        self.status=json.loads(m.data);self.last_status_wall=time.monotonic()
        if self.started:
            if self.status['state'] not in ('DISABLED',):self.accepted_state=True
            if self.status['state']!=self.last_state:
                self.event('driver_state',status=self.status);self.last_state=self.status['state']
            if self.ending is None and self.accepted_state and self.status['state'] in ('ARRIVED','FAILED','ESTOP','DISABLED'):
                self.finish(self.status.get('reason',self.status['state']),self.status['state']=='ARRIVED')

    def on_prediction(self,m):
        if self.started and self.ending is None:
            p=json.loads(m.data);self.latencies.append(p['onnx_ms'])

    def on_avoidance(self,m):
        if not self.started or self.ending is not None:return
        status=m.data.strip().upper()
        counted=self.interventions.observe(status)
        self.event('avoidance_status',status=status,sim_time=self.get_clock().now().nanoseconds*1e-9,
                   counted=counted,interventions=self.interventions.count,episode_active=self.interventions.active)

    def on_odom(self,m):
        p=m.pose.pose.position;t=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
        v,w=m.twist.twist.linear.x,m.twist.twist.angular.z
        self.odom=(t,p.x,p.y,v,w)
        if not self.started:return
        if self.ending is not None:
            if abs(v)<.02 and abs(w)<.02:self.zero_since=self.zero_since or t
            else:self.zero_since=None
            return
        if self.metrics.add(t,p.x,p.y,v,w):
            self.percent=self.progress.update((p.x,p.y))
            if t-self.last_odom_log>=.049:
                self.writer.writerow([t,p.x,p.y,v,w,self.percent,self.status.get('route_index')]);self.last_odom_log=t

    def begin(self):
        self.start_sim=self.status['sim_time'];self.start_wall=time.monotonic()
        self.progress=CourseProgress(self.odom[1:3],self.status['route_points_odom'])
        self.started=True
        (self.output/'armed').touch()
        self.event('start',status=self.status,origin=self.odom,route=self.progress.points)
        save_json(self.output/'start.json',{'sim_time':self.start_sim,'wall_time':time.time(),'initial_status':self.status,'origin':self.odom})
        self.start_future=self.start_client.call_async(Trigger.Request())

    def finish(self,reason,arrived=False):
        if self.ending is not None:return
        now=time.monotonic()
        end=max(self.status.get('sim_time',0),self.odom[0] if self.odom else 0)
        if arrived and self.odom:self.percent=self.progress.update(self.odom[1:3],arrived=True)
        self.ending={'reason':reason,'goal_reached':arrived,'end_sim':end,'end_wall':now,
                     'end_pose':self.odom,'driver_status':self.status}
        self.event('finish_requested',**self.ending)

    def tick(self):
        if self.done:return
        now=time.monotonic();c=self.contacts()
        if not self.started:
            if now-self.created>self.get_parameter('prepare_wall_sec').value:
                save_json(self.output/'preparation_failure.json',{'reason':'preparation_timeout','status':self.status,'contacts':c})
                self.done=True;return
            s=self.status
            if (s.get('state')=='DISABLED' and s.get('stop_autonomy_off_acknowledged') and s.get('predictions',0)>0
                and s.get('sim_time',0)>=4.1 and self.odom and len(c.get('registered_collisions',[]))==12
                and c.get('physics_steps',0)>100 and not c.get('active_objects')
                and abs(c['sim_time']-s['sim_time'])<.5 and self.start_client.service_is_ready()):
                if s['assistance_enabled']!=self.get_parameter('assistance').value:raise RuntimeError('Assistance mode mismatch')
                self.begin()
            return
        if self.start_future is not None and self.start_future.done():
            response=self.start_future.result();self.start_future=None
            if not response.success:self.finish('start_rejected: '+response.message)
        if self.ending is None:
            if now-self.start_wall>self.get_parameter('max_wall_sec').value:self.finish('wall_timeout')
            elif now-self.last_status_wall>5:self.finish('driver_status_lost')
            elif self.status.get('sim_time',0)-self.start_sim>605:self.finish('simulation_timeout')
            elif not c or self.status.get('sim_time',0)-c.get('sim_time',0)>.5:self.finish('contact_measurement_lost')
            available=next((int(l.split()[1]) for l in Path('/proc/meminfo').read_text().splitlines() if l.startswith('MemAvailable:')),0)
            if available<512*1024:self.finish('low_memory')
        if self.ending is not None:
            # Terminal driver already requests OFF; repeat emergency stop if needed.
            if now-self.last_stop>1 and self.stop_client.service_is_ready():
                self.stop_future=self.stop_client.call_async(Trigger.Request());self.last_stop=now
            stopped=(self.zero_since is not None and self.odom[0]-self.zero_since>=.3
                     and self.status.get('stop_autonomy_off_acknowledged') and self.status.get('command')==[0.,0.])
            if (stopped and c.get('sim_time',0)>=self.ending['end_sim']) or now-self.ending['end_wall']>15:
                self.complete(stopped)
        if now-self.last_checkpoint>1:
            self.trajectory.flush()
            save_json(self.output/'checkpoint.json',{'sim_time':self.status.get('sim_time'),
                'progress_percent':self.percent,'contacts':c,'driver_status':self.status,
                'avoidance_interventions':self.interventions.count})
            self.last_checkpoint=now
        if now-self.last_log>30:
            self.get_logger().info(f"{self.status.get('state')} pose={self.odom} progress={self.percent:.2f}% contacts={c.get('collision_count')}")
            self.last_log=now

    def complete(self,stopped):
        end=self.ending['end_sim']
        contact_events=[]
        for line in (self.output/'contacts.jsonl').read_text().splitlines():
            e=json.loads(line)
            if e.get('type')=='contact_begin' and e.get('armed') and self.start_sim<=e['sim_time']<=end:contact_events.append(e)
        sim=max(0.,end-self.start_sim);wall=self.ending['end_wall']-self.start_wall
        result={'collisions':len(contact_events),'progress_percent':self.percent,'sim_seconds':sim,
                'avoidance_interventions':self.interventions.count,
                'wall_seconds':wall,'rtf':sim/wall if wall else 0,'goal_reached':self.ending['goal_reached'],
                'reason':self.ending['reason'],'start_sim':self.start_sim,'end_sim':end,
                'end_pose':self.ending['end_pose'],'safe_stop_confirmed':stopped,
                'contact_measurement_valid':self.contacts().get('sim_time',0)>=end,
                'motion':self.metrics.summary(),'inference_count':len(self.latencies),
                'median_onnx_ms':statistics.median(self.latencies) if self.latencies else None,
                'final_driver_status':self.ending['driver_status']}
        self.event('result',result=result)
        for f in (self.events,self.trajectory):f.flush();os.fsync(f.fileno())
        with (self.output/'contacts.jsonl').open('a') as f:f.flush();os.fsync(f.fileno())
        save_json(self.output/'result.json',result)
        self.done=True


def main():
    rclpy.init();node=Recorder()
    try:
        while rclpy.ok() and not node.done:rclpy.spin_once(node,timeout_sec=.1)
    except KeyboardInterrupt:
        if node.started:
            node.finish('operator_interrupted')
            deadline=time.monotonic()+18
            while not node.done and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.1)
    finally:
        node.events.close();node.trajectory.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
