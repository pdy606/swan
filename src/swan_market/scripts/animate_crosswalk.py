#!/usr/bin/env python3
"""Deterministic kinematic crossing traffic, driven by Gazebo simulation time.

Two ridden motorcycles follow a continuous closed loop. They wait away from the
crosswalk while two pedestrians cross in opposite directions. Pose commands move
collision shapes but do not simulate tire forces, walking joints or avoidance.
No wheelchair commands are published. Pausing Gazebo freezes the phase.
"""
import argparse,json,math,signal,threading,time,multiprocessing,queue
from pathlib import Path


def loop_pose(distance,cfg):
    r=cfg['lane_radius'];h=cfg['road_straight_half_length'];x=cfg['road_center_x']
    straight=2*h;arc=math.pi*r;d=distance%(2*straight+2*arc)
    if d<straight:return x+r,-h+d,math.pi/2
    d-=straight
    if d<arc:
        a=d/r;return x+r*math.cos(a),h+r*math.sin(a),a+math.pi/2
    d-=arc
    if d<straight:return x-r,h-d,-math.pi/2
    d-=straight;a=math.pi+d/r
    return x+r*math.cos(a),-h+r*math.sin(a),a+math.pi/2


def traffic_poses(elapsed,cfg):
    cycle=cfg['cycle_seconds'];lap=int(max(elapsed,0)//cycle);phase=max(elapsed,0)%cycle
    half_loop=2*cfg['road_straight_half_length']+math.pi*cfg['lane_radius']
    travelled=(lap+min(phase/cfg['bike_move_seconds'],1))*half_loop
    poses={name:loop_pose(travelled+i*half_loop,cfg) for i,name in enumerate(cfg['motorcycle_names'])}
    u=max(0,min(1,(phase-cfg['pedestrian_start'])/cfg['pedestrian_duration']))
    for i,name in enumerate(cfg['pedestrian_names']):
        forward=(lap+i)%2==0
        start,end=(cfg['pedestrian_west_x'],cfg['pedestrian_east_x']) if forward else (cfg['pedestrian_east_x'],cfg['pedestrian_west_x'])
        poses[name]=(start+(end-start)*u,(-.75 if i==0 else .75),0 if forward else math.pi)
    stage='motorcycles' if phase<cfg['bike_move_seconds'] else ('pedestrians' if cfg['pedestrian_start']<=phase<cfg['pedestrian_start']+cfg['pedestrian_duration'] else 'waiting')
    return poses,stage,phase


def pose_sender(service, incoming, results):
    # Gazebo Python subscription callbacks can hold the transport dispatch thread
    # while a synchronous request holds the GIL. Isolate requests in a process
    # without subscriptions so acknowledgements and pose callbacks stay live.
    from gz.transport13 import Node
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.msgs10.boolean_pb2 import Boolean
    node=Node()
    while True:
        poses=incoming.get()
        if poses is None:return
        request=Pose_V()
        for name,(x,y,yaw) in poses.items():
            p=request.pose.add();p.name=name;p.position.x=x;p.position.y=y;p.position.z=0
            p.orientation.z=math.sin(yaw/2);p.orientation.w=math.cos(yaw/2)
        ok,response=node.request(service,request,Pose_V,Boolean,500)
        results.put(bool(ok and response.data))


def main():
    from gz.transport13 import Node
    from gz.msgs10.clock_pb2 import Clock
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.msgs10.boolean_pb2 import Boolean
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--layout',required=True);parser.add_argument('--status',default='/tmp/market-traffic-status.json')
    parser.add_argument('--world', help='Override the target world name')
    args=parser.parse_args();layout=json.loads(Path(args.layout).read_text());cfg=layout['traffic'];world=args.world or layout['world']
    node=Node();lock=threading.Lock();clock=[None];observed={};running=[True]
    names=set(cfg['pedestrian_names']+cfg['motorcycle_names'])
    def on_clock(m):
        with lock:clock[0]=m.sim.sec+m.sim.nsec*1e-9
    def on_pose(m):
        with lock:
            for p in m.pose:
                if p.name in names:observed[p.name]=[p.position.x,p.position.y]
    def stop(*_):running[0]=False
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    node.subscribe(Clock,f'/world/{world}/clock',on_clock)
    node.subscribe(Pose_V,f'/world/{world}/pose/info',on_pose)
    context=multiprocessing.get_context('spawn')
    incoming=context.Queue(maxsize=1);results=context.Queue()
    sender=context.Process(target=pose_sender,args=(f'/world/{world}/set_pose_vector',incoming,results),daemon=True)
    sender.start()
    start=None;last=-1.;last_status=0.;success=0;failures=0;trace=[]
    path=Path(args.status)
    print('Crosswalk traffic ready: 2 pedestrians + 2 motorcycles; simulation-time cycle 32 s',flush=True)
    while running[0]:
        with lock:now=clock[0];actual=dict(observed)
        if now is None:time.sleep(.02);continue
        if start is None or now<last:start=now;last=-1.
        if now-last<.05:time.sleep(.01);continue
        elapsed=now-start;poses,stage,phase=traffic_poses(elapsed,cfg)
        try:incoming.put_nowait(poses)
        except queue.Full:
            try:incoming.get_nowait()
            except queue.Empty:pass
            try:incoming.put_nowait(poses)
            except queue.Full:pass
        while True:
            try:ok=results.get_nowait()
            except queue.Empty:break
            if ok:success+=1
            else:
                failures+=1
                if failures<5:print('Waiting for pose command service / model creation',flush=True)
        last=now
        if time.monotonic()-last_status>=1:
            row={'elapsed_sim_s':elapsed,'phase':stage,'cycle_phase_s':phase,'commanded':poses,'observed':actual}
            trace.append(row);trace=trace[-180:]
            payload={**row,'successful_updates':success,'failed_updates':failures,'recent_samples':trace}
            temp=path.with_suffix('.tmp');temp.write_text(json.dumps(payload,indent=2));temp.replace(path)
            last_status=time.monotonic()
    sender.terminate();sender.join(timeout=2)
    print('Crosswalk traffic stopped; actors remain at their last poses.',flush=True)


if __name__=='__main__':main()
