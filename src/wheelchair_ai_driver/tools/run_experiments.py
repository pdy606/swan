#!/usr/bin/env python3
"""Sequential fresh-simulator trials, durable commits, resume by trial identity."""
import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import signal
import shutil
import subprocess
import sys
import time
from wheelchair_ai_driver.experiment_store import ExperimentStore,save_json,completed_attempt
from wheelchair_ai_driver.result_paths import select_results,activate_results

ROOT=Path(__file__).resolve().parents[1]

def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()

def fingerprint():
    files=[ROOT/'config/driving.yaml',ROOT/'wheelchair_ai_driver/driver_node.py',
        ROOT/'wheelchair_ai_driver/trajectory_control.py',ROOT/'wheelchair_ai_driver/inference_worker.py',
        ROOT/'wheelchair_ai_driver/mbra_adapter.py',ROOT/'wheelchair_ai_driver/derived_world.py',
        ROOT/'wheelchair_ai_driver/evaluation_world.py',ROOT/'evaluation/contacts.cc',
        ROOT/'wheelchair_ai_driver/experiment_metrics.py',ROOT/'wheelchair_ai_driver/experiment_recorder.py',
        ROOT/'wheelchair_ai_driver/avoidance_metrics.py',ROOT/'wheelchair_ai_driver/experiment_store.py',
        ROOT/'assets/navigation_model_zoo/MBRA_PG_Official/mbra.onnx']
    for package in ('wheelchair_gazebo','wheelchair_navigation','swan_pipeline'):
        files.extend(p for p in (ROOT.parent/package).rglob('*') if p.is_file() and p.suffix in ('.sdf','.yaml','.py'))
    return {'schema':2,'contact_plugin_sha256':digest(ROOT/'runtime/contact_build/libswan_evaluation_contacts.so'),'files':{str(p.relative_to(ROOT.parent)):digest(p) for p in sorted(set(files))},
            'goal':[53,0],'max_user_speed':.30,'camera_hfov_deg':90,'physics_step_sec':.001,
            'run_sim_timeout_sec':600,'run_wall_timeout_sec':3000,
            'progress':'furthest ordered course projection; 2m corridor; guide radius .8; final .5',
            'contacts':'all source wheelchair collisions incl. mounted occupant dummy; external-model event; exact solver-step separation',
            'avoidance_interventions':'first Nav2-accepted AVOIDING per episode; replans do not increment',
            'gui':False,'camera_viewer':False,'video_recording':False,'nav2_launched_both_modes':True}

def own_processes(marker):
    found=[]
    needle=('GZ_PARTITION='+marker).encode()
    for directory in Path('/proc').iterdir():
        if not directory.name.isdecimal():continue
        try:
            if needle in (directory/'environ').read_bytes().split(b'\0'):found.append(int(directory.name))
        except (OSError,PermissionError):pass
    return found

def cleanup(marker):
    # Unique per-attempt environment marker excludes all user or unrelated processes.
    for sig,wait in ((signal.SIGINT,5),(signal.SIGTERM,3),(signal.SIGKILL,1)):
        for pid in own_processes(marker):
            try:os.kill(pid,sig)
            except ProcessLookupError:pass
        deadline=time.monotonic()+wait
        while own_processes(marker) and time.monotonic()<deadline:time.sleep(.2)
    return own_processes(marker)

def main():
    p=argparse.ArgumentParser()
    p.add_argument('--output',help='Explicit dataset directory; omitted: select local matching environment')
    p.add_argument('--runs',type=int,default=3,help='Total trials per mode, including already committed trials')
    p.add_argument('--off-runs',type=int)
    p.add_argument('--on-runs',type=int)
    p.add_argument('--domain',type=int,default=82)
    args=p.parse_args()
    targets={'off':args.off_runs if args.off_runs is not None else args.runs,
             'on':args.on_runs if args.on_runs is not None else args.runs}
    if min(targets.values())<1:raise ValueError('Invalid run count')
    base=ROOT/'results';base.mkdir(exist_ok=True)
    lock=(base/'experiment.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    expected=fingerprint()
    output=(ROOT/args.output).resolve() if args.output else select_results(ROOT,expected,create=True)
    if not output.is_relative_to(base.resolve()):raise ValueError('Output must be inside results/')
    store=ExperimentStore(output,expected)
    activate_results(ROOT,output)
    # Salvage final JSONs written just before an interrupted orchestrator could commit.
    for pending in sorted((output/'runs').glob('*/attempt.json')):
        info=json.loads(pending.read_text());result=pending.parent/'result.json'
        invalid=(pending.parent/'invalid_measurement.json').exists()
        leftovers=own_processes(info['partition'])
        if leftovers:cleanup(info['partition'])
        if invalid:continue
        recovered=completed_attempt(pending.parent)
        if recovered is not None and not store.completed(info['mode'],info['run']):
            store.commit(info['mode'],info['run'],recovered)
    current=None;recorder=None;directory=None;mode=None;number=None
    def interrupted(_sig,_frame):raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,interrupted)
    try:
        for number in range(1,max(targets.values())+1):
            for mode in ('off','on'):
                if number>targets[mode]:continue
                if store.completed(mode,number):continue
                if fingerprint()!=expected:raise RuntimeError('Source/settings changed during the batch; preserving previous results and stopping')
                attempt=f'{mode}_{number:03d}_{time.time_ns()}'
                directory=output/'runs'/attempt;directory.mkdir(parents=True)
                marker=f'swan_eval_{attempt}';current=marker
                env=dict(os.environ,ROS_DOMAIN_ID=str(args.domain),GZ_PARTITION=marker,
                    ROS_LOG_DIR=str(directory/'ros_logs'),PYTHONDONTWRITEBYTECODE='1',EGL_PLATFORM='x11',QT_QPA_PLATFORM='xcb')
                save_json(directory/'attempt.json',{'mode':mode,'run':number,'partition':marker,'domain':args.domain,'start_wall':time.time()})
                print(f'START {mode.upper()} {number}: {directory}',flush=True)
                with (directory/'launch.log').open('w') as launch_log,(directory/'recorder.log').open('w') as rec_log:
                    launch=subprocess.Popen(['ros2','launch','wheelchair_ai_driver','driving.launch.py',
                        'gui:=false','with_nav2:=true','with_vision:=false',f'assistance:={str(mode=="on").lower()}',
                        f'evaluation_dir:={directory}'],env=env,stdout=launch_log,stderr=subprocess.STDOUT,start_new_session=True)
                    recorder=subprocess.Popen([sys.executable,'-m','wheelchair_ai_driver.experiment_recorder',
                        '--ros-args','-p','use_sim_time:=true','-p',f'output_dir:={directory}',
                        '-p',f'assistance:={str(mode=="on").lower()}'],env=env,stdout=rec_log,stderr=subprocess.STDOUT,start_new_session=True)
                    deadline=time.monotonic()+3230
                    while recorder.poll() is None:
                        if time.monotonic()>deadline:raise RuntimeError('Recorder exceeded bounded lifetime')
                        if launch.poll() is not None:raise RuntimeError('Simulator launch terminated unexpectedly')
                        time.sleep(1)
                    if not (directory/'result.json').exists():raise RuntimeError(f'No measured result; inspect {directory}')
                    if (directory/'invalid_measurement.json').exists():raise RuntimeError('Known invalid measurement; result excluded')
                    result=completed_attempt(directory)
                    if result is None:raise KeyboardInterrupt
                    store.commit(mode,number,result)
                    print(f'COMMITTED {mode.upper()} {number}: contacts={result["collisions"]}, progress={result["progress_percent"]:.2f}%, sim={result["sim_seconds"]:.2f}s, reason={result["reason"]}',flush=True)
                leftovers=cleanup(marker);current=None;recorder=None
                if leftovers:raise RuntimeError(f'Owned simulator processes could not stop: {leftovers}')
                # launch.log already captures node output. Per-node ROS copies
                # are redundant after every owned process has exited.
                shutil.rmtree(directory/'ros_logs',ignore_errors=True)
                if not result['contact_measurement_valid']:raise RuntimeError('Contact measurement lost; stopped batch for investigation')
        print(f'COMPLETE. CSVs: {output}',flush=True)
    except KeyboardInterrupt:
        print('Stopping current attempt; preserving committed trials.',flush=True)
        if recorder and recorder.poll() is None:
            recorder.send_signal(signal.SIGINT)
            try:recorder.wait(timeout=20)
            except subprocess.TimeoutExpired:pass
        if directory and not store.completed(mode,number):
            recovered=completed_attempt(directory)
            if recovered is not None:store.commit(mode,number,recovered)
    finally:
        if current:cleanup(current)
        store.refresh()

if __name__=='__main__':main()
