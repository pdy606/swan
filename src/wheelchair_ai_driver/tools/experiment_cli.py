#!/usr/bin/env python3
"""Terminal lifecycle manager; driving and evaluation remain in the existing nodes."""
import argparse
import fcntl
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from wheelchair_ai_driver.experiment_store import save_json
from wheelchair_ai_driver.result_paths import active_results,activate_results,select_results,all_attempts,restore_csvs_if_needed


def read_json(path):
    try:return json.loads(path.read_text())
    except (OSError,ValueError):return {}


def process_identity(pid):
    try:return Path(f'/proc/{pid}/stat').read_text().rsplit(')',1)[1].split()[19]
    except (OSError,IndexError):return None


def alive(state):
    pid=state.get('pid')
    if not pid or process_identity(pid)!=state.get('process_identity'):return False
    try:
        args=Path(f'/proc/{pid}/cmdline').read_bytes().split(b'\0')
        return b'_worker' in args and state.get('token','').encode() in args
    except OSError:return False


def completed(root,output=None):
    output=output if output is not None else active_results(root)
    return {mode:len(list((output/'records').glob(f'{mode}_*.json'))) for mode in ('off','on')}


def targets_met(root,targets,output=None):
    output=output if output is not None else active_results(root)
    return all((output/'records'/f'{mode}_{number:03d}.json').is_file()
               for mode,count in targets.items() for number in range(1,count+1))


def start(root,targets,worker_args=None):
    runtime=root/'runtime';runtime.mkdir(exist_ok=True)
    with (runtime/'control.lock').open('a') as lock:
        try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:raise RuntimeError('실험이 이미 실행 중입니다. status 또는 stop을 사용하세요.')
        previous=read_json(runtime/'experiment_state.json')
        if alive(previous):raise RuntimeError('실험이 이미 실행 중입니다.')
        expected=runner_module(root).fingerprint()
        (root/'results').mkdir(exist_ok=True)
        with (root/'results/experiment.lock').open('a') as result_lock:
            try:fcntl.flock(result_lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
            except BlockingIOError:raise RuntimeError('실험 또는 설정이 이미 실행 중입니다.')
            output=select_results(root,expected,create=True)
            restore_csvs_if_needed(output,expected)
            activate_results(root,output)
        output_name=str(output.relative_to(root))
        counts=completed(root,output)
        if targets_met(root,targets,output):
            save_json(runtime/'experiment_state.json',{'status':'complete','targets':targets,'output':output_name,'ended_wall':time.time()})
            print(f'목표가 이미 완료됐습니다. OFF {counts["off"]}회 / ON {counts["on"]}회. 새 주행은 실행하지 않습니다.')
            return None
        token=uuid.uuid4().hex
        command=worker_args or [sys.executable,str(Path(__file__).resolve()),'_worker']
        command=command+['--off',str(targets['off']),'--on',str(targets['on']),'--token',token]
        with (output/'experiment.log').open('a') as log:
            child=subprocess.Popen(command,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,
                                   start_new_session=True,env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1'))
        state={'pid':child.pid,'process_identity':process_identity(child.pid),'token':token,
               'targets':targets,'output':output_name,'status':'preparing','started_wall':time.time()}
        save_json(runtime/'experiment_state.json',state)
    # Do not return until signal handlers are installed, so an immediate stop
    # cannot kill the worker in its interpreter startup window.
    deadline=time.monotonic()+5
    while child.poll() is None and not read_json(runtime/'experiment_state.json').get('worker_ready') and time.monotonic()<deadline:
        time.sleep(.02)
    if child.poll() is not None:raise RuntimeError(f'실험 준비에 실패했습니다. {output_name}/experiment.log를 확인하세요.')
    print(f'백그라운드 시작: PID {child.pid}, 누적 목표 OFF {targets["off"]}회 / ON {targets["on"]}회',flush=True)
    print(f'결과 폴더: {output}',flush=True)
    return child


def prepare_environment(root):
    env=dict(os.environ,AI_ROOT=str(root),PYTHONDONTWRITEBYTECODE='1',
             ROS_LOG_DIR=str(root/'runtime/ros_logs'))
    # Source Jazzy in a child shell; ROS always uses the system interpreter.
    script='source /opt/ros/jazzy/setup.bash\n'
    script+='if [ -f "$AI_ROOT/runtime/install/setup.bash" ]; then source "$AI_ROOT/runtime/install/setup.bash"; fi\n'
    script+='env -0'
    result=subprocess.run(['bash','-e','-c',script],env=env,capture_output=True,check=True)
    env=dict(part.decode().split('=',1) for part in result.stdout.split(b'\0') if b'=' in part)
    env['PYTHONPATH']=str(root)+os.pathsep+env.get('PYTHONPATH','')
    if not (root/'.venv_mbra/bin/python').exists():raise RuntimeError('MBRA 가상환경이 없습니다: .venv_mbra')
    if not (root/'assets/navigation_model_zoo/MBRA_PG_Official/mbra.onnx').exists():raise RuntimeError('MBRA 모델이 없습니다.')
    library=root/'runtime/contact_build/libswan_evaluation_contacts.so'
    if not library.exists():
        subprocess.run(['cmake','-S',str(root/'evaluation'),'-B',str(root/'runtime/contact_build'),'-DCMAKE_BUILD_TYPE=Release'],env=env,check=True)
        subprocess.run(['cmake','--build',str(root/'runtime/contact_build'),'-j2'],env=env,check=True)
    executable=root/'runtime/install/wheelchair_ai_driver/lib/wheelchair_ai_driver/mbra_driver'
    if not executable.exists():
        subprocess.run(['colcon','--log-base',str(root/'runtime/log'),'build','--base-paths',str(root),
                        '--packages-select','wheelchair_ai_driver','--build-base',str(root/'runtime/build'),
                        '--install-base',str(root/'runtime/install'),'--symlink-install'],env=env,check=True)
        sourced=subprocess.run(['bash','-e','-c','source "$AI_ROOT/runtime/install/setup.bash"\nenv -0'],
                               env=env,capture_output=True,check=True)
        env=dict(part.decode().split('=',1) for part in sourced.stdout.split(b'\0') if b'=' in part)
    return env


def worker(root,targets,token,command=None):
    requested=False
    def request_stop(_sig,_frame):
        nonlocal requested
        requested=True
    signal.signal(signal.SIGTERM,request_stop);signal.signal(signal.SIGINT,request_stop)
    state_path=root/'runtime/experiment_state.json';child=None
    # Parent publishes identity before releasing this lock. Worker holds it to exit.
    with (root/'runtime/control.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX)
        state=read_json(state_path)
        if state.get('token')!=token:return 1
        state['worker_ready']=True;save_json(state_path,state)
        try:
            if command is None:stop_orphans(root)
            env=prepare_environment(root) if command is None else os.environ.copy()
            if requested:
                state.update(status='stopped',ended_wall=time.time());save_json(state_path,state);return 0
            state['status']='running';save_json(state_path,state)
            args=command or [sys.executable,str(root/'tools/run_experiments.py'),'--output',state['output'],
                             '--off-runs',str(targets['off']),'--on-runs',str(targets['on'])]
            child=subprocess.Popen(args,env=env,start_new_session=True)
            signaled=False
            while child.poll() is None:
                if requested and not signaled:
                    child.send_signal(signal.SIGINT);signaled=True
                time.sleep(.1)
            output=root/state.get('output','results')
            status='complete' if targets_met(root,targets,output) else ('stopped' if requested else 'failed')
            state.update(status=status,returncode=child.returncode,ended_wall=time.time())
        except Exception as exc:
            state.update(status='failed',error=str(exc),ended_wall=time.time())
            print(f'실험 실행 실패: {exc}',flush=True)
        finally:
            if child is not None and child.poll() is None:
                child.send_signal(signal.SIGINT)
                try:child.wait(timeout=35)
                except subprocess.TimeoutExpired:
                    state.update(status='stopping',error='안전 정지가 아직 진행 중입니다.')
            save_json(state_path,state)
    return 0 if state['status'] in ('complete','stopped') else 1


def runner_module(root):
    spec=importlib.util.spec_from_file_location('experiment_runner',root/'tools/run_experiments.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module


def stop_orphans(root):
    """Only processes bearing our saved, unique Gazebo partition are targeted."""
    runner=runner_module(root)
    for path in all_attempts(root):
        info=read_json(path);marker=info.get('partition')
        if not marker:continue
        pids=runner.own_processes(marker)
        if not pids:continue
        # The recorder's existing SIGINT path requests emergency stop and OFF,
        # waits for physical stopping, and marks an administrative interruption.
        recorders=[]
        for pid in pids:
            try:
                if b'wheelchair_ai_driver.experiment_recorder' in Path(f'/proc/{pid}/cmdline').read_bytes():
                    os.kill(pid,signal.SIGINT);recorders.append(pid)
            except (OSError,ProcessLookupError):pass
        if recorders:
            deadline=time.monotonic()+22
            while any(pid in runner.own_processes(marker) for pid in recorders) and time.monotonic()<deadline:time.sleep(.1)
        else:
            env=prepare_environment(root);env['ROS_DOMAIN_ID']=str(info.get('domain',82))
            try:
                subprocess.run(['ros2','service','call','/ai_driver/emergency_stop','std_srvs/srv/Trigger','{}'],
                               env=env,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,timeout=12)
            except subprocess.TimeoutExpired:pass
        remaining=runner.cleanup(marker)
        if remaining:raise RuntimeError(f'종료되지 않은 실험 프로세스: {remaining}')


def stop(root):
    path=root/'runtime/experiment_state.json';state=read_json(path)
    if alive(state):
        os.kill(state['pid'],signal.SIGTERM)
        print('현재 주행에 안전 정지를 요청했습니다. 결과 저장과 프로세스 정리를 기다립니다.',flush=True)
        deadline=time.monotonic()+45
        while alive(state) and time.monotonic()<deadline:time.sleep(.1)
        if alive(state):raise RuntimeError('안전 정지가 진행 중입니다. status로 확인 후 stop을 다시 실행하세요.')
    stop_orphans(root)
    state=read_json(path)
    if state.get('status')!='complete':
        state.update(status='stopped',ended_wall=time.time());save_json(path,state)
    print('실험이 정지했습니다. 완료 결과는 보존하며 미완료 회차는 다음 start에서 다시 실행합니다.')


def status(root):
    state=read_json(root/'runtime/experiment_state.json');running=alive(state)
    labels={'preparing':'준비 중','running':'실행 중','complete':'목표 완료','stopped':'중단','failed':'오류','stopping':'정지 중'}
    label=labels.get(state.get('status'),'대기')
    if not running and state.get('status') in ('running','preparing','stopping'):label='예기치 않은 중단 — start로 재개 가능'
    counts=completed(root);targets=state.get('targets',{})
    print(f'상태: {label}'+(f' (PID {state["pid"]})' if running else ''))
    print(' / '.join(f'{mode.upper()}: 완료 {counts[mode]}회'+(f' / 목표 {targets[mode]}회' if mode in targets else '') for mode in ('off','on')))
    if state.get('error'):print('오류:',state['error'])
    if running:
        candidates=sorted((active_results(root)/'runs').glob('*/checkpoint.json'),key=lambda p:p.stat().st_mtime)
        if candidates and candidates[-1].stat().st_mtime>=state.get('started_wall',0):
            record=read_json(candidates[-1]);s=record.get('driver_status',{})
            print(f'현재 {candidates[-1].parent.name}: {s.get("state")} / 진행률 {record.get("progress_percent",0):.2f}% / 접촉 {record.get("contacts",{}).get("collision_count",0)}회')
    output=active_results(root)
    print('결과 폴더:',output)
    print('CSV:',output/'swan_off.csv',',',output/'swan_on.csv')


def main():
    p=argparse.ArgumentParser(description='SWAN 누적 ON/OFF 실험')
    p.add_argument('action',choices=['start','status','stop','_worker'])
    p.add_argument('--off',type=int);p.add_argument('--on',type=int);p.add_argument('--token')
    args=p.parse_args();root=ROOT
    try:
        if args.action=='status':status(root)
        elif args.action=='stop':stop(root)
        else:
            counts=completed(root)
            targets={mode:getattr(args,mode) for mode in ('off','on')}
            for mode in targets:
                if targets[mode] is None:
                    value=input(f'SWAN {mode.upper()} 최종 누적 목표 횟수 [현재 {counts[mode]}회]: ').strip()
                    targets[mode]=int(value)
                if targets[mode]<1:raise ValueError('목표 횟수는 1 이상이어야 합니다.')
            if args.action=='_worker':return worker(root,targets,args.token)
            start(root,targets)
    except (OSError,ValueError,RuntimeError,EOFError,KeyboardInterrupt) as exc:
        print(str(exc),file=sys.stderr);return 1
    return 0


if __name__=='__main__':sys.exit(main())
