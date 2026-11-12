#!/usr/bin/env python3
"""Initial provisioning only. Never changes driving code or experiment records."""
import argparse
import fcntl
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time
import urllib.request
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
SYSTEM_PYTHON = '/usr/bin/python3'
ROS_SETUP = Path('/opt/ros/jazzy/setup.bash')
PREREQUISITES = '''Ubuntu 24.04 / system Python 3.12, ROS2 Jazzy, Gazebo Harmonic가 필요합니다.
관리자가 ROS/Gazebo 공식 apt 저장소를 구성한 뒤 필요한 패키지를 설치하세요:
  sudo apt install python3-venv python3-colcon-common-extensions python3-numpy python3-yaml \\
    build-essential cmake nlohmann-json3-dev ros-jazzy-desktop ros-jazzy-ros-gz \\
    ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-laser-filters \\
    ros-jazzy-gz-sim-vendor ros-jazzy-gz-plugin-vendor
이 스크립트는 sudo/apt를 실행하지 않습니다. 전체 SWAN 저장소를 함께 클론해야 합니다.
실험에는 카메라 렌더링용 그래픽 세션(DISPLAY), RAM 8GB 이상을 권장합니다.'''


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def save_report(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix('.tmp')
    with temporary.open('w') as stream:
        json.dump(value, stream, indent=2, ensure_ascii=False)
        stream.write('\n')
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)


def run(command, env=None, capture=False, timeout=None):
    return subprocess.run(command, env=env, check=True, text=True,
                          stdout=subprocess.PIPE if capture else None,
                          stderr=subprocess.PIPE if capture else None, timeout=timeout)


def source_environment(root):
    if not ROS_SETUP.is_file():
        raise RuntimeError(f'ROS2 Jazzy가 없습니다: {ROS_SETUP}\n{PREREQUISITES}')
    env = dict(os.environ, AI_ROOT=str(root), PYTHONDONTWRITEBYTECODE='1',
               ROS_LOG_DIR=str(root / 'runtime/ros_logs'))
    for key in ('PYTHONHOME', 'VIRTUAL_ENV'):
        env.pop(key, None)
    result = run(['bash', '-e', '-c', 'source /opt/ros/jazzy/setup.bash\nenv -0'],
                 env=env, capture=True)
    return dict(part.split('=', 1) for part in result.stdout.split('\0') if '=' in part)


def prerequisites(root, env):
    if sys.version_info[:2] != (3, 12):
        raise RuntimeError(f'Jazzy용 system Python 3.12가 필요합니다.\n{PREREQUISITES}')
    missing = [name for name in ('cmake', 'c++', 'colcon', 'gz', 'ldd')
               if not shutil.which(name, path=env.get('PATH'))]
    if missing:
        raise RuntimeError(f'필수 명령 누락: {", ".join(missing)}\n{PREREQUISITES}')
    if not (root / '.venv_mbra/bin/python').exists():
        try:
            run([SYSTEM_PYTHON, '-B', '-c', 'import venv, ensurepip'], env=env, capture=True)
        except subprocess.CalledProcessError as error:
            raise RuntimeError('새 가상환경 생성에 필요한 python3-venv가 없습니다.\n' + PREREQUISITES) from error
    for relative in ('wheelchair_gazebo/worlds/wheelchair_world.sdf',
                     'wheelchair_gazebo/scripts/swan_drive_controller.py',
                     'wheelchair_navigation/scripts/nav_avoidance_node.py',
                     'wheelchair_navigation/config/nav2_params.yaml',
                     'swan_pipeline/swan_pipeline/situation_node.py'):
        if not (root.parent / relative).is_file():
            raise RuntimeError(f'원본 SWAN 소스 누락: {relative}. 전체 저장소를 클론하세요.')
    packages = {element.text.strip() for element in ET.parse(root / 'package.xml').findall('exec_depend')}
    packages = {name for name in packages if not name.startswith('wheelchair_')}
    packages.update(('nav2_mppi_controller', 'nav2_navfn_planner',
                     'nav2_costmap_2d', 'nav2_msgs', 'tf2_msgs', 'rosgraph_msgs'))
    probe = '''import importlib,json
from ament_index_python.packages import get_package_prefix
missing=[]
for name in PACKAGES:
    try: get_package_prefix(name)
    except LookupError: missing.append(name)
for name in ['rclpy','numpy','yaml','rcl_interfaces.msg','nav2_msgs.action',
             'sensor_msgs.msg','geometry_msgs.msg','nav_msgs.msg','std_msgs.msg',
             'std_srvs.srv','tf2_ros','launch','launch_ros']:
    try: importlib.import_module(name)
    except ImportError: missing.append('Python:'+name)
print(json.dumps(missing))
'''.replace('PACKAGES', repr(sorted(packages)))
    missing = json.loads(run([SYSTEM_PYTHON, '-B', '-c', probe], env=env, capture=True).stdout)
    if missing:
        raise RuntimeError(f'ROS/Python 패키지 누락: {", ".join(missing)}\n{PREREQUISITES}')
    versions = run(['gz', 'sim', '--versions'], env=env, capture=True, timeout=30).stdout
    if not any(word.startswith('8.') for word in versions.split()):
        raise RuntimeError(f'Gazebo Harmonic(Sim 8)이 필요합니다: {versions.strip()}')
    # Configure separately: do not touch an already working contact library/cache.
    try:
        run(['cmake', '--log-level=WARNING', '-S', str(root / 'evaluation'), '-B', str(root / 'runtime/setup/cmake_probe'),
             '-DCMAKE_BUILD_TYPE=Release'], env=env)
    except subprocess.CalledProcessError as error:
        raise RuntimeError('Gazebo C++ 개발 의존성 검사 실패.\n' + PREREQUISITES) from error
    if not env.get('DISPLAY'):
        raise RuntimeError('DISPLAY가 없습니다. 기존 카메라 렌더링에 필요한 그래픽 세션의 터미널에서 실행하세요.')
    # Check the X connection without starting Gazebo or any ROS node.
    run([SYSTEM_PYTHON, '-B', '-c',
         'import ctypes; x=ctypes.CDLL("libX11.so.6"); x.XOpenDisplay.restype=ctypes.c_void_p; '
         'd=x.XOpenDisplay(None); '
         'assert d, "DISPLAY에 연결할 수 없습니다"; '
         'x.XCloseDisplay.argtypes=[ctypes.c_void_p]; x.XCloseDisplay(d)'], env=env, capture=True)


def model_url(manifest):
    return (f'https://huggingface.co/{manifest["repository"]}/resolve/'
            f'{manifest["revision"]}/MBRA_PG_Official/mbra.onnx')


def verify_file(path, expected):
    return path.is_file() and path.stat().st_size == expected['bytes'] and sha256(path) == expected['sha256']


def ensure_model(root, check_only=False, opener=urllib.request.urlopen):
    directory = root / 'assets/navigation_model_zoo/MBRA_PG_Official'
    manifest = json.loads((directory / 'manifest.json').read_text())
    for name, expected in manifest['files'].items():
        if name != 'mbra.onnx' and not verify_file(directory / name, expected):
            raise RuntimeError(f'공식 참조 파일 손상/누락: {name}')
    path = directory / 'mbra.onnx'
    expected = manifest['files']['mbra.onnx']
    if path.exists():
        if not verify_file(path, expected):
            raise RuntimeError('기존 mbra.onnx가 공식 크기/SHA-256과 다릅니다. 덮어쓰지 않고 중단합니다.')
        print('[model] 기존 공식 모델 검증 완료; 다운로드 생략', flush=True)
        return expected
    if check_only:
        raise RuntimeError('MBRA 모델이 없습니다. ./setup.sh를 실행하세요.')
    # Keep even an uncatchable interruption's partial download under *.onnx ignores.
    temporary = directory / '.setup-download-mbra.onnx'
    try:
        for attempt in range(3):
            try:
                print(f'[model] 공식 고정 revision 다운로드 ({expected["bytes"]:,} bytes)', flush=True)
                request = urllib.request.Request(model_url(manifest), headers={'User-Agent': 'SWAN-setup/1.0'})
                with opener(request, timeout=60) as response, temporary.open('wb') as output:
                    count = 0
                    while chunk := response.read(1024 * 1024):
                        count += len(chunk)
                        if count > expected['bytes']:
                            raise RuntimeError('다운로드 크기가 공식 모델 크기를 초과했습니다.')
                        output.write(chunk)
                    output.flush()
                    os.fsync(output.fileno())
                if not verify_file(temporary, expected):
                    raise RuntimeError('다운로드 크기/SHA-256 검증 실패')
                # link is atomic and refuses to overwrite a file created concurrently.
                os.link(temporary, path)
                return expected
            except (OSError, RuntimeError) as error:
                if path.exists() or attempt == 2:
                    raise RuntimeError(f'모델 다운로드 실패: {error}') from error
                time.sleep(attempt + 1)
    finally:
        temporary.unlink(missing_ok=True)


def dependency_versions(python, requirements):
    wanted = dict(line.strip().split('==', 1) for line in requirements.read_text().splitlines()
                  if line.strip() and not line.startswith('#'))
    code = '''import importlib.metadata as m,json,sys
from pathlib import Path
assert sys.version_info[:2] == (3,12), 'MBRA Python 3.12가 필요합니다'
assert sys.prefix != sys.base_prefix, '격리된 Python 가상환경이 필요합니다'
config=dict(line.split('=',1) for line in (Path(sys.prefix)/'pyvenv.cfg').read_text().splitlines() if '=' in line)
assert any(k.strip()=='include-system-site-packages' and v.strip()=='false' for k,v in config.items()), 'system-site-packages 격리가 필요합니다'
wanted=WANTED
found={}
for name in wanted:
    try: found[name]=m.version(name)
    except m.PackageNotFoundError: found[name]=None
print(json.dumps(found))
'''.replace('WANTED', repr(wanted))
    actual = json.loads(run([str(python), '-I', '-B', '-c', code], capture=True).stdout)
    return wanted, actual


def ensure_venv(root, check_only=False):
    directory = root / '.venv_mbra'
    python = directory / 'bin/python'
    marker = root / 'runtime/setup/venv_incomplete.json'
    requirements = root / 'requirements/cpu.lock.txt'
    created = False
    if marker.exists():
        try:
            pending = json.loads(marker.read_text())
        except (OSError, ValueError) as error:
            raise RuntimeError('미완료 가상환경 기록을 확인할 수 없습니다. 기존 환경은 수정하지 않습니다.') from error
        if pending != {'directory': str(directory), 'created_by': 'setup_environment.py'}:
            raise RuntimeError('이 위치에서 생성한 미완료 가상환경이 아닙니다. 수정하지 않습니다.')
    if directory.exists():
        if not python.exists():
            if not marker.is_file():
                raise RuntimeError('기존 .venv_mbra가 불완전합니다. 변경하지 않고 중단합니다.')
        else:
            wanted, actual = dependency_versions(python, requirements)
            if wanted == actual:
                # Finish a provisioning interrupted after pip succeeded, retiring
                # permission to resume installation into this now-ready environment.
                marker.unlink(missing_ok=True)
                print('[venv] 기존 고정 의존성 일치; 설치 생략', flush=True)
                return actual
            if check_only or not marker.is_file():
                raise RuntimeError(f'기존 가상환경 버전 불일치; 수정하지 않습니다.\n필요: {wanted}\n현재: {actual}')
    if check_only:
        raise RuntimeError('준비된 MBRA 가상환경이 없습니다. ./setup.sh를 실행하세요.')
    if not directory.exists() or marker.is_file():
        save_report(marker, {'directory': str(directory), 'created_by': 'setup_environment.py'})
        run([SYSTEM_PYTHON, '-B', '-m', 'venv', str(directory)])
        created = True
    if created:
        run([str(python), '-I', '-B', '-m', 'pip', 'install', '--disable-pip-version-check',
             '--no-cache-dir', '--only-binary=:all:', '-r', str(requirements)])
    wanted, actual = dependency_versions(python, requirements)
    if wanted != actual:
        raise RuntimeError('새 가상환경의 고정 의존성 검증 실패')
    run([str(python), '-I', '-B', '-m', 'pip', 'check'])
    marker.unlink(missing_ok=True)
    return actual


def build_and_check(root, env, check_only=False):
    library = root / 'runtime/contact_build/libswan_evaluation_contacts.so'
    if not library.exists():
        if check_only:
            raise RuntimeError('충돌 계측 플러그인이 빌드되지 않았습니다.')
        run(['cmake', '--log-level=WARNING', '-S', str(root / 'evaluation'), '-B', str(library.parent),
             '-DCMAKE_BUILD_TYPE=Release'], env=env)
        run(['cmake', '--build', str(library.parent), '-j2'], env=env)
    linked = run(['ldd', str(library)], env=env, capture=True).stdout
    if 'not found' in linked:
        raise RuntimeError(f'충돌 플러그인 공유 라이브러리 누락:\n{linked}')
    install = root / 'runtime/install'
    executables = [install / 'wheelchair_ai_driver/lib/wheelchair_ai_driver' / name
                   for name in ('mbra_driver', 'experiment_recorder')]
    if not (install / 'setup.bash').is_file() or not all(p.is_file() and os.access(p, os.X_OK) for p in executables):
        if check_only:
            raise RuntimeError('AI ROS2 패키지가 빌드되지 않았습니다.')
        run(['colcon', '--log-base', str(root / 'runtime/log'), 'build', '--base-paths', str(root),
             '--packages-select', 'wheelchair_ai_driver', '--build-base', str(root / 'runtime/build'),
             '--install-base', str(install), '--symlink-install'], env=env)
    script = '''source "$AI_ROOT/runtime/install/setup.bash"
/usr/bin/python3 -B -c 'import wheelchair_ai_driver.driver_node, wheelchair_ai_driver.experiment_recorder
from ament_index_python.packages import get_package_prefix
print(get_package_prefix("wheelchair_ai_driver"))'
'''
    output = run(['bash', '-e', '-c', script], env=env, capture=True).stdout
    if str(install / 'wheelchair_ai_driver') not in output.splitlines():
        raise RuntimeError('AI ROS2 패키지 설치 경로 검증 실패')
    print('[build] 충돌 플러그인 및 ROS2 실행 모듈 검증 완료', flush=True)


def check_protocol(root):
    # Preview a local dataset; its fingerprint is created on first experiment start.
    sys.path.insert(0, str(root))
    spec = importlib.util.spec_from_file_location('swan_setup_runner', root / 'tools/run_experiments.py')
    runner = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(runner)
    actual = runner.fingerprint()
    from wheelchair_ai_driver.result_paths import select_results
    directory = select_results(root, actual)
    print(f'[results] 다음 실험 결과 폴더: {directory} (기존 기록 보존)', flush=True)
    return actual


def check_inference(root):
    available = next(int(line.split()[1]) for line in Path('/proc/meminfo').read_text().splitlines()
                     if line.startswith('MemAvailable:'))
    if available < 1536 * 1024:
        raise RuntimeError('가용 RAM이 1.5GB 미만입니다. CPU 검증을 시작하지 않습니다.')
    code = '''import sys,base64,json
sys.path.insert(0,sys.argv[1])
from wheelchair_ai_driver.mbra_adapter import MbraAdapter
adapter=MbraAdapter(sys.argv[2],provider='cpu',threads=2)
frame={'encoding':'rgb8','width':96,'height':96,'step':288,
       'data':base64.b64encode(bytes(96*96*3)).decode()}
result=adapter.infer([frame]*6,[3.,0.])
print(json.dumps({'contract':adapter.info,'inference_ms':result['total_ms'],
                  'input_shapes':result['input_shapes'],'waypoints':len(result['raw_waypoints'])}))
'''
    result = run([str(root / '.venv_mbra/bin/python'), '-I', '-B', '-c', code, str(root),
                  str(root / 'assets/navigation_model_zoo/MBRA_PG_Official/mbra.onnx')],
                 capture=True, timeout=120)
    info = json.loads(result.stdout)
    print(f'[CPU] {info["input_shapes"]} → 8×4, {info["inference_ms"]:.1f}ms', flush=True)
    return info


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true', help='설치/다운로드/본 빌드 없이 준비 상태 재검증')
    args = parser.parse_args()
    root = ROOT
    runtime = root / 'runtime'
    runtime.mkdir(exist_ok=True)
    if not (runtime / 'COLCON_IGNORE').exists():
        (runtime / 'COLCON_IGNORE').touch()
    report = {'status': 'checking', 'root': str(root), 'check_only': args.check}
    locks = []
    try:
        # Respect both terminal manager and direct runner, without signalling either.
        for path in (runtime / 'setup.lock', runtime / 'control.lock', root / 'results/experiment.lock'):
            path.parent.mkdir(parents=True, exist_ok=True)
            lock = path.open('a')
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                lock.close()
                raise RuntimeError('설정 또는 실험이 실행 중입니다. 종료 후 다시 설정하세요.')
            locks.append(lock)
        env = source_environment(root)
        prerequisites(root, env)
        if not args.check and shutil.disk_usage(root).free < 1024 ** 3:
            raise RuntimeError('여유 디스크 1GB 이상이 필요합니다. 기존 파일을 자동 삭제하지 않습니다.')
        report['model'] = ensure_model(root, args.check)
        report['dependencies'] = ensure_venv(root, args.check)
        build_and_check(root, env, args.check)
        report['protocol'] = check_protocol(root)
        report['cpu'] = check_inference(root)
        report['status'] = 'ready'
        save_report(runtime / 'setup/report.json', report)
        print('\n준비 완료. 모델·기존 가상환경·실험 결과는 덮어쓰지 않았습니다.\n'
              '  ./experiment.sh start --off 20 --on 20', flush=True)
        return 0
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError, KeyboardInterrupt) as error:
        detail = str(error)
        if isinstance(error, subprocess.CalledProcessError):
            detail += '\n' + (error.stderr or error.stdout or '')
        report.update(status='failed', error=detail)
        save_report(runtime / 'setup/report.json', report)
        print(f'\n설정 실패 (준비 완료 아님): {detail}\n검증 기록: runtime/setup/report.json', file=sys.stderr)
        return 1
    finally:
        for lock in reversed(locks):
            lock.close()


if __name__ == '__main__':
    raise SystemExit(main())
