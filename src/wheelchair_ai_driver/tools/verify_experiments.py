#!/usr/bin/env python3
"""Cross-check saved CSVs against immutable results and solver events."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from wheelchair_ai_driver.experiment_store import save_json
from wheelchair_ai_driver.avoidance_metrics import AvoidanceInterventions
from wheelchair_ai_driver.result_paths import active_results

ROOT=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser();p.add_argument('--output')
p.add_argument('--report-dir');args=p.parse_args()
folder=(ROOT/args.output).resolve() if args.output else active_results(ROOT)
report=(ROOT/args.report_dir).resolve() if args.report_dir else folder/'provenance'
assert folder.is_relative_to(ROOT) and report.is_relative_to(ROOT)
if not (folder/'experiment.json').is_file():p.error('아직 시작한 실험이 없습니다. experiment.sh start를 먼저 실행하세요.')
schema=json.loads((folder/'experiment.json').read_text()).get('schema',1)
results=[]
for path in sorted((folder/'records').glob('*.json'),key=lambda p:(p.stem.split('_')[0],int(p.stem.split('_')[1]))):
    result=json.loads(path.read_text())
    attempts=[]
    for candidate in (folder/'runs').glob(f'{result["mode"]}_{result["run"]:03d}_*/result.json'):
        if json.loads(candidate.read_text())=={k:v for k,v in result.items() if k not in ('mode','run')}:attempts.append(candidate.parent)
    assert len(attempts)==1,(path,attempts)
    directory=attempts[0];events=[json.loads(line) for line in (directory/'contacts.jsonl').read_text().splitlines()]
    counted=[e for e in events if e.get('type')=='contact_begin' and e.get('armed') and result['start_sim']<=e['sim_time']<=result['end_sim']]
    assert len(counted)==result['collisions'],path
    active=set()
    for e in events:
        if e['type']=='armed':active.clear()
        elif e['type']=='contact_begin':
            assert e['object'] not in active,('duplicate without release',e)
            active.add(e['object'])
        elif e['type']=='contact_end':
            assert e['object'] in active,('release without contact',e)
            active.remove(e['object'])
    assert 0<=result['progress_percent']<=100
    if result['goal_reached']:assert result['progress_percent']==100
    if schema>=2:
        counter=AvoidanceInterventions(result['mode']=='on')
        for e in map(json.loads,(directory/'validation.jsonl').read_text().splitlines()):
            if e['kind']=='avoidance_status':
                assert counter.observe(e['status'])==e['counted']
                assert counter.count==e['interventions']
        assert counter.count==result['avoidance_interventions']
        # The independent original node log confirms accepted-goal transitions.
        launch=(directory/'launch.log').read_text()
        published=launch.count('AVOIDANCE STATUS: AVOIDING')
        observed=sum(e['kind']=='avoidance_status' and e['status']=='AVOIDING'
                     for e in map(json.loads,(directory/'validation.jsonl').read_text().splitlines()))
        assert published==observed,(path,published,observed)
    results.append(result)
summary={}
for mode in ('off','on'):
    selected=[r for r in results if r['mode']==mode]
    rows=list(csv.reader((folder/f'swan_{mode}.csv').open(encoding='utf-8-sig')))
    keys=['collisions','progress_percent','sim_seconds']
    if mode=='on' and schema>=2:keys.append('avoidance_interventions')
    assert all(len(row)==len(keys)+1 for row in rows)
    assert len(rows)==len(selected)+(2 if selected else 1)
    if selected:
        if len(keys)>3:assert rows[0][-1]=='자동 회피 개입 횟수'
        for row,r in zip(rows[1:-1],selected):
            assert int(row[0])==r['run']
            assert all(abs(float(row[i+1])-r[key])<1e-6 for i,key in enumerate(keys))
        means=[sum(r[k] for r in selected)/len(selected) for k in keys]
        assert rows[-1][0]=='평균'
        assert all(abs(float(rows[-1][i+1])-means[i])<1e-6 for i in range(len(keys)))
        summary[mode]={'trials':len(selected),'mean_collisions':means[0],'mean_progress_percent':means[1],
            'mean_sim_seconds':means[2],'goals_reached':sum(r['goal_reached'] for r in selected),
            'mean_rtf':sum(r['rtf'] for r in selected)/len(selected)}
        if len(keys)>3:summary[mode]['mean_avoidance_interventions']=means[3]
snapshot=report/'preservation.json'
before=json.loads(snapshot.read_text()) if snapshot.is_file() else {}
changed=[name for name,digest in before.items() if hashlib.sha256((Path(name) if Path(name).is_absolute() else folder/name).read_bytes()).hexdigest()!=digest]
assert not changed,changed
verification={'validated_results':len(results),'summary':summary,'original_and_frozen_files_checked':len(before),
    'changed_files':changed,'csv_matches_durable_results':True,'contact_begins_all_follow_release':True}
save_json(report/'verification.json',verification)
print(json.dumps(verification,indent=2))
