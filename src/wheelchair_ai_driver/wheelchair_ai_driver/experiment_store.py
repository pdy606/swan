"""Durable per-run commits are authoritative; CSVs can always be regenerated."""
import csv
import io
import json
import os
from pathlib import Path


def atomic_write(path, text):
    path=Path(path)
    path.parent.mkdir(parents=True,exist_ok=True)
    temporary=path.with_name(path.name+'.tmp')
    with temporary.open('w') as file:
        file.write(text);file.flush();os.fsync(file.fileno())
    os.replace(temporary,path)
    fd=os.open(path.parent,os.O_RDONLY|os.O_DIRECTORY)
    try:os.fsync(fd)
    finally:os.close(fd)


def save_json(path,value):
    atomic_write(path,json.dumps(value,indent=2,allow_nan=False)+'\n')


class ExperimentStore:
    def __init__(self, directory, fingerprint):
        self.directory=Path(directory)
        self.directory.mkdir(parents=True,exist_ok=True)
        manifest=self.directory/'experiment.json'
        if manifest.exists():
            if json.loads(manifest.read_text())!=fingerprint:
                raise ValueError('Experiment configuration changed; choose a new output directory')
        else:save_json(manifest,fingerprint)
        self.on_interventions = fingerprint.get('schema',1) >= 2
        self.refresh()

    def result_path(self,mode,number):
        if mode not in ('off','on') or number<1:raise ValueError('Invalid trial identity')
        return self.directory/'records'/f'{mode}_{number:03d}.json'

    def completed(self,mode,number):return self.result_path(mode,number).exists()

    def commit(self,mode,number,result):
        path=self.result_path(mode,number)
        if path.exists():raise ValueError('Refusing duplicate trial commit')
        result=dict(result,mode=mode,run=number)
        for key in ('collisions','progress_percent','sim_seconds'):
            if not isinstance(result[key],(int,float)):raise ValueError('Missing measured result')
        if mode=='on' and self.on_interventions:
            count=result.get('avoidance_interventions')
            if type(count) is not int or count<0:raise ValueError('Missing measured avoidance count')
        save_json(path,result)
        self.refresh()

    def refresh(self):
        for mode in ('off','on'):
            rows=[]
            for path in sorted((self.directory/'records').glob(f'{mode}_*.json'),key=lambda p:int(p.stem.split('_')[-1])):
                r=json.loads(path.read_text())
                row=(r['run'],r['collisions'],r['progress_percent'],r['sim_seconds'])
                if mode=='on' and self.on_interventions:row+= (r['avoidance_interventions'],)
                rows.append(row)
            stream=io.StringIO(newline='')
            writer=csv.writer(stream)
            header=['실험 번호','충돌 횟수','목적지 도달률(%)','주행 시간(s)']
            if mode=='on' and self.on_interventions:header.append('자동 회피 개입 횟수')
            writer.writerow(header)
            for row in rows:writer.writerow([row[0],row[1],f'{row[2]:.6f}',f'{row[3]:.6f}',*row[4:]])
            if rows:writer.writerow(['평균',*[f'{sum(row[k] for row in rows)/len(rows):.6f}' for k in range(1,len(header))]])
            atomic_write(self.directory/f'swan_{mode}.csv','\ufeff'+stream.getvalue())


def completed_attempt(directory):
    """Administrative interruption is an unfinished attempt, not a driving failure."""
    directory=Path(directory)
    path=directory/'result.json'
    if (directory/'invalid_measurement.json').exists() or not path.exists():
        return None
    result=json.loads(path.read_text())
    if result.get('reason')=='operator_interrupted':
        # Keep all measurements, but resume this logical trial with a fresh simulator.
        save_json(directory/'interrupted_attempt.json',{'reason':'operator_interrupted','count_as_trial':False})
        os.replace(path,directory/'interrupted_result.json')
        fd=os.open(directory,os.O_RDONLY|os.O_DIRECTORY)
        try:os.fsync(fd)
        finally:os.close(fd)
        return None
    return result
