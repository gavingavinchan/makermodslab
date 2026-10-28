"""No-inference integration test and exact A/B pgrep proof."""
import datetime as dt,hashlib,json,os,subprocess as sp,time
from pathlib import Path
from snapshot import snapshot,ROOT
PYTHON='/home/gavin/dreamtrial/.venv/bin/python'
OUT=ROOT/'runs/shackle_look/v2'

def guard():
    result=sp.run(['pgrep','-f','[a]stra_op/server.py'],capture_output=True,text=True)
    pids=[int(x) for x in result.stdout.split()]
    return dict(argv=['pgrep','-f','[a]stra_op/server.py'],returncode=result.returncode,pids=pids,
                processes={str(pid):(Path(f'/proc/{pid}/cmdline').read_bytes().decode().replace('\0',' ') if Path(f'/proc/{pid}/cmdline').exists() else 'exited') for pid in pids})

def protected_all():
    paths=[]
    for directory in ('astra_learn','astra_op'):
        paths.extend(p for p in (ROOT/directory).rglob('*') if p.is_file())
    paths.extend(p for p in (ROOT/'runs/astra_flange/ab').rglob('*') if p.is_file())
    paths.extend(p for p in ROOT.rglob('ledger*') if p.is_file())
    return {str(p.relative_to(ROOT)):dict(sha256=hashlib.sha256(p.read_bytes()).hexdigest(),mtime_ns=p.stat().st_mtime_ns) for p in paths}

if __name__=='__main__':
    import sys
    verify_only = '--verify-only' in sys.argv
    proof = json.loads((OUT/'smoke_proof.json').read_text()) if verify_only else dict(started_at=dt.datetime.now(dt.timezone.utc).isoformat(),before_guard=guard(),protected_before=protected_all(),episodes=[])
    (OUT/'smoke_proof.json').write_text(json.dumps(proof,indent=2)+'\n')
    for suffix,lessons in ([] if verify_only else [('L',True),('N',False)]):
        name='shackle_smoke_20261028_'+suffix
        cmd=[PYTHON,str(ROOT/'shackle_learn/run_episode.py'),'--episode',name,'--prompt',str(ROOT/('ASTRA_OPERATOR_PROMPT_SHACKLE_LESSONS.md' if lessons else 'ASTRA_OPERATOR_PROMPT_SHACKLE.md')),'--wall-cap-min','2','--operator-cmd','./robot status && sleep 5 && ./robot done "smoke"']
        if lessons:cmd+=['--lessons',str(ROOT/'astra_learn/lessons/v006.md')]
        record=dict(episode=name,runner_command=cmd)
        env=os.environ.copy();env.update(MUJOCO_GL='egl',PYTHONDONTWRITEBYTECODE='1',TMPDIR=str(ROOT/'shackle_learn/.cache/tmp'))
        with (OUT/(name+'_test.log')).open('x') as log:
            proc=sp.Popen(cmd,cwd=ROOT,env=env,stdout=log,stderr=sp.STDOUT)
            deadline=time.monotonic()+360
            meta_path=ROOT/f'runs/shackle_astra/{name}_runner.json'
            server_log=ROOT/f'runs/shackle_astra/{name}_server.log'
            while proc.poll() is None:
                if server_log.exists() and '"ready"' in server_log.read_text():
                    meta=json.loads(meta_path.read_text());pid=meta['server_pid']
                    record['shackle_server_pid']=pid
                    record['shackle_server_argv']=Path(f'/proc/{pid}/cmdline').read_bytes().decode().strip('\0').split('\0')
                    record['while_ready_guard']=guard()
                    assert pid not in record['while_ready_guard']['pids']
                    assert not any('astra_op/server.py' in x for x in record['shackle_server_argv'])
                    # Duplicate runner is rejected by its own held lock, never the A/B lock.
                    duplicate=sp.run(cmd,cwd=ROOT,env=env,capture_output=True,text=True)
                    record['concurrent_launch_refused']=duplicate.returncode!=0
                    record['concurrent_launch_stderr']=duplicate.stderr
                    break
                if time.monotonic()>deadline:raise RuntimeError('smoke readiness timeout')
                time.sleep(.1)
            record['returncode']=proc.wait()
        record['outcome']=json.loads((ROOT/f'runs/shackle_astra/{name}/outcome.json').read_text())
        ws=ROOT.parent/'astra_ws'/name
        assert (ws/'LESSONS.md').exists()==lessons
        if lessons:assert (ws/'LESSONS.md').read_bytes()==(ROOT/'astra_learn/lessons/v006.md').read_bytes()
        record['lessons_verified']=True
        assert record['returncode']==0,record
        assert record['outcome']['ended_by']=='done' and record['outcome']['final_claim']=='smoke'
        assert record['outcome']['memory_check']['rows']==0
        # Completed episode cannot be reused, even with an idle shackle runner.
        duplicate=sp.run(cmd,cwd=ROOT,env=env,capture_output=True,text=True)
        record['freshness_refused']=duplicate.returncode!=0 and 'fresh episode required' in duplicate.stderr
        assert record['freshness_refused']
        proof['episodes'].append(record)
        (OUT/'smoke_proof.json').write_text(json.dumps(proof,indent=2)+'\n')
    proof['after_guard']=guard();after=protected_all()
    proof['protected_changes']={p:dict(before=proof['protected_before'].get(p),after=after.get(p)) for p in set(proof['protected_before'])|set(after) if proof['protected_before'].get(p)!=after.get(p)}
    proof['protected_file_count']=len(after)
    current = snapshot()
    proof['source_asset_changes']=[p for p,h in json.loads((OUT/'protected_before.json').read_text()).items() if current.get(p)!=h]
    assert not proof['protected_changes'],proof['protected_changes']
    assert not proof['source_asset_changes']
    proof['finished_at']=dt.datetime.now(dt.timezone.utc).isoformat()
    (OUT/'smoke_proof.json').write_text(json.dumps(proof,indent=2)+'\n')
    print(json.dumps(dict(episodes=[dict(episode=r['episode'],pid=r['shackle_server_pid'],guard_pids=r['while_ready_guard']['pids'],outcome=r['outcome']) for r in proof['episodes']],protected_changes=proof['protected_changes'],source_asset_changes=proof['source_asset_changes']),indent=2))
