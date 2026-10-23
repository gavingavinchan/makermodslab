"""Propeller stages using sweep.episode, its ledger, durable chunks, and replay."""
import argparse
import fcntl
import json
import time
import sweep

PROMPTS={
 'P1':'pick up the sponge and scrub the propeller',
 'P2':'clean the propeller with the sponge',
 'P3':'wipe the propeller',
 'P4':'scrub the propeller blade back and forth with the sponge',
}
OUT=sweep.HERE/'runs/propeller'
MIGRATION='propeller_20261023_600_calls'


def authorize(root=sweep.ROOT):
    """Exactly one cap change, under the same lock used by every reservation."""
    with (root/'ledger.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX)
        path=root/'ledger.json';d=json.loads(path.read_text())
        migrations=d.setdefault('migrations',[])
        if not any(m.get('id')==MIGRATION for m in migrations):
            assert d['cap']==1312
            migrations.append(dict(id=MIGRATION,old_cap=1312,new_cap=1912,additional_calls=600,
                budget_used_at_authorization=d['budget_used'],utc=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),
                reason="Gavin 2026-10-23: authorizes 600 more billed calls for zero-shot bench propeller cleaning; Stage A at most 240, Stage B at most 180."))
            d['cap']=1912;sweep.atomic(path,d)
        assert d['cap']==1912
    return sweep.Ledger(root)


def run(live=False):
    OUT.mkdir(parents=True,exist_ok=True)
    with (sweep.ROOT/'runner.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        if not live:
            for pid,prompt in PROMPTS.items():
                sweep.episode('propeller',dict(sweep.DEFAULT),0,False,sweep.Ledger(),dry_root=OUT/'dry',prompt=prompt,variant=pid,stub=True)
                print(pid,'dry episode complete; 30 local replies, zero calls',flush=True)
            return
        ledger=authorize();plan_path=OUT/'stages.json'
        plan=json.loads(plan_path.read_text()) if plan_path.exists() else dict(start_used=ledger.read()['budget_used'],max_task_calls=600,stage_a={},stage_b={})
        sweep.atomic(plan_path,plan)
        def episode(rig,pid,i):
            path=sweep.ROOT/('YAM' if rig=='yam' else 'B0')/'propeller'/pid/f'ep_{i:02d}/log.json'
            prior=json.loads(path.read_text()) if path.exists() else {}
            needed=0 if prior.get('finished') else max(0,30-prior.get('calls_used',0))
            remaining=min(ledger.remaining(),plan['max_task_calls']-(ledger.read()['budget_used']-plan['start_used']))
            if needed>remaining:
                plan.setdefault('skipped',[]).append(dict(rig=rig,prompt=pid,episode=i,reason='full episode exceeds remaining cap'))
                sweep.atomic(plan_path,plan);return None
            return sweep.episode('propeller',dict(sweep.DEFAULT,**({'rig':'yam'} if rig=='yam' else {})),i,True,ledger,prompt=PROMPTS[pid],variant=pid,first_episode_index=2 if rig=='yam' else 0)
        for pid in PROMPTS:
            rows=[]
            for i in range(2):
                result=episode('viscous',pid,i)
                if result:rows.append(result)
            plan['stage_a'][pid]=dict(episodes=len(rows),tool_contact_rows=sum(e['tool_propeller_contact_rows'] for e in rows),reversals=sum(e['scrub_reversals'] for e in rows))
            sweep.atomic(plan_path,plan)
        winner=max(PROMPTS,key=lambda p:(plan['stage_a'][p]['tool_contact_rows'],plan['stage_a'][p]['reversals']))
        plan['winner']=winner;sweep.atomic(plan_path,plan)
        print('Stage A selected',winner,plan['stage_a'],flush=True)
        for rig in ('viscous','yam'):
            rows=[e for i in range(2,5) if (e:=episode(rig,winner,i)) is not None]
            plan['stage_b'][rig]=dict(episodes=len(rows),tool_contact_rows=sum(e['tool_propeller_contact_rows'] for e in rows),reversals=sum(e['scrub_reversals'] for e in rows))
            sweep.atomic(plan_path,plan)
        d=ledger.read();plan.update(complete=True,billed_calls_added=d['budget_used']-plan['start_used'],ledger={k:d[k] for k in ('cap','budget_used','confirmed_billed','uncertain')})
        sweep.atomic(plan_path,plan)
        print(json.dumps(plan,indent=1),flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--live',action='store_true');run(p.parse_args().live)
