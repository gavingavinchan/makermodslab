"""Authorized flange Dream-1 rerun; durable episodes, at most 400 billed calls.
The existing 2212 ledger cap is checked, never migrated or raised here.
"""
import argparse,fcntl,json
import sweep

OUT=sweep.HERE/'runs/flange_dream2'
PROMPTS={'T':'put the bolt through the hole in the flange and screw the nut onto it',
         'G1':'put the bolt through the hole in the flange and screw the nut onto it',
         'G2':'insert the bolt into the flange','G3':'screw the nut onto the bolt'}
START={'T':'table','G1':'pregrasp','G2':'pregrasp','G3':'inserted'}

class TaskLedger(sweep.Ledger):
    def __init__(self,start):super().__init__();self.start=start
    def reserve(self,episode,cap):
        if self.read()['budget_used']-self.start>=400:raise sweep.BudgetEnd()
        return super().reserve(episode,cap)

def run(live=False):
    OUT.mkdir(parents=True,exist_ok=True)
    with (sweep.ROOT/'runner.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        plan_path=OUT/'stages.json'
        if not live:
            for pid in ('T','G1','G2','G3'):
                sweep.episode('flange',dict(sweep.DEFAULT,start=START[pid]),0,False,None,
                    dry_root=OUT/'dry',prompt=PROMPTS[pid],variant=pid,stub=True)
            return
        state=sweep.Ledger().read();assert state['cap']==2212
        scripted=json.loads((OUT/'scripted/log.json').read_text())
        # Full Stage A fits the allowance; include the authorized G3/G2 fallback
        # even when the script has not passed, as well as the original T/G1.
        conditions=list(PROMPTS)
        plan=json.loads(plan_path.read_text()) if plan_path.exists() else dict(start_used=state['budget_used'],max_task_calls=400,
            scripted_success=scripted['success'],conditions=conditions,stage_a={},stage_b={})
        assert plan['start_used']==1626 and plan['max_task_calls']==400
        sweep.atomic(plan_path,plan);ledger=TaskLedger(plan['start_used'])
        def episode(rig,pid,i):
            c=dict(sweep.DEFAULT,start=START[pid],flange_trial='dream2',rig=rig)
            path=OUT/'episodes'/sweep.config_name(c)/'flange'/pid/f'ep_{i:02d}/log.json'
            prior=json.loads(path.read_text()) if path.exists() else {}
            needed=0 if prior.get('finished') else max(0,30-prior.get('calls_used',0))
            remaining=min(ledger.remaining(),400-(ledger.read()['budget_used']-plan['start_used']))
            if needed>remaining:
                plan.setdefault('skipped',[]).append(dict(rig=rig,condition=pid,episode=i,reason='full episode exceeds remaining cap'))
                sweep.atomic(plan_path,plan);return None
            return sweep.episode('flange',c,i,True,ledger,dry_root=OUT/'episodes',prompt=PROMPTS[pid],variant=pid,first_episode_index=2 if rig=='yam' else 0)
        def score(rows):
            return dict(episodes=len(rows),max_insertion_m=max((e['max_bolt_inserted_depth_m'] for e in rows),default=0),
                nut_engaged_eps=sum(e['nut_engaged'] for e in rows),max_turns=max((e['max_nut_turns'] for e in rows),default=0),calls=sum(e['calls_used'] for e in rows))
        for pid in plan['conditions']:
            rows=[e for i in range(2) if (e:=episode('viscous',pid,i)) is not None]
            plan['stage_a'][pid]=score(rows);sweep.atomic(plan_path,plan)
        winner=max(plan['conditions'],key=lambda p:(plan['stage_a'][p]['max_insertion_m'],plan['stage_a'][p]['nut_engaged_eps'],plan['stage_a'][p]['max_turns']))
        plan['winner']=winner;sweep.atomic(plan_path,plan)
        print('Stage A selected',winner,plan['stage_a'],flush=True)
        for rig in ('viscous','yam'):
            rows=[e for i in range(2,4) if (e:=episode(rig,winner,i)) is not None]
            plan['stage_b'][rig]=score(rows);sweep.atomic(plan_path,plan)
        d=ledger.read();plan.update(complete=True,billed_calls_added=d['budget_used']-plan['start_used'],ledger={k:d[k] for k in ('cap','budget_used','confirmed_billed','uncertain')})
        sweep.atomic(plan_path,plan);print(json.dumps(plan,indent=1),flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--live',action='store_true');run(p.parse_args().live)
