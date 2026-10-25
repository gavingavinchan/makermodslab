"""Record Gavin's explicit +300 authorization once; reserve no calls."""
import sys,fcntl,json,time
sys.path.insert(0,'.')
import sweep
root=sweep.ROOT
with (root/'ledger.lock').open('a') as lock:
    fcntl.flock(lock,fcntl.LOCK_EX)
    path=root/'ledger.json';d=json.loads(path.read_text())
    migrations=d.setdefault('migrations',[]);mid='flange_dream_20261024_300_calls'
    if not any(m.get('id')==mid for m in migrations):
        assert d['cap']==1912
        migrations.append(dict(id=mid,old_cap=1912,new_cap=2212,additional_calls=300,
            budget_used_at_authorization=d['budget_used'],utc=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),
            reason="Gavin 2026-10-24 BRIEF_FLANGE_DREAM: authorizes 300 additional billed calls; flange task at most 400 calls, Stage A <=240, Stage B <=120. Authorization recorded only; scripted feasibility failed, so Part 2 has not started and no calls have been reserved."))
        d['cap']=2212;sweep.atomic(path,d)
    assert d['cap']==2212
    result={k:d[k] for k in ('cap','budget_used','confirmed_billed','uncertain')};result.update(task_billed_calls=0,part2_started=False)
    sweep.atomic(sweep.HERE/'runs/flange_dream/validation/accounting.json',result)
    print(result)
