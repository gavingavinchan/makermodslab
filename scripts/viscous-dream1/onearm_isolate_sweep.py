"""Paired apple isolation; existing global cap unchanged, additional hard limit 220."""
import argparse
import fcntl
import json
import sweep

START_USED = 1029
ALLOWANCE = 220
NAMES = ('single_fixedcam', 'parked_blind')
PLAN = sweep.ROOT / 'isolate_stages.json'


class IsolateLedger(sweep.Ledger):
    def update(self, fn):
        def bounded(data):
            result = fn(data)
            used = sum(r['status'] != 'pool_warming' for r in data['requests'])
            if used > START_USED + ALLOWANCE:
                raise sweep.BudgetEnd('Isolation task 220-call limit')
            return result
        return super().update(bounded)


def run(live):
    if not live:
        raise ValueError('Use onearm_isolate_check.py for no-call checks; --live for inference')
    with (sweep.ROOT / 'runner.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        ledger = IsolateLedger()
        assert ledger.cap == 1312
        state = ledger.read()
        plan = json.loads(PLAN.read_text()) if PLAN.exists() else dict(start_used=START_USED, allowance=ALLOWANCE, cap=ledger.cap)
        if 'episodes_per_config' not in plan:
            remaining = min(ledger.remaining(), START_USED + ALLOWANCE - state['budget_used'])
            plan['episodes_per_config'] = min(6, remaining // (len(NAMES) * sweep.CAPS['apple']))
            sweep.atomic(PLAN, plan)
        for name in NAMES:
            sweep.run_config(sweep.parse_config(name), ['apple'], plan['episodes_per_config'], True, ledger)
        state = ledger.read()
        plan.update(complete=True, billed_calls_added=state['confirmed_billed']-START_USED,
                    budget_used_added=state['budget_used']-START_USED,
                    ledger={k:v for k,v in state.items() if k != 'requests'})
        sweep.atomic(PLAN, plan)
        print(json.dumps(plan, indent=2), flush=True)


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--live', action='store_true')
    run(p.parse_args().live)
