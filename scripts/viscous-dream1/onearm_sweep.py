"""Budgeted, resumable apple-only one-arm follow-up. No calls without --live."""
import argparse
import fcntl
import json
import sweep

PLAN = sweep.ROOT / 'onearm_stages.json'
START_USED = 872
ALLOWANCE = 440


def extend_ledger():
    """One-time authorized cap change, under the existing accounting lock."""
    ledger = sweep.Ledger()
    with ledger.lock_path.open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        data = json.loads(ledger.path.read_text())
        if 'onearm_allowance' not in data:
            assert data['budget_used'] == data['confirmed_billed'] == START_USED and data['uncertain'] == 0
            assert data['cap'] == 900
            data['onearm_allowance'] = dict(start_used=START_USED, additional_calls=ALLOWANCE, previous_cap=900)
            data['cap'] = START_USED + ALLOWANCE
            sweep.atomic(ledger.path, data)
        assert data['cap'] == START_USED + ALLOWANCE
        assert data['onearm_allowance']['additional_calls'] == ALLOWANCE
    return sweep.Ledger()


def run(live):
    if not live:
        raise ValueError('Use onearm_check.py for zero-inference validation, or --live for the authorized sweep')
    with (sweep.ROOT / 'runner.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        ledger = extend_ledger()
        plan = json.loads(PLAN.read_text()) if PLAN.exists() else dict(start_used=START_USED, cap=START_USED+ALLOWANCE)
        def save(): sweep.atomic(PLAN, plan)
        def stage(names, key):
            if key + '_episodes' not in plan:
                # All requested fresh episodes fit at full caps; keep pairing if shortened.
                plan[key + '_episodes'] = min(6, ledger.remaining() // (len(names) * sweep.CAPS['apple']))
                save()
            for name in names:
                sweep.run_config(sweep.parse_config(name), ['apple'], plan[key + '_episodes'], True, ledger)
            plan[key + '_scores'] = {name: sweep.score(sweep.parse_config(name), ['apple']) for name in names}
            save()
        stage(['single_copy', 'parked'], 'stage1')
        winner = max(['single_copy', 'parked'], key=lambda name: sweep.score(sweep.parse_config(name), ['apple']))
        plan['stage1_winner'] = winner
        if max(v[0] for v in plan['stage1_scores'].values()) < 5:
            plan['stage2_required'] = True
            save()
            # The layout is validated with geometry and camera checks before use.
            from onearm_layout import check_layout
            check_layout()
            stage([winner + '_left', 'single_left'], 'stage2')
        else:
            plan['stage2_required'] = False
        plan['complete'] = True
        state = ledger.read()
        plan['billed_calls_added'] = state['confirmed_billed'] - START_USED
        plan['budget_used_added'] = state['budget_used'] - START_USED
        plan['ledger'] = {k: v for k, v in state.items() if k != 'requests'}
        save()
        print(json.dumps(plan, indent=2), flush=True)

if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--live', action='store_true')
    run(p.parse_args().live)
