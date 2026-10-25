"""Unchanged propeller checks, disposable runner lock, accounting/network guards."""
import runpy,sweep
constructor=sweep.Ledger.__init__
sweep.ROOT=sweep.HERE/'runs/flange_dream2/validation/propeller_lock_fixture'
sweep.ROOT.mkdir(parents=True,exist_ok=True)
def initialize_ledger(self,root=None,cap=None):
    constructor(self,sweep.ROOT if root is None else root,cap)
sweep.Ledger.__init__=initialize_ledger
runpy.run_path('propeller_check.py',run_name='__main__')
