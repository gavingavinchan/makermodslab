"""Restore the before-demo accidentally overwritten by the force-only diagnostic.
Re-run the saved original source with its original 10 N physics and no welds.
"""
import sys,runpy,importlib.util
sys.path.insert(0,'.')
import sweep,flange_holding,flange_scene
flange_holding.configure=lambda root:None
old=sweep.initialize
sweep.initialize=lambda task,c,seed:old(task,dict(c,flange_attach=False),seed)
spec=importlib.util.spec_from_file_location('v2_before','runs/flange_dream/source_before/flange_scene.py');mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
flange_scene.Measurements=mod.Measurements
runpy.run_path('runs/flange_dream/source_before/flange_demo.py',run_name='__main__')
