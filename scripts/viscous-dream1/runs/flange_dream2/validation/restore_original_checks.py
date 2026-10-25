"""Restore trial-1 check artifacts using its saved modules and 5 Nm limits."""
import sys,runpy,importlib.abc,importlib.util
from pathlib import Path
root=Path.cwd();snapshot=root/'runs/flange_dream2/source_before'
sys.path.insert(0,str(root))
class SavedLoader(importlib.abc.Loader):
    def __init__(self,name):self.name=name
    def create_module(self,spec):return None
    def exec_module(self,module):
        module.__file__=str(root/(self.name+'.py'))
        exec(compile((snapshot/(self.name+'.py')).read_text(),module.__file__,'exec'),module.__dict__)
class SavedFinder(importlib.abc.MetaPathFinder):
    def find_spec(self,fullname,path=None,target=None):
        if fullname in ('sweep','sweep_scene','flange_scene','flange_holding','flange_start'):
            return importlib.util.spec_from_loader(fullname,SavedLoader(fullname))
sys.meta_path.insert(0,SavedFinder())
import sweep
e=sweep.SweepEnv('flange',sweep.DEFAULT)
assert all((e.model.actuator_forcerange[e.aids[s]]==[-5,5]).all() for s in e.sides)
e.close()
for script in ('flange_holding_check.py','flange_pregrasp_check.py'):
    sys.argv=['safe_check.py',script]
    runpy.run_path('runs/flange_v2/validation/safe_check.py',run_name='__main__')
print('Restored original trial-1 holding/stub artifacts with original 5 Nm modules; no source edits or inference.',flush=True)
