"""Read-only protected-file hashes; live A/B outputs can change under their owner."""
import hashlib,json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def snapshot():
 paths=[]
 for directory in ('astra_learn','astra_op','assets'):
  paths.extend(p for p in (ROOT/directory).rglob('*') if p.is_file() and '.cache' not in p.parts and '__pycache__' not in p.parts)
 paths.extend(p for p in ROOT.glob('*.py') if p.name not in ('shackle_scene.py','shackle_astra.py'))
 paths.extend(p for p in ROOT.rglob('ledger*') if p.is_file())
 paths.extend(p for p in ROOT.rglob('*runner.lock') if p.is_file())
 return {str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(set(paths))}
if __name__=='__main__':
 (ROOT/'runs/shackle_look/v2/protected_before.json').write_text(json.dumps(snapshot(),indent=2)+'\n')
