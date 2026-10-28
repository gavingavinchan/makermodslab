"""Host-only final workspace inventory plus the A/B's heuristic Codex tool audit."""
import hashlib,json,subprocess,sys
from pathlib import Path
from outcome import session_for


def inventory(ws):
    files = {}
    if ws.exists():
        for path in sorted(ws.rglob('*')):
            rel = str(path.relative_to(ws))
            if path.is_symlink():
                files[rel] = {'symlink': str(path.readlink())}
            elif path.is_file():
                files[rel] = {'bytes': path.stat().st_size, 'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}
            elif path.is_dir():
                files[rel] = {'directory': True}
    return files


def write_audit(out, ws, meta, operator_cmd):
    session = session_for(Path(str(out) + '_astra.log'))
    final = inventory(ws)
    record = dict(workspace=str(ws), initial=meta.get('workspace_initial_inventory'), final=final,
                  lessons_expected=bool(meta.get('lessons')), lessons_present=(ws/'LESSONS.md').is_file(),
                  test_command=operator_cmd, session=str(session) if session else None)
    (out/'workspace_audit.json').write_text(json.dumps(record, indent=2)+'\n')
    with (out/'audit.txt').open('w') as dest:
        if session:
            subprocess.run([sys.executable, str(Path(__file__).with_name('audit_session.py')), str(session)], stdout=dest, stderr=subprocess.STDOUT, check=True)
            dest.write(f'\nsession {session}\n')
        else:
            dest.write('No Codex session (test operator or launch failure).\n')
        dest.write('Test command: '+str(operator_cmd)+'\n')
        dest.write('Initial workspace: '+json.dumps(meta.get('workspace_initial_files'))+'\n')
        dest.write('Workspace inventory and hashes: workspace_audit.json\n')
        dest.write('This inventory/tool audit is heuristic, not an OS sandbox.\n')
