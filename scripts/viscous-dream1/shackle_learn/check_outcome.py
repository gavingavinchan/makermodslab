"""Small host-only fixtures for scoring success, drops and claims."""
import json
from pathlib import Path
from outcome import ROOT, score
out=ROOT/'runs/shackle_look/v2/outcome_fixture';out.mkdir(exist_ok=True)
def run(rows):
 (out/'episode.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
 return score(out)
def row(t,z,**more):
 return dict(kind='physics',time=t,pin_xyz=[0,0,z],seated=False,inserted=False,engaged=False,turns=0.,gap_to_seated_m=.01,**more)
a=row(0.,1.);a.update(seated=True,turns=.2,inserted=True,engaged=True,gap_to_seated_m=0.)
assert run([a])['success'] # Seating defines success, not an arbitrary turn threshold.
a.update(seated=False,turns=20.)
assert not run([a])['success']
r=[row(0.,1.),row(.1,.989),row(.2,.97),row(.3,.97,grasp_attached={'left':'pin'}),row(.4,.96),row(.5,.94)]
result=run(r);assert result['pin_drops']==2,result
claim=dict(r[-1],kind='command_end',command_number=1,command='done seated',response={'claim':'seated'})
result=run(r+[claim]);assert 'does not show' in result['claim_vs_truth']
result=run([dict(claim,response={'claim':'not seated'},command='done not seated')]);assert 'No disagreement' in result['claim_vs_truth']
print('PASS: seating-only success, continuous drop/re-support, positive/negative claim checks')
