"""No-call replay audit: did the parked body physically help the apple/working arm?"""
import json
import sys
from collections import Counter
import mujoco
import sweep


def audit(names=('parked','parked_blind')):
    output=sweep.ROOT/'isolate_validation/parked_physical_contacts.json'
    results=json.loads(output.read_text())['episodes'] if output.exists() else {}
    original=mujoco.mj_step
    for name in names:
        results[name]=[]
        for path in sorted((sweep.ROOT/name/'apple').glob('ep_*/log.json')):
            log=json.loads(path.read_text())
            if not log.get('finished'):continue
            steps=0;counts=Counter();examples={}
            def monitored(model,data):
                nonlocal steps
                original(model,data)
                steps+=1
                types=set()
                for con in data.contact:
                    if con.dist>0:continue
                    bodies=[model.body(model.geom_bodyid[g]).name for g in (con.geom1,con.geom2)]
                    if not any(n.startswith('right_') for n in bodies):continue
                    other=[n for n in bodies if not n.startswith('right_')]
                    kind='right_self' if not other else ('left' if other[0].startswith('left_') else other[0] or 'world')
                    types.add(kind)
                    examples.setdefault(kind,bodies)
                counts.update(types)
            mujoco.mj_step=monitored
            env=None
            try:
                env,*_=sweep.initialize('apple',sweep.parse_config(name),log['seed'])
                for line in (path.parent/'chunks.jsonl').read_text().splitlines():
                    chunk=json.loads(line)
                    for l,r in zip(chunk['execution']['left'],chunk['execution']['right']):env.step(l,r)
                assert env.parked_max_joint_deviation==0
            finally:
                if env:env.close()
                mujoco.mj_step=original
            result=dict(seed=log['seed'],physics_steps=steps,right_contact_steps_by_other_body=dict(counts),examples=examples)
            results[name].append(result)
            print(name,log['seed'],dict(counts),flush=True)
    sweep.atomic(output,dict(inference_calls=0,note='Contacts with dist<=0 inspected after every 1ms mj_step, before projection; initialization and replayed execution included.',episodes=results))


if __name__=='__main__':audit(tuple(sys.argv[1:]) or ('parked','parked_blind'))
