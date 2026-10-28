#!/usr/bin/env python3
"""No-inference checks for scoring and A/B control flow; uses disposable fixtures."""
import contextlib
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import ab
import outcome
import runner
from consolidate import validate


class Checks(unittest.TestCase):
    def test_drop_support_and_rearm(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / 'fixture'; out.mkdir()
            rows = []
            # Held downward motion, then two independent falls, then threaded release.
            for i, (z, held, engaged) in enumerate([(1, True, False), (.8, True, False),
                    (.8, False, False), (.795, False, False), (.77, False, False),
                    (.75, False, False), (.9, True, False), (.9, False, False),
                    (.88, False, False), (.9, False, True), (.7, False, True)]):
                rows.append(dict(kind='physics', time=i*.05, nut_xyz=[0, 0, z],
                    nut_engaged=engaged, grasp_attached={'right': 'nut'} if held else {},
                    nut_turns=2.1 if engaged else 0, nut_seated=False,
                    nut_face_gap_m=.002, command='screw right 720', left_holds_bolt=i%2==0))
            rows += [dict(kind='command_end', command_number=1, time=.5, nut_turns=2.1),
                     dict(kind='end', reason='operator done', time=.5, nut_turns=2.1,
                          nut_seated=False, nut_face_gap_m=.002, command='done Nut seated')]
            (out/'episode.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
            o = outcome.score(out)
            self.assertEqual(o['nut_drops'], 2)
            self.assertTrue(o['success']); self.assertEqual(o['commands'], 1)
            self.assertAlmostEqual(o['left_holds_bolt'], .5)
            self.assertEqual(o['claim_vs_truth'], 'claimed seated; sim 2.0 mm short.')
            Path(str(out)+'_runner.json').write_text(json.dumps({'ended_by':'wall_cap', 'host_cleanup_claim':'Nut seated'}))
            o = outcome.score(out)
            self.assertEqual(o['ended_by'], 'wall_cap'); self.assertIsNone(o['final_claim'])

    def test_missing_and_truncated_log(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp)/'fixture';out.mkdir()
            (out/'episode.jsonl').write_text('{"kind":"physics","time":1,"nut_xyz":[0,0,1],"nut_turns":0}\n{"kind":')
            o=outcome.score(out)
            self.assertEqual(o['ended_by'], 'crash'); self.assertIsNone(o['left_holds_bolt'])
            self.assertEqual(o['malformed_log_lines'], 1)

    def test_memory_contamination_refused(self):
        import sqlite3
        with tempfile.TemporaryDirectory() as tmp:
            home=Path(tmp); (home/'.codex').mkdir()
            con=sqlite3.connect(home/'.codex/memories_1.sqlite')
            con.execute('CREATE TABLE stage1_outputs (text TEXT)');con.commit()
            with patch.object(runner.Path, 'home', return_value=home):
                self.assertEqual(runner.memory_check()['rows'], 0)
                con.execute("INSERT INTO stage1_outputs VALUES ('contamination')");con.commit()
                with self.assertRaisesRegex(RuntimeError, 'contamination'): runner.memory_check()
            con.close()

    def test_export_completion_detection(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp)
            log = out/'episode.jsonl'
            log.write_text(json.dumps({'kind':'command_end','command':'status'})+'\n')
            self.assertFalse(runner.simulator_finished(out))
            with log.open('a') as f:
                f.write(json.dumps({'kind':'command_end','command':'done smoke'})+'\n')
            self.assertTrue(runner.simulator_finished(out))
            log.write_text(json.dumps({'kind':'end','reason':'command cap'})+'\n')
            self.assertTrue(runner.simulator_finished(out))

    def test_lesson_lint(self):
        self.assertTrue(validate('General\nROV\nTabletop\nposition=(0.1, -0.2, 0.3)'))
        self.assertTrue(validate('General\nROV\nTabletop\nx=0.1'))
        self.assertTrue(validate('General\nROV\nTabletop\njoint 2 = -1.2'))
        self.assertTrue(validate('General\nROV\nTabletop\n'+'line\n'*100))
        self.assertEqual(validate('General\nROV two-hand\nTabletop\nUse 1 mm relative nudges; turn +20 degrees.'), [])

    def test_ab_continues_after_crash(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);runs=root/'runs';runs.mkdir()
            (root/'astra_learn/lessons').mkdir(parents=True)
            (root/'astra_learn/lessons/v001.md').write_text('fixed')
            calls=[]
            def fake_run(cmd, **kwargs):
                name=cmd[cmd.index('--episode')+1];calls.append(name)
                if name=='astra_ab_N1': return type('Result', (), {'returncode':1})()
                out=runs/name;out.mkdir()
                o=dict(episode=name,success=True,nut_seated=True,commands=10,
                    sim_s=20,wall_s=30,nut_drops=0,ended_by='done',claim_vs_truth=None)
                (out/'outcome.json').write_text(json.dumps(o))
                return type('Result', (), {'returncode':0})()
            with patch.object(ab,'ROOT',root), patch.object(ab,'RUNS',runs), patch.object(ab.sp,'run',side_effect=fake_run), patch('sys.argv',['ab.py']):
                ab.main()
            self.assertEqual(calls,[n for n,a in ab.PLAN])
            records=json.loads((runs/'ab/outcomes.json').read_text())
            self.assertEqual(records[1]['ended_by'],'crash')
            self.assertEqual((runs/'ab/ab.log').read_text().count('END '),6)
            self.assertIn('2/3',(runs/'ab/summary.md').read_text())


if __name__ == '__main__': unittest.main(verbosity=2)
