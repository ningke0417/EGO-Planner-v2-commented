import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
import run_comparison_sequence as sequence


class SequenceTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.tasks = [dict(task=t, wall_length=w, runs=n, output=str(self.root/str(t)),
                          schedule_status=str(self.root/('%d_schedule.json' % t)))
                      for t, w, n in [(4, 9, 50), (5, 10, 50)]]
        self.plan = dict(tasks=self.tasks, status=str(self.root/'sequence.json'))

    def test_task_five_starts_only_after_task_four_completion_audit(self):
        events = []
        with patch.object(sequence, 'validate_inputs'), \
             patch.object(sequence, 'run_task', side_effect=lambda p, t: events.append(('run', t['task'], t['runs']))), \
             patch.object(sequence, 'verify_completion', side_effect=lambda t: events.append(('audit', t['task']))):
            self.assertEqual(sequence.run_sequence(self.plan), 0)
        self.assertEqual(events, [('run', 4, 50), ('audit', 4), ('run', 5, 50), ('audit', 5)])
        for task in self.tasks:
            self.assertEqual(json.loads(Path(task['schedule_status']).read_text())['state'], 'complete')
        self.assertNotEqual(self.tasks[0]['output'], self.tasks[1]['output'])

    def test_adopting_task_four_does_not_restart_it_and_task_five_uses_fifty(self):
        self.tasks[0]['adopt_existing'] = True
        events = []
        with patch.object(sequence, 'validate_inputs'), \
             patch.object(sequence, 'wait_existing_task', side_effect=lambda p, t: events.append(('adopt', t['task']))), \
             patch.object(sequence, 'run_task', side_effect=lambda p, t: events.append(('run', t['task'], t['runs']))), \
             patch.object(sequence, 'verify_completion', side_effect=lambda t: events.append(('audit', t['task']))):
            self.assertEqual(sequence.run_sequence(self.plan), 0)
        self.assertEqual(events, [('adopt', 4), ('audit', 4), ('run', 5, 50), ('audit', 5)])

    def test_incomplete_or_invalid_task_four_blocks_task_five(self):
        for failure_at in ('run_task', 'verify_completion'):
            with self.subTest(failure_at=failure_at), patch.object(sequence, 'validate_inputs'), \
                 patch.object(sequence, 'run_task') as run, patch.object(sequence, 'verify_completion') as audit:
                {'run_task': run, 'verify_completion': audit}[failure_at].side_effect = RuntimeError('incomplete')
                self.assertEqual(sequence.run_sequence(self.plan), 1)
                self.assertEqual(run.call_count, 1)
                self.assertEqual(json.loads(Path(self.tasks[1]['schedule_status']).read_text())['state'], 'blocked_previous_task')

    def test_valid_algorithm_failure_counts_but_corrupt_bag_blocks_completion(self):
        task = dict(self.tasks[0], runs=1)
        root = Path(task['output']); root.mkdir()
        sequence.save(root/'config.json', dict(task=4, wall_length_m=9, runs_per_project=1))
        summary = {}
        for project in sequence.PROJECTS:
            run = root/project/'run_001'; run.mkdir(parents=True)
            filename = sequence.bag_filename(4, project, 9, 1)
            bag = run/filename; bag.write_bytes(b'test finalized bag')
            sequence.save(run/'metrics.json', dict(project=project, trial=1, bag_file=filename,
                          bag_valid=True, reason='flight_timeout', arrival_success=False,
                          bag_bytes=bag.stat().st_size, bag_sha256=sequence.sha(bag)))
            summary[project] = dict(requested_runs=1, completed_runs=1, valid_runs=1, infrastructure_errors=0)
        sequence.save(root/'status.json', dict(state='complete', completed_runs=3, summary=summary))
        sequence.verify_completion(task)
        bag.write_bytes(b'corrupt')
        with self.assertRaisesRegex(RuntimeError, 'bag audit failed'):
            sequence.verify_completion(task)

    def random_tasks(self):
        return [dict(task=task, scene='sparse_random', runs=20,
                     scene_parameters=dict(random_seed=17, small_obstacle_count=8, small_position_seed=task-5),
                     output=str(self.root/('task%02d' % task)),
                     schedule_status=str(self.root/('task%02d_schedule.json' % task)),
                     geometry_reference=str(self.root/('geometry%02d.json' % task)))
                for task in range(7, 16)]

    def test_nine_random_tasks_are_prepared_run_and_audited_strictly_in_order(self):
        tasks = self.random_tasks()
        plan = dict(self.plan, tasks=tasks)
        events = []
        with patch.object(sequence, 'validate_inputs'), \
             patch.object(sequence, 'prepare_task', side_effect=lambda p, t: events.append(('prepare', t['task']))), \
             patch.object(sequence, 'run_task', side_effect=lambda p, t: events.append(('run', t['task']))), \
             patch.object(sequence, 'verify_completion', side_effect=lambda t: events.append(('audit', t['task']))):
            self.assertEqual(sequence.run_sequence(plan), 0)
        self.assertEqual(events, [(action, task) for task in range(7, 16) for action in ('prepare', 'run', 'audit')])
        self.assertEqual(len({task['output'] for task in tasks}), 9)
        self.assertEqual([task['scene_parameters']['small_position_seed'] for task in tasks], list(range(2, 11)))
        self.assertEqual(sum(task['runs']*3 for task in tasks), 540)

    def test_random_command_and_schedule_preserve_seed_and_count(self):
        task = self.random_tasks()[0]
        command = sequence.benchmark_command(dict(runner='runner.py', image='ros'), task)
        for flag, value in [('--task', '7'), ('--runs', '20'), ('--scene', 'sparse_random'),
                            ('--random-seed', '17'), ('--small-obstacle-count', '8'), ('--small-position-seed', '2')]:
            self.assertEqual(command[command.index(flag)+1], value)
        self.assertNotIn('--wall-length', command)
        sequence.task_status(task, 'queued')
        status = json.loads(Path(task['schedule_status']).read_text())
        self.assertEqual(status['small_position_seed'], 2)
        self.assertNotIn('wall_length_m', status)

    def test_resume_uses_original_batch_and_existing_geometry(self):
        task = dict(self.random_tasks()[0], resume=True)
        reference = Path(task['geometry_reference'])
        sequence.save(reference, dict(passed=True, scene_parameters=task['scene_parameters']))
        command = sequence.benchmark_command(dict(runner='runner.py', image='ros'), task)
        self.assertIn('--resume', command)
        self.assertEqual(command[command.index('--output')+1], task['output'])
        with patch.object(sequence.subprocess, 'Popen') as start:
            sequence.prepare_task({}, task)
        start.assert_not_called()
        sequence.save(reference, dict(passed=True, scene_parameters=dict(task['scene_parameters'],small_position_seed=99)))
        with self.assertRaisesRegex(RuntimeError, 'Resume geometry reference differs'):
            sequence.prepare_task({}, task)

    def test_final_aggregation_runs_only_after_all_tasks_pass_completion_audit(self):
        command = ['python3', 'aggregate.py']
        plan = dict(self.plan, after_success_command=command)
        events = []
        with patch.object(sequence, 'validate_inputs'), patch.object(sequence, 'run_task'), \
             patch.object(sequence, 'verify_completion', side_effect=lambda task: events.append(task['task'])), \
             patch.object(sequence.subprocess, 'run', side_effect=lambda *args, **kwargs: events.append('aggregate')) as aggregate:
            self.assertEqual(sequence.run_sequence(plan), 0)
        self.assertEqual(events, [4, 5, 'aggregate'])
        aggregate.assert_called_once_with(command, check=True)
        with patch.object(sequence, 'validate_inputs'), patch.object(sequence, 'run_task'), \
             patch.object(sequence, 'verify_completion', side_effect=RuntimeError('audit failed')), \
             patch.object(sequence.subprocess, 'run') as aggregate:
            self.assertEqual(sequence.run_sequence(plan), 1)
        aggregate.assert_not_called()

    def test_random_preflight_failure_prevents_current_and_subsequent_recordings(self):
        tasks = self.random_tasks()
        with patch.object(sequence, 'validate_inputs'), \
             patch.object(sequence, 'prepare_task', side_effect=RuntimeError('geometry differs')), \
             patch.object(sequence, 'run_task') as run:
            self.assertEqual(sequence.run_sequence(dict(self.plan, tasks=tasks)), 1)
        run.assert_not_called()
        self.assertEqual(json.loads(Path(tasks[0]['schedule_status']).read_text())['state'], 'stopped_error')
        for task in tasks[1:]:
            self.assertEqual(json.loads(Path(task['schedule_status']).read_text())['state'], 'blocked_previous_task')

    def test_random_completion_rejects_wrong_seed_and_checks_descriptive_bag_names(self):
        task = dict(self.random_tasks()[0], runs=1)
        root = Path(task['output']); root.mkdir()
        config = sequence.expected_configuration(task)
        sequence.save(root/'config.json', config)
        summary = {}
        for project in sequence.PROJECTS:
            run = root/project/'run_001'; run.mkdir(parents=True)
            filename = sequence.trial_bag_filename(config, project, 1)
            bag = run/filename; bag.write_bytes(b'finalized random trial')
            sequence.save(run/'metrics.json', dict(project=project, trial=1, bag_file=filename,
                bag_valid=True, reason='flight_timeout', bag_bytes=bag.stat().st_size, bag_sha256=sequence.sha(bag)))
            summary[project] = dict(requested_runs=1, completed_runs=1, valid_runs=1, infrastructure_errors=0)
        sequence.save(root/'status.json', dict(state='complete', completed_runs=3, summary=summary))
        sequence.verify_completion(task)
        config['scene_parameters'] = dict(config['scene_parameters'], small_position_seed=3)
        sequence.save(root/'config.json', config)
        with self.assertRaisesRegex(RuntimeError, 'configuration differs'):
            sequence.verify_completion(task)


if __name__ == '__main__':
    unittest.main()
