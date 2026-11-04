#!/usr/bin/env python3
"""Run separate benchmark batches serially, advancing only after a full audit."""
import argparse
import datetime as dt
import fcntl
import json
import os
from pathlib import Path
import shlex
import shutil
import signal
import subprocess
import sys
import time

from run_comparison_benchmark import PROJECTS, bag_filename, trial_bag_filename, save, sha


def timestamp():
    return dt.datetime.now().astimezone().isoformat()


def task_status(task, state, **extra):
    fields = (dict(scene='sparse_random', **task['scene_parameters'])
              if task.get('scene') == 'sparse_random' else dict(wall_length_m=task['wall_length']))
    save(Path(task['schedule_status']), dict(task=task['task'], state=state, **fields,
         runs_per_project=task['runs'],
         output=task['output'], updated_at=timestamp(), **extra))


def expected_configuration(task):
    expected = dict(task=task['task'], runs_per_project=task['runs'])
    if task.get('scene') == 'sparse_random':
        expected.update(scene='sparse_random', scene_parameters=task['scene_parameters'])
    else:
        expected['wall_length_m'] = task['wall_length']
    return expected


def validate_inputs(plan):
    for path, digest in plan['input_sha256'].items():
        if sha(Path(path)) != digest:
            raise RuntimeError('Scheduled experiment input changed: '+path)
    actual = subprocess.check_output(
        ['docker', 'image', 'inspect', '--format', '{{.Id}}', plan['image']], text=True).strip()
    if actual != plan['image_id']:
        raise RuntimeError('Scheduled Docker image changed')


def verify_completion(task):
    """Valid algorithm failures count; missing/corrupt/infra trials block advancement."""
    output = Path(task['output'])
    config = json.loads((output/'config.json').read_text())
    state = json.loads((output/'status.json').read_text())
    expected = expected_configuration(task)
    if any(config.get(k) != v for k, v in expected.items()):
        raise RuntimeError('Completed task configuration differs from schedule')
    if state.get('state') != 'complete' or state.get('completed_runs') != task['runs']*len(PROJECTS):
        raise RuntimeError('Previous task did not finish all requested trials')
    for project in PROJECTS:
        summary = state['summary'][project]
        if any(summary.get(k) != task['runs'] for k in ('requested_runs', 'completed_runs', 'valid_runs')) or summary.get('infrastructure_errors') != 0:
            raise RuntimeError('Previous task has incomplete or invalid trials: '+project)
        files = sorted((output/project).glob('run_*/metrics.json'))
        if len(files) != task['runs']:
            raise RuntimeError('Unexpected metrics count: '+project)
        for trial, path in enumerate(files, 1):
            result = json.loads(path.read_text())
            filename = trial_bag_filename(config, project, trial)
            bag = path.parent/filename
            if (path.parent.name != 'run_%03d' % trial or result.get('trial') != trial
                    or result.get('project') != project or result.get('bag_file') != filename
                    or not result.get('bag_valid') or result.get('reason') == 'infrastructure_error'
                    or Path(str(bag)+'.active').exists() or not bag.is_file()
                    or bag.stat().st_size != result.get('bag_bytes')
                    or sha(bag) != result.get('bag_sha256')):
                raise RuntimeError('Previous task bag audit failed: '+str(path))
    save(output/'completion_verification.json', dict(passed=True,
         verified_trials=task['runs']*len(PROJECTS), bag_sha256_verified=True,
         completed_at=timestamp()))


def benchmark_command(plan, task):
    command = [sys.executable, '-u', plan['runner'], '--task', str(task['task']),
               '--runs', str(task['runs']), '--image', plan['image'], '--output', task['output']]
    if task.get('scene') == 'sparse_random':
        command += ['--scene', 'sparse_random', '--geometry-reference', task['geometry_reference']]
        for key, value in task['scene_parameters'].items():
            command += ['--'+key.replace('_', '-'), str(value)]
    else:
        command += ['--wall-length', str(task['wall_length'])]
    if task.get('resume'):
        command.append('--resume')
    return command


def prepare_task(plan, task):
    if task.get('scene') != 'sparse_random':
        return
    if task.get('resume'):
        reference = json.loads(Path(task['geometry_reference']).read_text())
        if not reference.get('passed') or reference['scene_parameters'] != task['scene_parameters']:
            raise RuntimeError('Resume geometry reference differs from scheduled scene')
        return
    task_status(task, 'preparing_geometry')
    active = 'cmp-task%02d-geometry-%s' % (task['task'], Path(task['output']).name[-6:])
    command = ['docker', 'run', '--rm', '--network', 'none', '--name', active, '--user', '1000:1000']
    for project, (repo, _) in PROJECTS.items():
        command += ['--mount', 'type=bind,src=%s,dst=%s%s' % (
            repo, repo, ',readonly' if project != 'ours' else '')]
    arguments = ['/usr/bin/python3', plan['geometry_preparer'], '--output', task['geometry_directory']]
    for key, value in task['scene_parameters'].items():
        arguments += ['--'+key.replace('_', '-'), str(value)]
    shell = 'set -e\nsource /opt/ros/noetic/setup.bash\nexec '+shlex.join(arguments)
    command += ['--entrypoint', '/bin/bash', plan['image'], '-lc', shell]
    with Path(task['geometry_log']).open('x') as log:
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL)
        try:
            if process.wait(timeout=150):
                raise RuntimeError('Task geometry preflight failed: '+task['geometry_log'])
        finally:
            if process.poll() is None:
                subprocess.run(['docker', 'stop', '--time', '10', active], timeout=25, check=False)
                process.wait(timeout=15)
    reference = json.loads(Path(task['geometry_reference']).read_text())
    if not reference.get('passed') or reference['scene_parameters'] != task['scene_parameters']:
        raise RuntimeError('Geometry reference does not match scheduled scene')


def run_task(plan, task):
    output = Path(task['output'])
    if output.exists() and not task.get('resume'):
        raise RuntimeError('Refusing to overwrite existing batch: '+str(output))
    if task.get('resume'):
        previous = json.loads((output/'config.json').read_text())
        if any(previous.get(key) != value for key, value in expected_configuration(task).items()):
            raise RuntimeError('Resume task configuration differs from schedule')
    command = benchmark_command(plan, task)
    with Path(task['log']).open('x') as log:
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                   stdin=subprocess.DEVNULL)
        task_status(task, 'running', supervisor_pid=process.pid, command=command)
        copied = False
        startup_checked = False
        def check_startup():
            if not plan.get('startup_reference'):
                return False
            from verify_comparison_startup import verify_startup
            return verify_startup(output, Path(plan['startup_reference'])) is not None
        try:
            while process.poll() is None:
                if not copied and (output/'config.json').exists():
                    shutil.copyfile(task['protocol'], output/'PROTOCOL.md')
                    shutil.copyfile(plan['paper_excerpt'], output/'paper_equation45_excerpt.txt')
                    save(output/'scheduled_provenance.json', dict(
                        plan=plan['plan_path'], input_sha256=plan['input_sha256'],
                        image_id=plan['image_id']))
                    if task.get('scene') == 'sparse_random' and not task.get('resume'):
                        shutil.copytree(task['geometry_directory'], output/'geometry_preflight_details')
                    if plan.get('startup_reference'):
                        previous = json.loads((Path(plan['startup_reference'])/'config.json').read_text())
                        config = json.loads((output/'config.json').read_text())
                        for key in ('evaluation', 'binaries', 'paper_sha256', 'image', 'project_order'):
                            if config[key] != previous[key]:
                                raise RuntimeError('Protocol differs from reference: '+key)
                        save(output/'protocol_validation.json', dict(passed=True,
                             reference_task=previous['task'], task=task['task'],
                             evaluation_thresholds_identical=True, planning_binaries_identical=True,
                             paper_identical=True, image_identical=True,
                             three_project_random_geometry_identical=True))
                    copied = True
                if copied and not startup_checked:
                    startup_checked = check_startup()
                time.sleep(1)
            if process.returncode:
                raise RuntimeError('Task %d supervisor exited %d; see %s' % (
                    task['task'], process.returncode, task['log']))
            if not copied:
                raise RuntimeError('Task ended without configuration/protocol snapshot')
            if plan.get('startup_reference') and not startup_checked and not check_startup():
                raise RuntimeError('Task ended without three-project startup verification')
        finally:
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=60)
                except subprocess.TimeoutExpired:
                    # Stop only the container explicitly owned by this batch.
                    status = json.loads((output/'status.json').read_text())
                    active = status.get('active_container')
                    if active:
                        subprocess.run(['docker', 'stop', '--time', '15', active], timeout=35, check=False)
                    process.kill()
                    process.wait(timeout=10)


def wait_existing_task(plan, task):
    """Adopt an uninterrupted supervisor after replacing only the queue controller."""
    output = Path(task['output'])
    config = json.loads((output/'config.json').read_text())
    for key, value in expected_configuration(task).items():
        if config.get(key) != value:
            raise RuntimeError('Existing task configuration differs: '+key)
    pid = json.loads((output/'supervisor.json').read_text())['pid']
    task_status(task, 'running', supervisor_pid=pid, adopted_existing_supervisor=True)
    while True:
        state = json.loads((output/'status.json').read_text())
        if state.get('state') == 'complete':
            return
        if state.get('state') != 'running':
            raise RuntimeError('Existing task stopped: '+str(state))
        command_path = Path('/proc')/str(pid)/'cmdline'
        if not command_path.exists():
            raise RuntimeError('Existing supervisor exited before completion')
        command = command_path.read_bytes().split(b'\0')
        if str(output).encode() not in command or not any(
                item.endswith(b'/run_comparison_benchmark.py') for item in command):
            raise RuntimeError('Existing supervisor PID/command no longer matches')
        time.sleep(2)


def run_sequence(plan):
    current = -1
    try:
        for current, task in enumerate(plan['tasks']):
            validate_inputs(plan)
            save(Path(plan['status']), dict(state='running', active_task=task['task'],
                 pid=os.getpid(), updated_at=timestamp()))
            if task.get('adopt_existing'):
                wait_existing_task(plan, task)
            else:
                prepare_task(plan, task)
                run_task(plan, task)
            verify_completion(task)
            task_status(task, 'complete', completion_audit_passed=True)
        if plan.get('after_success_command'):
            subprocess.run(plan['after_success_command'], check=True)
        save(Path(plan['status']), dict(state='complete', finished_at=timestamp()))
        return 0
    except (Exception, KeyboardInterrupt) as exc:
        error = str(exc) or type(exc).__name__
        if current >= 0:
            task_status(plan['tasks'][current], 'stopped_error', error=error)
        for task in plan['tasks'][current+1:]:
            task_status(task, 'blocked_previous_task', error=error)
        save(Path(plan['status']), dict(state='stopped_error', error=error,
             active_task=plan['tasks'][current]['task'] if current >= 0 else None,
             updated_at=timestamp()))
        print(error, flush=True)
        return 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan', type=Path, required=True)
    args = parser.parse_args()
    plan = json.loads(args.plan.read_text())
    plan['plan_path'] = str(args.plan.resolve())
    def interrupt(*_):
        raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM, interrupt)
    with args.plan.with_suffix('.lock').open('w') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        return run_sequence(plan)


if __name__ == '__main__':
    sys.exit(main())
