"""Names for benchmark artifacts, with compatibility for earlier batches."""
import json
from pathlib import Path

DEFAULT_NAMES = dict(bag='flight.bag', odometry='odometry.csv', result='result.json',
                     formation='formation_error.json', metadata='metadata.json',
                     parameters='parameters.yaml', live='live_status.json',
                     console='scene.log', recorder='recorder.log')


def initialize(directory, prefix, trial):
    names = dict(DEFAULT_NAMES)
    if prefix:
        stem = '%s_run%02d' % (prefix, trial)
        names = dict(bag=stem + '.bag', odometry=stem + '_positions.csv',
                     result=stem + '_metrics.json', formation=stem + '_formation_error_eq45.json',
                     metadata=stem + '_scene_config.json', parameters=stem + '_ros_parameters.yaml',
                     live=stem + '_live_status.json', console=stem + '_planner.log',
                     recorder=stem + '_recorder.log')
    (directory / 'files.json').write_text(json.dumps(names, indent=2) + '\n')
    return names


def artifact_path(directory, role):
    directory = Path(directory)
    manifest = directory / 'files.json'
    names = json.loads(manifest.read_text()) if manifest.exists() else DEFAULT_NAMES
    name = names[role]
    if Path(name).name != name:
        raise ValueError('Artifact must be a filename inside the trial directory')
    return directory / name
