#!/usr/bin/env python3
"""Compute Eqs. (45)/(46), Quan et al., T-RO 2023, from ROS1 bags.

Requires numpy, PyYAML and (for bag reading only) the sourced ROS1 environment.
The paper averages over path length, not time. Here the formation path is the
3D centroid trajectory; this convention is explicit because the paper does not
uniquely specify how to extract a formation path from individual odometry.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np
import yaml


def save_json(path, data):
    path.write_text(json.dumps(data, indent=2, ensure_ascii=False, allow_nan=False)+'\n')


def recording_bag_path(run, result=None):
    if result is None:
        result = json.loads((run/'result.json').read_text())
    name = result.get('bag_file', 'flight.bag')
    if Path(name).name != name:
        raise ValueError('Bag filename must be a basename within its trial directory')
    return run/name


def similarity_residual(actual, desired):
    """Global minimum of sum_i ||desired_i-(s R actual_i+t)||^2.

    Batched Umeyama/orthogonal Procrustes solution, positive scale, R in SO(3).
    Points are stored as rows, so the returned fit uses the transpose rotation.
    No division by N and no square root: the printed Eq. (45) uses a squared sum.
    """
    x = np.asarray(actual, dtype=float)
    y = np.broadcast_to(np.asarray(desired, dtype=float), x.shape)
    xc = x-x.mean(axis=-2, keepdims=True)
    yc = y-y.mean(axis=-2, keepdims=True)
    var = np.sum(xc*xc, axis=(-2, -1))
    if np.any(var <= 1e-14):
        raise ValueError('Collapsed formation: Sim(3) scale is undefined')
    h = np.swapaxes(xc, -1, -2) @ yc
    u, singular, vt = np.linalg.svd(h)
    signs = np.ones_like(singular)
    signs[..., -1] = np.where(np.linalg.det(u @ vt) < 0, -1.0, 1.0)
    rotation = (u*signs[..., None, :]) @ vt
    scale = np.sum(singular*signs, axis=-1)/var
    if np.any(scale <= 0):
        raise ValueError('No strictly positive optimal alignment scale')
    residual = scale[..., None, None]*(xc @ rotation)-yc
    return np.sum(residual*residual, axis=(-2, -1)), scale


def normalized_laplacian(points):
    points = np.asarray(points, dtype=float)
    delta = points[..., :, None, :]-points[..., None, :, :]
    adjacency = np.sum(delta*delta, axis=-1)
    degree = np.sum(adjacency, axis=-1)
    if np.any(degree <= 1e-14):
        raise ValueError('Zero-degree formation graph')
    return np.eye(points.shape[-2])-adjacency/np.sqrt(degree[..., :, None]*degree[..., None, :])


def weighted_means(t, points, values, s0):
    dl = np.linalg.norm(np.diff(points.mean(axis=1), axis=0), axis=1)
    length = float(dl.sum())
    duration = float(t[-1]-t[0])
    if length <= 1e-10 or duration <= 0:
        raise ValueError('Nonpositive path length or duration')
    midpoint_values = (values[:-1]+values[1:])*0.5
    return (float(np.sum(midpoint_values*dl)/(s0*length)),
            float(np.sum(midpoint_values*np.diff(t))/duration), length)


def sample_formation(streams, start, end, hz):
    # Relative timestamps avoid taking differences of large epoch floats later.
    duration = end-start
    grid = np.arange(int(np.floor(duration*hz))+1, dtype=float)/hz
    if duration-grid[-1] > 1e-8:
        grid = np.append(grid, duration)
    else:
        grid[-1] = duration
    sampled, max_gap = [], 0.0
    for stream in streams:
        data = np.asarray(stream, dtype=float)
        if data.ndim != 2 or len(data) < 2 or not np.all(np.isfinite(data)):
            raise ValueError('Missing/nonfinite odometry')
        times = data[:, 0]-start
        if np.any(np.diff(times) < 0):
            raise ValueError('Odometry timestamp moved backwards')
        # Repeated timestamps retain the last value, with no extrapolation.
        keep = np.r_[np.diff(times) > 0, True]
        times, xyz = times[keep], data[keep, 1:]
        if times[0] > 0 or times[-1] < duration:
            raise ValueError('Odometry does not bracket the complete flight')
        lo = max(0, np.searchsorted(times, 0, side='right')-1)
        hi = min(len(times)-1, np.searchsorted(times, duration))
        max_gap = max(max_gap, float(np.diff(times[lo:hi+1]).max()))
        sampled.append(np.column_stack([np.interp(grid, times, xyz[:, j]) for j in range(3)]))
    if max_gap > 0.1:
        raise ValueError('Odometry gap exceeds 0.1 s: '+str(max_gap))
    return grid, np.stack(sampled, axis=1), max_gap


def evaluate(t, points, desired, s0):
    residual, scales = similarity_residual(points, desired)
    lap_delta = normalized_laplacian(points)-normalized_laplacian(desired)
    graph = np.sum(lap_delta*lap_delta, axis=(-2, -1))
    distance_mean, distance_time_mean, length = weighted_means(t, points, residual, s0)
    graph_mean, graph_time_mean, _ = weighted_means(t, points, graph, s0)
    return dict(e_dist_eq45=distance_mean, e_sim_eq46=graph_mean,
                residual_time_mean_m2=distance_time_mean,
                graph_time_mean=graph_time_mean, centroid_path_length_m=length,
                duration_s=float(t[-1]), residual_max_m2=float(residual.max()),
                alignment_scale_min=float(scales.min()), alignment_scale_max=float(scales.max())), residual, graph, scales


def load_bag(run, include_failures=False):
    import rosbag
    result = json.loads((run/'result.json').read_text())
    if not result['success'] and not include_failures:
        raise ValueError('Trial was not successful: '+str(run))
    params = yaml.safe_load((run/'params.yaml').read_text())
    formation = params['drone_0_ego_planner_node']['formation']
    count = formation['num']
    desired = np.asarray([formation['drone'+str(i)] for i in range(count)], dtype=float)
    for i in range(count):
        other = params['drone_%d_ego_planner_node' % i]['formation']
        if not np.allclose(desired, [other['drone'+str(j)] for j in range(count)]):
            raise ValueError('Inconsistent desired slots across planners')
    odom_topics = {'/drone_%d_visual_slam/odom' % i: i for i in range(count)}
    applied_topics = {'/drone_%d/formation_applied' % i: i for i in range(count)}
    streams = [[] for _ in range(count)]
    ack_counts = [0]*count
    applied = [[] for _ in range(count)]
    with rosbag.Bag(str(recording_bag_path(run, result))) as bag:
        for topic, msg, bag_stamp in bag.read_messages(topics=list(odom_topics)+list(applied_topics)):
            if topic in odom_topics:
                p = msg.pose.pose.position
                streams[odom_topics[topic]].append((msg.header.stamp.to_sec(), p.x, p.y, p.z))
            else:
                i = applied_topics[topic]
                ack_counts[i] += 1
                slots = list(msg.drone_to_slot)
                if sorted(slots) != list(range(count)):
                    raise ValueError('Invalid applied slot permutation')
                # The local planner is authoritative for its own physical ID.
                # Commits propagate asynchronously; do not infer peer application
                # from another drone's full assignment vector.
                stamp = msg.header.stamp.to_sec() or bag_stamp.to_sec()
                applied[i].append((stamp, slots[i]))
    if not all(ack_counts):
        raise ValueError('Missing applied-assignment evidence')
    return streams, desired, result, ack_counts, applied


def applied_reference(slots, history, absolute_times):
    reference = np.empty((len(absolute_times), len(slots), 3))
    mappings = np.empty((len(absolute_times), len(slots)), dtype=int)
    changes = []
    for i, events in enumerate(history):
        data = np.asarray(events, dtype=float)
        if len(data) == 0 or np.any(np.diff(data[:, 0]) < 0):
            raise ValueError('Missing or nonmonotonic applied assignment evidence')
        indices = np.searchsorted(data[:, 0], absolute_times, side='right')-1
        if np.any(indices < 0):
            raise ValueError('Applied assignment evidence begins after departure')
        selected = data[indices, 1].astype(int)
        if np.any((selected < 0) | (selected >= len(slots))):
            raise ValueError('Applied slot outside template')
        reference[:, i] = slots[selected]
        mappings[:, i] = selected
        changes.append(int(np.count_nonzero(np.diff(data[:, 1]))))
    return reference, mappings, changes


def analyze_run(run, output, sample_hz, check_hz, s0, include_failures):
    streams, desired, result, ack_counts, history = load_bag(run, include_failures)
    start = result['departure_ros_time']
    reached = result.get('all_arrived_ros_time')
    end = reached if reached is not None else result.get('recording_end_ros_time')
    if start is None or end is None:
        raise ValueError('No usable flight interval')
    t, xyz, gap = sample_formation(streams, start, end, sample_hz)
    reference, mappings, changes = applied_reference(desired, history, t+start)
    summary, residual, graph, scales = evaluate(t, xyz, reference, s0)
    tc, pc, _ = sample_formation(streams, start, end, check_hz)
    coarse_reference, _, _ = applied_reference(desired, history, tc+start)
    check, _, _, _ = evaluate(tc, pc, coarse_reference, s0)
    summary.update(trial=run.name, success=result['success'], reason=result['reason'],
                   interval='departure_to_all_arrived' if reached is not None else 'departure_to_failure_stop',
                   sample_count=len(t), max_odom_gap_s=gap,
                   applied_slot_changes=sum(changes),
                   coarse_e_dist_eq45=check['e_dist_eq45'],
                   coarse_e_sim_eq46=check['e_sim_eq46'],
                   relative_resolution_change=abs(summary['e_dist_eq45']-check['e_dist_eq45'])/max(summary['e_dist_eq45'], 1e-14))
    np.savetxt(str(output/(run.name+'_timeseries.csv')),
               np.column_stack([t, t/t[-1], residual, graph, scales, xyz.mean(axis=1), mappings]),
               delimiter=',', header='elapsed_s,normalized_time,aligned_sum_squared_error_m2,graph_squared_frobenius_error,alignment_scale,centroid_x,centroid_y,centroid_z,'+
               ','.join('drone_%d_applied_slot' % i for i in range(len(desired))), comments='')
    save_json(output/(run.name+'_audit.json'), dict(
        bag=str(recording_bag_path(run, result).resolve()), departure_ros_time=start,
        analysis_end_ros_time=end, desired_positions_m=desired.tolist(),
        odometry_counts=[len(s) for s in streams], applied_ack_counts=ack_counts,
        per_agent_applied_assignment_verified=True, **summary))
    return summary


def analyze(args):
    output = args.output or args.batch/'formation_error_paper'
    output.mkdir(parents=True, exist_ok=True)
    summaries = []
    for run in sorted(args.batch.glob('trial_*')):
        if not (run/'result.json').exists():
            continue
        result = json.loads((run/'result.json').read_text())
        if not recording_bag_path(run, result).exists():
            continue
        if result.get('departure_ros_time') is None:
            continue
        audit = output/(run.name+'_audit.json')
        if args.incremental and audit.exists():
            old = json.loads(audit.read_text())
            summary = {k: old[k] for k in ('e_dist_eq45', 'e_sim_eq46', 'residual_time_mean_m2',
                'graph_time_mean', 'centroid_path_length_m', 'duration_s', 'residual_max_m2',
                'alignment_scale_min', 'alignment_scale_max', 'trial', 'success', 'reason',
                'interval', 'sample_count', 'max_odom_gap_s', 'applied_slot_changes',
                'coarse_e_dist_eq45', 'coarse_e_sim_eq46', 'relative_resolution_change')}
        else:
            summary = analyze_run(run, output, args.sample_hz, args.check_hz, args.s0, args.include_failures)
        summaries.append(summary)
        print(run.name, 'e_dist=%.9f e_sim=%.9f' % (summary['e_dist_eq45'], summary['e_sim_eq46']), flush=True)
    if not summaries:
        raise ValueError('No bag files found')
    with (output/'per_trial.csv').open('w') as f:
        writer = csv.DictWriter(f, fieldnames=list(summaries[0]))
        writer.writeheader()
        writer.writerows(summaries)
    mean = lambda key: float(np.mean([s[key] for s in summaries]))
    summary = dict(trials=len(summaries), e_dist_mean=mean('e_dist_eq45'),
        e_dist_sample_std=float(np.std([s['e_dist_eq45'] for s in summaries], ddof=1)) if len(summaries)>1 else 0,
        e_sim_mean=mean('e_sim_eq46'),
        e_sim_sample_std=float(np.std([s['e_sim_eq46'] for s in summaries], ddof=1)) if len(summaries)>1 else 0,
        pooled_path_weighted_e_dist=float(np.average([s['e_dist_eq45'] for s in summaries], weights=[s['centroid_path_length_m'] for s in summaries])),
        residual_time_mean_m2=mean('residual_time_mean_m2'),
        max_relative_resolution_change=max(s['relative_resolution_change'] for s in summaries),
        sample_hz=args.sample_hz, check_hz=args.check_hz, s0=args.s0,
        path_convention='3D arithmetic centroid arc length',
        reference='full eight-agent 2x4 rectangle, 1m adjacent spacing; each physical drone follows its own planner-applied slot',
        interval='departure to all-arrived, or to failure stop for failed runs; includes split/merge',
        equation45='integral(sum_i ||p_des_i-(s R p_i+t)||^2 dl)/(s0 * centroid_path_length)',
        equation46='integral(||Lhat-Lhat_des||_F^2 dl)/(s0 * centroid_path_length)',
        paper='Quan et al., Robust and Efficient Trajectory Planning for Formation Flight in Dense Environments, DOI 10.1109/TRO.2023.3301295, p.12 Eqs.(45)-(46)')
    successful = [s['e_dist_eq45'] for s in summaries if s['success']]
    summary['successful_trials'] = len(successful)
    summary['e_dist_successful_mean'] = float(np.mean(successful)) if successful else None
    summary['failed_trials'] = sum(not s['success'] for s in summaries)
    if args.paper:
        summary['paper_path'] = str(args.paper.resolve())
        summary['paper_sha256'] = hashlib.sha256(args.paper.read_bytes()).hexdigest()
    save_json(output/'summary.json', summary)
    (output/'analysis_source.py').write_bytes(Path(__file__).read_bytes())
    print(json.dumps(summary, indent=2, ensure_ascii=False), flush=True)
    return output


def plot(output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    files = sorted(output.glob('trial_*_timeseries.csv'))
    grid = np.linspace(0, 1, 501)
    curves = []
    fig, ax = plt.subplots(figsize=(9, 4.8))
    for path in files:
        x = np.genfromtxt(str(path), delimiter=',', names=True)
        curves.append(np.interp(grid, x['normalized_time'], x['aligned_sum_squared_error_m2']))
        ax.plot(x['normalized_time']*100, x['aligned_sum_squared_error_m2'], color='#438cb4', alpha=.16, linewidth=.7)
    curves = np.asarray(curves)
    ax.plot(grid*100, curves.mean(axis=0), color='#123f60', linewidth=2, label='Mean instantaneous residual (%d trials)' % len(files))
    ax.set(xlabel='Flight progress in normalized time (%)',
           ylabel=r'Aligned squared residual $\sum_i\|p_i^d-(sRp_i+t)\|^2$ ($m^2$)',
           title='Sim(3)-aligned formation distortion — full eight-agent rectangle')
    ax.grid(alpha=.2)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(str(output/'formation_error_timeline.png'), dpi=180)
    fig.savefig(str(output/'formation_error_timeline.pdf'))
    plt.close(fig)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('batch', type=Path)
    p.add_argument('--output', type=Path)
    p.add_argument('--sample-hz', type=float, default=100)
    p.add_argument('--check-hz', type=float, default=50)
    p.add_argument('--s0', type=float, default=1.0, help='initial formation scale; unit-spacing template uses 1')
    p.add_argument('--paper', type=Path)
    p.add_argument('--include-failures', action='store_true', help='failed flights evaluated to their recording stop; labeled separately')
    p.add_argument('--incremental', action='store_true', help='reuse cached trial audit results within this batch')
    p.add_argument('--completion-marker', type=Path, help=argparse.SUPPRESS)
    p.add_argument('--plot-only', action='store_true', help='batch argument is the existing analysis output directory')
    args = p.parse_args()
    if min(args.sample_hz, args.check_hz, args.s0) <= 0:
        p.error('rates and s0 must be positive')
    if args.plot_only:
        plot(args.batch)
    else:
        output = analyze(args)
        if args.completion_marker:
            temporary = Path(str(args.completion_marker)+'.tmp')
            save_json(temporary, dict(complete=True, output=str(output.resolve())))
            temporary.replace(args.completion_marker)


if __name__ == '__main__':
    main()
