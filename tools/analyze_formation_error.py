#!/usr/bin/env python3
"""Evaluate paper Eqs. (45)/(46) from eight-drone benchmark bag odometry.

Extract with ROS Noetic (--extract-only), then analyze with NumPy/Matplotlib.
The primary path measure is the swarm centroid; alternative path measures are
reported because the paper does not identify the representative trajectory.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np

from benchmark_files import artifact_path


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def save_json(path, data):
    path.write_text(json.dumps(data, indent=2, allow_nan=False) + '\n')


def extract_bags(batch, output):
    import rosbag
    cache = output / 'bag_odometry'
    cache.mkdir(parents=True, exist_ok=False)
    records = []
    for directory in sorted(batch.glob('trial_*')):
        if not (artifact_path(directory, 'result')).exists():
            continue
        result = json.loads((artifact_path(directory, 'result')).read_text())
        if not result.get('bag_valid'):
            raise ValueError('Unvalidated bag: ' + str(directory))
        topics = ['/drone_%d_visual_slam/odom' % i for i in range(8)]
        lookup = {topic: i for i, topic in enumerate(topics)}
        rows = [[] for _ in topics]
        with rosbag.Bag(str(artifact_path(directory, 'bag')), 'r') as bag:
            for topic, msg, _ in bag.read_messages(topics=topics):
                p = msg.pose.pose.position
                rows[lookup[topic]].append([msg.header.stamp.to_sec(), p.x, p.y, p.z])
        arrays = {'drone_%d' % i: np.array(row, dtype=np.float64) for i, row in enumerate(rows)}
        for i, topic in enumerate(topics):
            if len(rows[i]) != result['topic_message_counts'][topic]:
                raise ValueError('Bag message count differs from original audit')
        target = cache / (directory.name + '.npz')
        np.savez_compressed(str(target), **arrays)
        records.append(dict(trial=result['trial'], source_bag=str(artifact_path(directory, 'bag')),
                            bag_sha256=digest(artifact_path(directory, 'bag')),
                            metadata_sha256=digest(artifact_path(directory, 'metadata')),
                            result_sha256=digest(artifact_path(directory, 'result')),
                            cache_sha256=digest(target), counts=[len(row) for row in rows]))
        print('Extracted %s: %d original odometry messages' % (directory.name, sum(map(len, rows))), flush=True)
    save_json(output / 'extraction.json', dict(source='ROS 1 rosbag.read_messages; header timestamps and XYZ', trials=records))


def normalized_laplacian(positions):
    """Eqs. (1)-(4): complete graph, squared-distance edges, symmetric D^-1/2."""
    positions = np.asarray(positions, dtype=float)
    delta = positions[..., :, None, :] - positions[..., None, :, :]
    adjacency = np.sum(delta * delta, axis=-1)
    degrees = np.sum(adjacency, axis=-1)
    if np.any(degrees <= 1e-14):
        raise ValueError('Normalized graph undefined for coincident collapsed formation')
    return np.eye(positions.shape[-2]) - adjacency / np.sqrt(degrees[..., :, None] * degrees[..., None, :])


def aligned_squared_error(actual, desired):
    """min_{s>0,R in SO(3),t} SUM_i ||desired_i - (s R actual_i+t)||^2.

    Batched SVD similarity alignment; row-vector rotation is the transpose of
    the column-vector R in the paper. Reflections are explicitly excluded.
    """
    actual = np.asarray(actual, dtype=float)
    desired = np.asarray(desired, dtype=float)
    x = actual - np.mean(actual, axis=-2, keepdims=True)
    y = desired - np.mean(desired, axis=-2, keepdims=True)
    denominator = np.sum(x * x, axis=(-2, -1))
    if np.any(denominator <= 1e-14):
        raise ValueError('Cannot align a fully collapsed formation')
    covariance = np.swapaxes(x, -1, -2) @ y
    u, singular, vt = np.linalg.svd(covariance)
    signs = np.ones_like(singular)
    signs[..., -1] = np.where(np.linalg.det(u @ vt) < 0, -1, 1)
    rotation = (u * signs[..., None, :]) @ vt
    scale = np.sum(singular * signs, axis=-1) / denominator
    if np.any(scale <= 0):
        raise ValueError('Positive-scale optimum is degenerate')
    residual = y - scale[..., None, None] * (x @ rotation)
    return np.sum(residual * residual, axis=(-2, -1)), scale


def synchronize(streams, start, end, hz, max_gap=0.1):
    duration = end - start
    if duration <= 0 or hz <= 0:
        raise ValueError('Positive duration and sampling frequency required')
    relative = np.r_[np.arange(0.0, duration, 1.0 / hz), duration]
    positions = np.empty((len(relative), len(streams), 3))
    gaps = []
    for i, values in enumerate(streams):
        if values.ndim != 2 or values.shape[1] != 4 or len(values) < 2 or not np.isfinite(values).all():
            raise ValueError('Invalid timestamp/XYZ stream')
        deltas = np.diff(values[:, 0])
        if np.any(deltas <= 0):
            raise ValueError('Odometry timestamps must be strictly increasing')
        if values[0, 0] > start or values[-1, 0] < end:
            raise ValueError('Mission is not covered by every drone; extrapolation forbidden')
        left = max(0, np.searchsorted(values[:, 0], start) - 1)
        right = min(len(values) - 1, np.searchsorted(values[:, 0], end))
        gap = float(np.max(np.diff(values[left:right + 1, 0])))
        if gap > max_gap:
            raise ValueError('Odometry gap exceeds %.3f s' % max_gap)
        gaps.append(gap)
        for axis in range(3):
            positions[:, i, axis] = np.interp(relative, values[:, 0] - start, values[:, axis + 1])
    return relative, positions, gaps


def spatial_average(values, increments, initial_scale):
    length = float(np.sum(increments))
    if length <= 1e-12 or initial_scale <= 0:
        raise ValueError('Positive trajectory length and initial formation scale required')
    return float(np.sum(0.5 * (values[:-1] + values[1:]) * increments) / (initial_scale * length))


def evaluate(positions, desired, relative, initial_scale):
    squared_error, scale = aligned_squared_error(positions, desired)
    laplacian_error = np.sum((normalized_laplacian(positions) - normalized_laplacian(desired)) ** 2,
                             axis=(-2, -1))
    centroid = np.mean(positions, axis=1)
    per_agent_step = np.linalg.norm(np.diff(positions, axis=0), axis=-1)
    measures = dict(centroid=np.linalg.norm(np.diff(centroid, axis=0), axis=-1),
                    leader0=per_agent_step[:, 0], mean_agent=np.mean(per_agent_step, axis=1))
    metrics = {}
    for name, steps in measures.items():
        metrics.update({name + '_path_length_m': float(np.sum(steps)),
                        'e_dist_' + name: spatial_average(squared_error, steps, initial_scale),
                        'e_sim_' + name: spatial_average(laplacian_error, steps, initial_scale)})
    metrics.update(time_mean_aligned_sse=float(np.trapz(squared_error, relative) / relative[-1]),
                   time_mean_laplacian_error=float(np.trapz(laplacian_error, relative) / relative[-1]),
                   max_instantaneous_aligned_sse=float(np.max(squared_error)),
                   max_instantaneous_laplacian_error=float(np.max(laplacian_error)))
    return metrics, dict(squared_error=squared_error, laplacian_error=laplacian_error,
                         scale=scale, centroid=centroid, centroid_step=measures['centroid'])


def analyze(args):
    batch, output = args.batch.resolve(), args.output.resolve()
    extraction = json.loads((output / 'extraction.json').read_text())
    records = {r['trial']: r for r in extraction['trials']}
    results = []
    series_dir = output / 'timeseries'
    series_dir.mkdir(exist_ok=True)
    for directory in sorted(batch.glob('trial_*')):
        if not (artifact_path(directory, 'result')).exists():
            continue
        original = json.loads((artifact_path(directory, 'result')).read_text())
        metadata = json.loads((artifact_path(directory, 'metadata')).read_text())
        trial = original['trial']
        record = records[trial]
        cache_path = output / 'bag_odometry' / (directory.name + '.npz')
        if (digest(cache_path) != record['cache_sha256']
                or digest(artifact_path(directory, 'metadata')) != record['metadata_sha256']
                or digest(artifact_path(directory, 'result')) != record['result_sha256']):
            raise ValueError('Input provenance mismatch for ' + directory.name)
        if not original['arrival_success']:
            raise ValueError('No arrival endpoint: define a failure-window policy before analyzing')
        start, end = original['departure_stamp_ros_s'], original['arrival_stamp_ros_s']
        with np.load(str(cache_path), allow_pickle=False) as cache:
            streams = [cache['drone_%d' % i] for i in range(8)]
        desired = np.asarray([metadata['ros_parameters']['drone_%d_ego_planner_node' % i]
                              ['formation']['drone%d' % i] for i in range(8)], dtype=float)
        relative, positions, gaps = synchronize(streams, start, end, args.hz)
        metrics, series = evaluate(positions, desired, relative, args.initial_scale)
        check_relative, check_positions, _ = synchronize(streams, start, end, args.hz / 2)
        coarse, _ = evaluate(check_positions, desired, check_relative, args.initial_scale)
        result = dict(trial=trial, flight_success=original['success'], arrival_success=original['arrival_success'],
                      wall_collision=original['sampled_wall_collision'],
                      pair_collision=original['sampled_pair_collision'],
                      departure_stamp_ros_s=start, arrival_stamp_ros_s=end, duration_s=end - start,
                      samples=len(relative), resample_hz=args.hz, initial_scale=args.initial_scale,
                      max_source_odom_gap_s=max(gaps), desired_positions_m=desired.tolist(), **metrics)
        result['convergence_half_rate_e_dist_absolute_difference'] = abs(metrics['e_dist_centroid'] - coarse['e_dist_centroid'])
        result['convergence_half_rate_e_sim_absolute_difference'] = abs(metrics['e_sim_centroid'] - coarse['e_sim_centroid'])
        cumulative = np.r_[0.0, np.cumsum(series['centroid_step'])]
        with (series_dir / (directory.name + '.csv')).open('w', newline='') as stream:
            writer = csv.writer(stream)
            writer.writerow(['from_departure_s', 'stamp_ros_s', 'centroid_x', 'centroid_y', 'centroid_z',
                             'centroid_arc_length_m', 'optimal_actual_to_desired_scale',
                             'aligned_squared_error_sum', 'laplacian_squared_frobenius_error'])
            writer.writerows(zip(relative, relative + start, *series['centroid'].T,
                                 cumulative, series['scale'], series['squared_error'], series['laplacian_error']))
        results.append(result)
        print('Trial %02d: e_dist=%.9f, e_sim=%.9f' % (trial, metrics['e_dist_centroid'], metrics['e_sim_centroid']), flush=True)
    numeric = ['e_dist_centroid', 'e_sim_centroid', 'e_dist_leader0', 'e_sim_leader0',
               'e_dist_mean_agent', 'e_sim_mean_agent']
    def aggregate(items):
        return dict(trials=len(items), **{key: dict(mean=float(np.mean([r[key] for r in items])),
                    sample_std=float(np.std([r[key] for r in items], ddof=1)) if len(items) > 1 else None,
                    minimum=min(r[key] for r in items), maximum=max(r[key] for r in items))
                    for key in numeric})
    success = [r for r in results if r['flight_success']]
    summary = dict(paper_title='Robust and Efficient Trajectory Planning for Formation Flight in Dense Environments',
                   equations=[45, 46], graph_equations=[1, 2, 3, 4, 5], primary_path_measure='centroid',
                   normalization='1/(s0 L); sum squared residuals over robots, no division by N and no square root',
                   initial_scale=args.initial_scale, initial_scale_convention='unit-scale original 1 m slot template; actual initial formation equals template plus translation',
                   sample_hz=args.hz, time_window='first actual departure to onset of confirmed all-drone arrival',
                   interpolation='linear interpolation of each original header-stamped XYZ to a common time grid; exact endpoints; no extrapolation',
                   quadrature='trapezoidal integration weighted by discrete path arc-length increments',
                   correspondence='fixed physical IDs 0..7, no assignment optimization',
                   rotation='proper SO(3), not reflections', all_trials=aggregate(results),
                   successful_trials_only=aggregate(success),
                   pooled_by_centroid_length={key: float(np.average([r[key] for r in results],
                                                        weights=[r['centroid_path_length_m'] for r in results]))
                                             for key in ['e_dist_centroid', 'e_sim_centroid']},
                   convergence_max_half_rate_e_dist_absolute_difference=max(r['convergence_half_rate_e_dist_absolute_difference'] for r in results),
                   convergence_max_half_rate_e_sim_absolute_difference=max(r['convergence_half_rate_e_sim_absolute_difference'] for r in results),
                   script_sha256=digest(Path(__file__)), trials=results)
    save_json(output / 'summary.json', summary)
    fields = ['trial', 'flight_success', 'wall_collision', 'pair_collision', 'duration_s', 'samples',
              'centroid_path_length_m', 'e_dist_centroid', 'e_sim_centroid',
              'leader0_path_length_m', 'e_dist_leader0', 'e_sim_leader0',
              'mean_agent_path_length_m', 'e_dist_mean_agent', 'e_sim_mean_agent',
              'convergence_half_rate_e_dist_absolute_difference', 'convergence_half_rate_e_sim_absolute_difference']
    with (output / 'per_trial.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction='ignore')
        writer.writeheader()
        writer.writerows(results)
    return summary


def report_and_plot(summary, output):
    import os
    os.environ.setdefault('MPLCONFIGDIR', '/tmp/ego-formation-matplotlib')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Patch
    trials = summary['trials']
    fig, axes = plt.subplots(2, 1, figsize=(11, 7.3), sharex=True, constrained_layout=True)
    for ax, key, label in zip(axes, ['e_dist_centroid', 'e_sim_centroid'],
                              [r'$\bar e_{dist}$ (Eq. 45)', r'$\bar e_{sim}$ (Eq. 46)']):
        colors = ['#b65b43' if not r['flight_success'] else '#39759b' for r in trials]
        ax.bar([r['trial'] for r in trials], [r[key] for r in trials], color=colors)
        mean = summary['all_trials'][key]['mean']
        ax.axhline(mean, color='#222222', linestyle='--', linewidth=1.2,
                   label='20-trial mean = %.6f' % mean)
        ax.set_ylabel(label)
        ax.set_ylim(bottom=0)
        ax.grid(axis='y', alpha=0.2)
        ax.set_axisbelow(True)
        ax.legend(loc='upper right', fontsize=9)
    axes[0].set_title('8-drone fixed-wall benchmark: Sim(3)-aligned formation error\n'
                      'Centroid arc-length weighting; initial scale = 1; all 20 trials')
    axes[1].set_xticks([r['trial'] for r in trials])
    axes[1].set_xlabel('Trial ID')
    axes[0].legend(handles=[axes[0].lines[0], Patch(color='#39759b', label='Flight passed collision check'),
                            Patch(color='#b65b43', label='Collision; included in mean')],
                   loc='upper right', fontsize=8)
    fig.savefig(str(output / 'formation_error.png'), dpi=200)
    fig.savefig(str(output / 'formation_error.pdf'))
    plt.close(fig)
    all_stats, success_stats = summary['all_trials'], summary['successful_trials_only']
    lines = ['# 论文公式（45）/（46）：20 次固定墙飞行的平均编队误差', '',
             '来源：用户提供的 *Robust and Efficient Trajectory Planning for Formation Flight in Dense Environments*，'
             'PDF 第 12 页，Section VIII，式（45）（46）；图定义见第 5 页式（1）—（5）。', '',
             '| 统计范围 | 次数 | 公式（45）e_dist，均值 ± 样本标准差 | 公式（46）e_sim，均值 ± 样本标准差 |',
             '|---|---:|---:|---:|']
    for label, stats in [('全部录制任务，包含碰撞', all_stats), ('仅通过原几何碰撞检查的任务', success_stats)]:
        a, b = stats['e_dist_centroid'], stats['e_sim_centroid']
        lines.append('| %s | %d | %.9f ± %.9f | %.9f ± %.9f |' %
                     (label, stats['trials'], a['mean'], a['sample_std'], b['mean'], b['sample_std']))
    lines += ['', '主结果是先计算每次飞行的路程平均误差，再对 20 次结果取等权算术平均。'
              '不会排除第 2 次撞墙和第 4 次机间碰撞记录。18 次子集仅供参考。', '',
              '## 公式与离散化', '',
              r'\[E_{dist}(t)=\min_{s>0,R\in SO(3),\mathbf t}\sum_{i=0}^{7}\|p_i^d-(sRp_i^c(t)+\mathbf t)\|^2\]', '',
              r'\[\bar e_{dist}=\frac{1}{s_0 L}\int_{\mathcal L}E_{dist}\,dl,\qquad '
              r'\bar e_{sim}=\frac{1}{s_0 L}\int_{\mathcal L}\|\hat L-\hat L_{des}\|_F^2\,dl\]', '',
              r'\[A_{ij}=\|p_i-p_j\|^2,\quad D_{ii}=\sum_j A_{ij},\quad '
              r'\hat L=I-D^{-1/2}AD^{-1/2}\]', '',
              r'\[\bar e\approx\frac{\sum_k\tfrac12(E_k+E_{k+1})\,\Delta l_k}{s_0\sum_k\Delta l_k}\]', '',
              '- 公式（45）是八机**残差平方和**，不除以 8，不开平方，不能作为单机 RMSE（米）解读。',
              '- 每个同步采样时刻独立求最优 Sim(3)，使用 SVD 闭式全局解，限制 R 为 SO(3)，不允许镜像。',
              '- 固定物理编号 0–7 与原始目标槽位对应，不重新分配编号。',
              '- 目标采用记录参数中的原始 2×4 编队槽位，横纵相邻间距 1 m。'
              '实际初始编队等于此模板加平移，因此取相对模板尺度 s0=1。此值是明确的归一化约定。',
              '- 采用原始里程计 header 时间戳，线性插值到共同 %.0f Hz 时间轴，包含精确起止端点；禁止外推。' % summary['sample_hz'],
              '- 区间沿用录包定义：首架无人机实际离开自身起点 0.05 m，至八机共同满足持续到达条件的区间起点；不含起飞前等待和到达确认尾段。',
              '- 论文未明确轨迹 L 是质心、领机还是其他轨迹。本报告主口径取八机质心的实际三维轨迹；'
              '以下同时列出领机及各机平均路程口径，便于后续统一比较。', '',
              '| 路程定义 | 20 次平均 e_dist | 20 次平均 e_sim |', '|---|---:|---:|']
    for name, label in [('centroid', '编队质心（主结果）'), ('leader0', '0 号领机'),
                        ('mean_agent', '八机路程增量的平均值')]:
        lines.append('| %s | %.9f | %.9f |' % (label, all_stats['e_dist_' + name]['mean'],
                                              all_stats['e_sim_' + name]['mean']))
    lines += ['', '不同路程口径会改变权重；这些是明确列出的实现约定，不能声称论文唯一指定了质心口径。', '',
              '50 Hz 与 100 Hz 的单次结果最大绝对差：e_dist = %.9g，e_sim = %.9g。' %
              (summary['convergence_max_half_rate_e_dist_absolute_difference'],
               summary['convergence_max_half_rate_e_sim_absolute_difference']), '',
              '## 逐次结果', '', '| 次数 | 原飞行判定 | e_dist | e_sim | 质心路程 (m) |',
              '|---:|---|---:|---:|---:|']
    for r in trials:
        lines.append('| %02d | %s | %.9f | %.9f | %.6f |' %
                     (r['trial'], '成功' if r['flight_success'] else
                      ('到达但撞墙' if r['wall_collision'] else '到达但机间碰撞'),
                      r['e_dist_centroid'], r['e_sim_centroid'], r['centroid_path_length_m']))
    lines += ['', '![逐包误差](formation_error.png)', '',
              '[逐次 CSV](per_trial.csv) · [完整结果 JSON](summary.json) · [原始 bag 提取来源及校验和](extraction.json) · '
              '[PDF 图](formation_error.pdf)', '',
              'timeseries/trial_NNN.csv 保存同步时间、质心位置/路程、最优缩放和每一时刻的两种误差；'
              'bag_odometry/trial_NNN.npz 保存直接从对应 bag 读取的原始时间戳与八机 XYZ。原 bag 和原录包结果均未修改。']
    (output / 'REPORT.md').write_text('\n'.join(lines) + '\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--batch', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--extract-only', action='store_true')
    parser.add_argument('--hz', type=float, default=100.0)
    parser.add_argument('--initial-scale', type=float, default=1.0)
    args = parser.parse_args()
    if not np.isfinite([args.hz, args.initial_scale]).all() or min(args.hz, args.initial_scale) <= 0:
        parser.error('sampling frequency and initial scale must be finite and positive')
    args.output.mkdir(parents=True, exist_ok=True)
    if args.extract_only:
        extract_bags(args.batch.resolve(), args.output.resolve())
    else:
        summary = analyze(args)
        report_and_plot(summary, args.output.resolve())
        print(json.dumps(summary['all_trials'], indent=2))


if __name__ == '__main__':
    main()
