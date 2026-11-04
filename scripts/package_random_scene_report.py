"""Package an independently validated, completed formation batch."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import re
import statistics

import numpy as np
import yaml


def save(path, value):
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False)+'\n')


def mean(values):
    return statistics.mean(values) if values else None


def std(values):
    return statistics.stdev(values) if len(values) > 1 else (0.0 if values else None)


def fmt(value, digits=6):
    return ('%.*f' % (digits, value)) if value is not None else '—'


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('batch', type=Path)
    parser.add_argument('--reference-batch', type=Path, required=True)
    parser.add_argument('--scene-number', type=int, required=True)
    parser.add_argument('--repository-root', type=Path, default=Path.cwd())
    args = parser.parse_args()
    p, ref = args.batch, args.reference_batch
    scene = 'scene%02d' % args.scene_number
    config = json.loads((p/'config.json').read_text())
    validation = json.loads((p/'validation.json').read_text())
    assert validation['passed'] and validation['complete']
    analysis = json.loads((p/'formation_error_paper/summary.json').read_text())
    previous = np.asarray(json.loads((ref/'trial_01/obstacle_geometry.json').read_text())['boxes_xyz_min_max'])
    reference_config = json.loads((ref/'config.json').read_text())
    fixed_count = len(previous)-reference_config['small_obstacle_count']
    root = args.repository_root.resolve()
    provenance = json.loads((p/'binary_provenance.json').read_text())
    for name, info in provenance.items():
        assert hashlib.sha256((root/'swarm-playground/formation_ws'/name).read_bytes()).hexdigest() == info['sha256'], name
    rows, checks, index, geometry_hashes, topic_counts = [], [], [], set(), set()
    for run in sorted(p.glob('trial_*')):
        r = json.loads((run/'result.json').read_text())
        r['trial'] = run.name
        rows.append(r)
        params = yaml.safe_load((run/'params.yaml').read_text())
        mp = params['sparse_random_obstacle_map']
        for key, param in [('random_seed', 'seed'), ('small_obstacle_count', 'small_obstacle_count'), ('small_position_seed', 'small_position_seed')]:
            assert mp[param] == config[key], (run.name, key)
        assert params['distributed_swap_enabled'] == config['swap_enabled']
        assert all(params['drone_%d_ego_planner_node' % i]['formation']['distributed_enabled'] for i in range(8))
        boxes = np.asarray(json.loads((run/'obstacle_geometry.json').read_text())['boxes_xyz_min_max'])
        assert len(boxes) == fixed_count+config['small_obstacle_count']
        np.testing.assert_allclose(boxes[:fixed_count], previous[:fixed_count], rtol=0, atol=1e-12)
        same_sizes = None
        positions_changed = None
        if config['small_obstacle_count'] == reference_config['small_obstacle_count']:
            for lo, hi in [(0, 1), (2, 3), (4, 5)]:
                np.testing.assert_allclose(boxes[fixed_count:, hi]-boxes[fixed_count:, lo], previous[fixed_count:, hi]-previous[fixed_count:, lo], rtol=0, atol=1e-12)
            same_sizes = True
            positions_changed = not np.allclose(boxes[fixed_count:, :4], previous[fixed_count:, :4])
            assert positions_changed
        geometry_hashes.add(hashlib.sha256((run/'obstacle_geometry.json').read_bytes()).hexdigest())
        enabled = set()
        if config['swap_enabled']:
            for f in (run/'ros_logs').glob('*/drone_*_formation_agent-*-stdout.log'):
                enabled.update(int(i) for i in re.findall(r'\[ASSIGNMENT_CONFIG\] drone=(\d+) enabled=1 scope=all', f.read_text(errors='replace')))
            assert enabled == set(range(8)), (run.name, enabled)
        assert r['bag_file'] == config['experiment_label']+'_run'+run.name[-2:]+'.bag'
        assert (run/r['bag_file']).is_file() and not Path(str(run/r['bag_file'])+'.active').exists()
        assert (run/r['positions_file']).is_file()
        if r.get('departure_ros_time') is not None:
            assert r['formation_error_status'] == 'ok'
        recorded_topics = json.loads((run/'topics.json').read_text())
        topic_counts.add(len(recorded_topics))
        r['planner_safety_event_messages'] = sum(
            info['count'] for name, info in recorded_topics.items()
            if name.endswith('/first_wall_violation'))
        checks.append(dict(trial=run.name, parameters_verified=True, fixed_obstacles_preserved=True,
                           small_sizes_preserved=same_sizes, small_positions_changed=positions_changed,
                           swap_enabled=config['swap_enabled']))
        index.append(dict(trial=run.name, bag=str(Path(run.name)/r['bag_file']),
            drone_positions=str(Path(run.name)/r['positions_file']), metrics=str(Path(run.name)/'result.json'),
            formation_error_timeseries='formation_error_paper/'+run.name+'_timeseries.csv',
            parameters=str(Path(run.name)/'params.yaml')))
    assert len(rows) == config['runs'] == validation['validated_trials']
    assert len(geometry_hashes) == 1
    save(p/'configuration_validation.json', dict(passed=True, reference_batch=str(ref.resolve()),
        trials=checks, geometry_variants=1, binaries_unchanged=True))
    with (p/(scene+'_recording_file_index.csv')).open('w') as f:
        writer = csv.DictWriter(f, fieldnames=list(index[0]))
        writer.writeheader()
        writer.writerows(index)
    successes = [r for r in rows if r['success']]
    runtimes = [r['completion_time_s'] for r in successes]
    errors = [r['formation_error_eq45'] for r in successes if r.get('formation_error_eq45') is not None]
    all_errors = [r['formation_error_eq45'] for r in rows if r.get('formation_error_eq45') is not None]
    stats = dict(scene=scene, planner='EGO-Planner-v2-commented',
        random_seed=config['random_seed'], small_obstacle_count=config['small_obstacle_count'],
        small_position_seed=config['small_position_seed'], coordination_mode='distributed',
        swap_enabled=config['swap_enabled'], trials=len(rows),
        all_arrived=sum(bool(r.get('all_arrived')) for r in rows),
        arrived_and_geometry_safe=sum(bool(r.get('all_arrived')) and bool(r.get('obstacle_geometry_received')) and not r.get('sampled_collisions') for r in rows),
        successful_trials=len(successes), success_rate=len(successes)/len(rows),
        recovered_recordings=[r['trial'] for r in rows if r.get('recording_recovery')],
        planner_safety_event_messages=sum(r['planner_safety_event_messages'] for r in rows),
        trials_with_planner_safety_events=[r['trial'] for r in rows if r['planner_safety_event_messages']],
        mean_runtime_s=mean(runtimes), runtime_sample_std_s=std(runtimes),
        min_runtime_s=min(runtimes) if runtimes else None, max_runtime_s=max(runtimes) if runtimes else None,
        mean_formation_error_eq45_m2=mean(errors), formation_error_sample_std_m2=std(errors),
        mean_formation_error_all_started_m2=mean(all_errors),
        mean_formation_error_eq46=analysis['e_sim_mean'],
        total_bag_bytes=sum(r['bag_bytes'] for r in rows),
        total_bag_mib=sum(r['bag_bytes'] for r in rows)/2**20,
        min_pair_distance_m=min(r['min_pair_distance_m'] for r in rows if r['min_pair_distance_m'] is not None),
        min_obstacle_distance_m=min(r['min_obstacle_distance_m'] for r in rows if r['min_obstacle_distance_m'] is not None),
        assignment_changes=sum(r.get('assignment_changes', 0) for r in rows),
        max_relative_resolution_change=analysis['max_relative_resolution_change'])
    save(p/(scene+'_summary_statistics.json'), stats)
    with (p/'summary.csv').open() as f:
        metric_rows = list(csv.DictReader(f))
    by_trial = {r['trial']: r for r in rows}
    fields = list(metric_rows[0])+['planner_safety_event_messages']
    with (p/(scene+'_trial_metrics.csv')).open('w') as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for r in metric_rows:
            r['planner_safety_event_messages'] = by_trial[r['trial']]['planner_safety_event_messages']
            writer.writerow(r)
    report_name = scene+'_experiment_report.md'
    text = ['# EGO-Planner-v2-commented：场景%d实验报告' % args.scene_number, '',
        '八机分布式，地图种子 %d，小块 %d 个，独立位置种子 %d；跨列换槽%s。关闭RViz，其余参数沿用场景默认配置。共%d轮，重复同一地图。' %
        (config['random_seed'], config['small_obstacle_count'], config['small_position_seed'], '开启' if config['swap_enabled'] else '关闭', len(rows)),
        '已逐轮验证：大型、堆叠及窄缝障碍与上一场景一致；小块数量、尺寸不变，仅位置改变。程序二进制在录制期间未发生变化。', '',
        '## 结果', '',
        '- 全员到达：**%d/%d**。' % (stats['all_arrived'], len(rows)),
        '- 全员到达且通过采样几何碰撞检查：**%d/%d（%.0f%%）**。' % (stats['arrived_and_geometry_safe'], len(rows), stats['arrived_and_geometry_safe']/len(rows)*100),
        '- 成功轮次平均运行时间：**%s s**；轮次间样本标准差 %s s，范围 %s–%s s。' % tuple(fmt(stats[k]) for k in ['mean_runtime_s','runtime_sample_std_s','min_runtime_s','max_runtime_s']),
        '- 成功轮次平均编队误差（式45）：**%s m²**；轮次间样本标准差 **%s m²**。' % (fmt(mean(errors),9),fmt(std(errors),9)),
        '- 所有已出发轮次（含失败）平均式45误差：%s m²；失败轮次按出发至终止统计，区间在CSV中单列。' % fmt(mean(all_errors),9),
        '- bag合计：**%.2f MiB**；单包%.2f–%.2f MiB，LZ4压缩。' % (stats['total_bag_mib'],min(r['bag_bytes'] for r in rows)/2**20,max(r['bag_bytes'] for r in rows)/2**20),
        '- 实际应用的分配变化共 %d 次。' % stats['assignment_changes'], '',
        '- 规划器first_wall_violation安全诊断共 %d 条，出现在 %s。' %
        (stats['planner_safety_event_messages'], '、'.join(stats['trials_with_planner_safety_events']) or '无轮次'), '',
        '## 文件入口', '',
        '- `%s_trial_metrics.csv`：每轮指标及bag/位置表路径。' % scene,
        '- `%s_summary_statistics.json`：总体统计。' % scene,
        '- `%s_recording_file_index.csv`：bag、物理无人机位置、指标、误差时序和参数文件索引。' % scene,
        '- `trial_01/%s_run01.bag` 等：场景、模式、地图种子、小块数量、位置种子及轮次均写入文件名。' % config['experiment_label'],
        '- 各轮 `drone_positions.csv` 为约20 Hz位置/速度/槽位/目标表；完整原频率Odometry在bag中。',
        '- 各轮 `result.json` 保存到达、采样几何碰撞检查、成功/失败原因、时间及离线误差。',
        '- `formation_error_paper/` 保存各轮CSV、100 Hz误差时序、50 Hz复核及PNG/PDF曲线。',
        '- `validation.json` 为逐包完整性检查；`configuration_validation.json` 为实际布局、参数、开关和二进制检查。',
        '- `planner_safety_diagnostics.json` 保存规划器first_wall_violation安全诊断原文与bag时间戳，各轮指标表单列消息数。',
        '- 参数、实际障碍盒小型JSON、源码快照、二进制/论文标识和日志均另存。', '',
        '## 统计口径', '',
        '出发定义为任一机相对首个odom位置移动至少0.05 m。八机均在各自实际目标0.30 m内、速度不超过0.15 m/s、处于WAIT_TARGET且安全状态通过，并稳定1 s后确认到达；稳定区间起点作为到达时刻，1 s确认时间不加入运行时间。要求odom与规划器状态保持新鲜。失败轮次的completion_time_s为null。',
        '成功还要求未发现机体中心进入实际障碍盒，或机间距离小于0.20 m。几何代理与净空以20 Hz采样检查，不代表物理接触检测或连续时间无碰撞证明。飞行超时240 s、启动90 s未出发、odom丢失超过5 s均按既定规则记录失败。',
        '规划器first_wall_violation消息另行统计，表示其局部占用图/墙保护曾触发告警。几何成功率采用实际障碍盒与机间距离口径，不能解释为全过程没有触发规划器告警；诊断原文保留于对应bag。',
        '本批次采样最小机间距离%.6f m，最小机体中心到障碍距离%.6f m。' % (stats['min_pair_distance_m'],stats['min_obstacle_distance_m']), '',
        '## 论文式45编队误差', '',
        'Quan等，Robust and Efficient Trajectory Planning for Formation Flight in Dense Environments，DOI 10.1109/TRO.2023.3301295，第12页式（45）。论文标识见paper_reference.json。', '',
        r'$$\bar e_{\mathrm{dist}}=\frac{1}{s_0 L}\int_{\mathcal L}\min_{R\in SO(3),t,s>0}\sum_{i=1}^{8}\|p_i^d-(sRp_i^c+t)\|^2\,\mathrm d\ell.$$', '',
        '直接读取bag中原始Odometry的header.stamp，将八机位置插值到共同100 Hz时间轴，逐时刻用SVD求最优Sim(3)对齐。保留八机残差平方和，不除以8、不取平方根。s0=1，目标模板为2×4、邻接间距1 m的完整矩形。按各机自身formation_applied回执跟随实际应用槽位，不做额外编号最优重排。',
        '论文没有唯一规定如何从各机轨迹提取整体路径，本计算明确用三维质心轨迹及其弧长做梯形积分；包含分裂绕障阶段，评价完整八机目标队形。先得到每轮路径平均，再对成功轮次等权平均；失败已出发轮次另行保留和标注。',
        'bag内/experiment/formation_error为约20 Hz在线估计，指标表中为100 Hz离线结果。50 Hz复核的式45最大相对差为%.6f%%；补充式46图误差均值为%.9f。' % (analysis['max_relative_resolution_change']*100,analysis['e_sim_mean']), '',
        '## 各轮结果', '',
        '| 轮次 | 成功 | 原因 | 运行时间/s | 式45误差/m² | 安全诊断/条 | bag/MiB |',
        '|---|---|---|---:|---:|---:|---:|']
    for r in rows:
        text.append('| %s | %s | %s | %s | %s | %d | %.2f |' % (r['trial'],'是' if r['success'] else '否',r['reason'],fmt(r.get('completion_time_s')),fmt(r.get('formation_error_eq45'),9),r['planner_safety_event_messages'],r['bag_bytes']/2**20))
    recovered = [r for r in rows if r.get('recording_recovery')]
    if recovered:
        text += ['', '## 收尾异常与恢复记录', '']
        for r in recovered:
            text += ['- [%s 恢复审计](%s/recording_recovery/README.md)：%s' %
                     (r['trial'], r['trial'], r['recording_recovery'].get('note', '详见原始审计记录。'))]
    if (p/'runtime_environment.json').exists():
        text += ['', '## 运行环境', '',
            '沿用上一批次验证过的Noetic隔离环境，使用与容器原文件SHA256一致的独立OpenSSL副本，绕过此前遇到的文件映射等待。规划器和地图生成器二进制保持不变。',
            '实际录制环境及归档副本信息见runtime_environment.json，启动命令见recording_invocation.json；独立库已另存于runtime_libs，本批次runtime_python.sh可独立执行后续分析。没有重启容器或主机。']
    text += ['', '## 重复录制', '',
        '在已source Noetic和formation_ws的仓库根目录执行，自动创建新的时间戳目录：', '',
        '```bash', 'python3 scripts/record_fixed_wall_batch.py \\',
        '  --scene sparse_random --runs %d --random-seed %d \\' % (config['runs'],config['random_seed']),
        '  --small-obstacle-count %d --small-position-seed %d \\' % (config['small_obstacle_count'],config['small_position_seed']),
        '  --experiment-label %s --port %d %s' %
        (config['experiment_label'],config['port'],'--swap-enabled' if config['swap_enabled'] else '--no-swap'), '```', '',
        'bag保留位置、速度、姿态、控制指令、规划轨迹、耗时、心跳、编队状态、协商和诊断。实际话题数量为%s；未录点云、图像、占用栅格或地图MarkerArray。完整话题清单见各轮topics.json。' % '/'.join(str(n) for n in sorted(topic_counts)), '']
    if (p/'runtime_python.sh').exists():
        text += ['当前机器隔离环境下重新录制一个新批次的命令（自动使用新目录和空闲端口）：', '', '```bash',
            'docker exec --user 1000:1000 split-merge /bin/bash \\',
            '  '+str(p.resolve()/'runtime_python.sh')+' \\',
            '  scripts/record_fixed_wall_batch.py --scene sparse_random --runs %d \\' % config['runs'],
            '  --random-seed %d --small-obstacle-count %d --small-position-seed %d \\' %
            (config['random_seed'], config['small_obstacle_count'], config['small_position_seed']),
            '  --experiment-label %s --port 0 %s' %
            (config['experiment_label'],'--swap-enabled' if config['swap_enabled'] else '--no-swap'), '```', '']
    (p/report_name).write_text('\n'.join(text))
    (p/'REPORT.md').write_text('# 场景%d实验结果\n\n[完整报告](%s)\n\n[各轮指标](%s_trial_metrics.csv) · [文件索引](%s_recording_file_index.csv)\n' % (args.scene_number,report_name,scene,scene))
    readme = p/'README.md'
    prefix = readme.read_text().split('\n录制与校验已完成：')[0] if readme.exists() else ''
    readme.write_text(prefix+'\n录制与校验已完成：[完整报告](%s)、[各轮指标](%s_trial_metrics.csv)、[文件索引](%s_recording_file_index.csv)。\n' % (report_name,scene,scene))
    (p/'report_source.py').write_bytes(Path(__file__).read_bytes())
    print(json.dumps(stats,indent=2,ensure_ascii=False))


if __name__ == '__main__':
    main()
