"""Human-readable results; never select candidates using holdout scores."""
import csv
import json
from collections import Counter

from .core import TARGETS


NAMES = dict(zip(TARGETS, ['机间碰撞', '机障碰撞', '死锁', '延迟/超时', '规划失败', '漂移']))
DEFINITIONS = {
    'source_velocity_norm': '源机速度模长', 'local_mean_velocity': '局部群体平均速度模长',
    'local_max_velocity': '局部群体最大速度模长', 'source_acceleration_norm': '源机加速度模长',
    'local_mean_acceleration': '局部群体平均加速度模长', 'local_max_acceleration': '局部群体最大加速度模长',
    'source_displacement_5s': '源机过去5秒净位移', 'local_max_displacement_5s': '局部群体过去5秒最大净位移',
    'source_heading_change_5s': '源机过去5秒航向变化绝对值', 'local_max_heading_change_5s': '群体过去5秒最大航向变化',
    'local_min_inter_drone_distance': '局部群体当前最小三维机间距',
    'local_future_3s_min_inter_drone_distance': '注入前已发布轨迹在未来3秒的采样最小机间距',
    'local_min_distance_change_5s': '当前最小机间距减去5秒前同一群体的最小机间距',
    'source_current_obstacle_distance': '源机当前位置到静态占据体素的最小距离',
    'source_future_3s_min_obstacle_distance': '源机注入前轨迹未来3秒到静态体素的采样最小距离',
    'source_planner_obstacle_distance': '源机在本地SDF中查询的障碍距离，地图更新时间未知',
    'local_replan_count_5s': '过去5秒局部群体进入PLAN_TRAJ的次数',
    'local_safety_replan_count_5s': '过去5秒局部群体预测避碰重规划事件数',
    'local_new_trajectory_count_5s': '过去5秒局部群体不同轨迹ID数量',
    'local_replans_after_trajectory_publication_5s': '过去5秒局部群体EXEC_TRAJ到PLAN_TRAJ的转换数',
    'source_trajectory_deviation': '源机仿真位置与注入前局部参考轨迹的偏差',
    'victim_trajectory_deviation': '当前最近邻机仿真位置与其参考轨迹的偏差',
    'local_max_trajectory_deviation': '局部群体最大的参考轨迹位置偏差',
    'source_victim_heading_angle': '源机与当前最近邻机的航向夹角',
    'local_max_heading_angle': '局部群体最大两机航向夹角',
    'source_active_grid_count': '源机当前活动网格数，非覆盖百分比',
    'source_frontier_count': '源机当前frontier数量', 'task_elapsed': '源机接受任务触发后的已用时间',
    'source_truth_broadcast_position_error_max': '接收端缓存源机位置与源机当前仿真位置的最大偏差',
}


def csv_rows(path):
    if not path.exists():
        return []
    with path.open() as f:
        return list(csv.DictReader(f))


def counts(rows, planned):
    done = [r for r in rows if r.get('status') not in ('not_run', 'planned_not_run')]
    invalid = [r for r in done if r.get('actual_injected') != 'true' or r.get('contract_valid') != 'true' or
               any(r.get(t + '_valid') != 'true' for t in TARGETS)]
    return {'planned_rounds': planned, 'finished_rounds': len(done),
            'completed_rounds': sum(r['status'] == 'completed' for r in done),
            'task_budget_exhausted_rounds': sum(r['status'] == 'task_budget_exhausted' for r in done),
            'actual_injected_rounds': sum(r.get('actual_injected') == 'true' for r in done),
            'skipped_rounds': sum(r.get('actual_injected') != 'true' and r['status'] in ('completed', 'task_budget_exhausted') for r in done),
            'invalid_rounds': len(invalid), 'pending_or_not_run_rounds': planned - len(done),
            'failed_rounds': sum(r['status'] not in ('completed', 'task_budget_exhausted') for r in done),
            'status_counts': dict(Counter(r['status'] for r in rows)),
            'skip_or_failure_reasons': dict(Counter(r.get('skip_reason') for r in done if r.get('skip_reason'))),
            'counting_note': 'finished=已终止轮次；completed=任务FINISH；跳过=正常终止但未实际注入；无效=任一评测有效性不足，计数可重叠；各目标分母独立'}


def pct(value):
    return '无可用证据' if value in ('', None) else '%.2f%%' % (float(value) * 100)


def append_results(path):
    root = path.parent
    trials = csv_rows(root / 'trials.csv')
    manifest = json.loads((root / 'manifest.json').read_text())
    planned = manifest.get('parameters', {}).get('trials', len(trials))
    state = counts(trials, planned)
    (root / 'run_summary.json').write_text(json.dumps(state, ensure_ascii=False, indent=2) + '\n')
    units = {}
    missing = Counter()
    for row in csv_rows(root / 'trial_features.csv'):
        for k, v in row.items():
            if k.endswith('__unit') and v:
                units[k[:-6]] = v
            if k.endswith('__missing_reason') and v:
                missing[(k[:-16], v)] += 1
    clusters = csv_rows(root / 'cluster_summary.csv')
    rules = csv_rows(root / 'rules_per_target.csv')
    lines = ['\n## 实际轮次与有效性\n', '```json', json.dumps(state, ensure_ascii=False, indent=2), '```',
             '\n发生率以每目标有效的实际注入样本为分母；一轮命中多个目标时分别计数。没有控制数据时不作因果解释。']
    for s in csv_rows(root / 'summary_per_target.csv'):
        target = s['target']
        lines += ['\n## ' + NAMES[target],
                  '有效 %s，命中 %s，未命中 %s，发生率 %s；无效 %s。' %
                  (s['valid_n'], s['hit_n'], s['miss_n'], pct(s['hit_rate']), s['invalid_n']),
                  '无效原因：' + s['invalid_reasons'], '\n命中/未命中聚类（train/holdout，簇编号无风险等级）：']
        for c in (r for r in clusters if r['target'] == target):
            lines.append('- label=%s，cluster=%s，训练 %s，留出 %s，%s；原单位中心：`%s`。' %
                         (c['label'], c['cluster'] or '无', c['training_n'], c['holdout_n'], c['status'], c['centers']))
        selected = [r for r in rules if r['target'] == target and r.get('expression')][:3]
        lines += ['\n候选规则按训练集排名展示，未按留出表现重新筛选：']
        if not selected:
            lines.append('证据不足：没有可生成并进行留出评估的候选规则。')
        for r in selected:
            conditions = json.loads(r['conditions'])
            text = ' AND '.join('%s %s %.6g %s' % (DEFINITIONS.get(c['field'], c['field']), c['op'], c['threshold'], units.get(c['field'], '')) for c in conditions)
            lines += ['- **%s**：%s。' % (r['rule_id'], text),
                      '  留出支持 %s/%s；触发后目标发生率 %s、未发生率 %s；未触发组发生率 %s；差异 %s 百分点（95%%CI %s 至 %s），提升率 %s（CI %s 至 %s）；置信标记 %s。' %
                      (r['support_n'], r['validation_n'], pct(r['target_rate']), pct(r['target_not_rate']), pct(r['nontrigger_target_rate']),
                       r['difference_pp'] or '不可估', r['difference_ci_low_pp'], r['difference_ci_high_pp'], r['lift'] or '不可估',
                       r['lift_ci_low'], r['lift_ci_high'], r['confidence']),
                      '  未命中样本触发 %s/%s，触发率 %s；控制样本触发 %s/%s，触发率 %s。缺失字段排除 %s；验证状态 %s。' %
                      (r['nonhit_trigger_n'], r['nonhit_sample_n'], pct(r['nonhit_trigger_rate']), r['control_trigger_n'], r['control_eligible_n'],
                       pct(r['control_trigger_rate']), r['excluded_missing_n'], r['validation_status'])]
    lines += ['\n## 字段定义与单位\n', '| 字段 | 定义 | 单位 |', '|---|---|---|']
    lines += ['| %s | %s | %s |' % (k, DEFINITIONS.get(k, k), units[k]) for k in sorted(units)]
    lines += ['\n## 字段缺失\n', '字段/原因计数不是互斥的轮次数；具体来源时间见 trial_features.csv。']
    lines += ['- %s：%s，%d 轮。' % (k, reason, n) for (k, reason), n in sorted(missing.items())]
    with path.open('a') as f:
        f.write('\n'.join(lines) + '\n')
