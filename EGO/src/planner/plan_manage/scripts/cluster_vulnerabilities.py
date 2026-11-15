import os
import csv
import collections

def main():
    import sys
    if len(sys.argv) > 1:
        base_dir = os.path.abspath(sys.argv[1])
        summary_csv = os.path.join(base_dir, "summary.csv")
        out_file = os.path.join(base_dir, "vulnerabilities_report.csv")
    else:
        summary_csv = "/home/dawang3111/EGO-Planner-v2/swarm-playground/main_ws/results/headless/20260410_202929/summary.csv"
        out_file = "/home/dawang3111/EGO-Planner-v2/swarm-playground/main_ws/results/headless/20260410_202929/vulnerabilities_report.csv"
    
    if not os.path.exists(summary_csv):
        print(f"Error: {summary_csv} not found.")
        return
        
    bugs = []
    counters = collections.Counter()
    
    with open(summary_csv, 'r', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for row in reader:
            if row.get('strict_result') != 'FAIL':
                continue
                
            target_id = row.get('comm_fuzz_target_drone_id', '').strip()
            collision_pair = row.get('collision_pair', '').strip()
            fuzz_mode = row.get('comm_fuzz_mode', '').strip()
            t_col_str = row.get('first_collision_time', 'NA')
            t_emg_str = row.get('first_emergency_time', 'NA')
            primary = row.get('root_cause_primary', '')
            
            has_collision = (primary == 'collision') or (collision_pair not in ('', 'none', 'NA'))
            
            try: t_col = float(t_col_str)
            except: t_col = None
            try: t_emg = float(t_emg_str)
            except: t_emg = None
                
            bug_class = "UNKNOWN"
            reason = "No specific rules matched"
            
            target_in_pair = False
            if collision_pair and collision_pair not in ('NA', 'none'):
                parts = collision_pair.replace('-', '_').split('_')
                ids_in_collision = [p for p in parts if p.isdigit()]
                if not ids_in_collision:
                    ids_in_collision = [''.join([c for c in p if c.isdigit()]) for p in parts]
                if target_id in ids_in_collision:
                    target_in_pair = True
            
            if has_collision:
                if not target_in_pair and target_id and collision_pair not in ('NA', 'none'):
                    bug_class = "VULN_1_RIPPLE_EFFECT"
                    reason = f"Target {target_id} NOT in pair {collision_pair}."
                elif t_col is not None and t_emg is not None and (t_col - t_emg) > 0.05:
                    bug_class = "VULN_2_KINEMATIC_E_STOP_FAILURE"
                    reason = f"Collision happened {t_col - t_emg:.2f}s after E-stop."
                elif target_in_pair and fuzz_mode in ['noise', 'jitter', 'drift']:
                    bug_class = "VULN_3_OUTLIER_ACCEPTANCE"
                    reason = f"Target {target_id} crashed under '{fuzz_mode}'."
                elif target_in_pair and fuzz_mode in ['delay', 'drop']:
                    bug_class = "VULN_4_TIMEOUT_DEGRADATION"
                    reason = f"Target {target_id} crashed under '{fuzz_mode}'."
                else:
                    bug_class = "VULN_5_ORTHOGONAL_MARGIN_DEFICIT"
                    reason = "Direct collision lacking distinct fuzz signatures."
            else:
                if primary == 'fatal_crash':
                    bug_class = "RUNTIME_FATAL_CRASH"
                elif primary == 'emergency_stop':
                    bug_class = "EMERGENCY_STOP_SUCCESS"
                else:
                    bug_class = "OTHER_FAILURE"
                    
            row['vulnerability_class'] = bug_class
            row['vulnerability_reason'] = reason
            bugs.append(row)
            counters[bug_class] += 1
            
    if bugs:
        with open(out_file, 'w', newline='', encoding='utf-8') as f:
            writer = csv.DictWriter(f, fieldnames=list(bugs[0].keys()))
            writer.writeheader()
            writer.writerows(bugs)
            
    print("\n" + "="*50)
    print(f"{'Vulnerability Clustering Summary':^50}")
    print("="*50)
    for b, c in counters.most_common():
        print(f"{b:38}: {c} occurrences")
    print("="*50)
    print(f"Detailed FAIL report saved to:\n{out_file}\n")

if __name__ == "__main__":
    main()
