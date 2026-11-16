import pandas as pd
import matplotlib.pyplot as plt
import os
import json
import argparse

def classify_ripple_effect(run_dir):
    """
    Classify the ripple effect into 4 sub-categories based on rigorous physical causality chains.
    Reads FSM traces, RX latency logs, and root cause timings directly.
    """
    root_cause_path = os.path.join(run_dir, "root_cause_report.json")
    
    first_emergency = 0.0
    first_col = 0.0
    run_id = ""
    drone_a = -1
    drone_b = -1

    # 1. Parse Collision Base Info
    try:
        with open(root_cause_path, 'r') as f:
            rc_data = json.load(f)
            signals = rc_data.get('analysis', {}).get('signals', {})
            first_emergency = signals.get('first_emergency_ros_time', 0.0)
            first_col = signals.get('first_collision_ros_time', 0.0)
            run_id = rc_data.get('run_id', '')
            
            first_col_line = signals.get('first_collision_line', '')
            if "\"drone_a\":" in first_col_line:
                import re
                match_a = re.search(r'"drone_a":\s*(\d+)', first_col_line)
                match_b = re.search(r'"drone_b":\s*(\d+)', first_col_line)
                if match_a and match_b:
                    drone_a = int(match_a.group(1))
                    drone_b = int(match_b.group(1))
    except Exception:
        pass

    # CONDITION 1: Kinematic Feasibility Bleed
    if first_col > 0 and first_emergency > 0:
        dt = first_col - first_emergency
        # Stricter physics bound for "slow prolonged drifting". 
        # A normal physics stop takes < 0.3s. If dragging for > 1.5 seconds, it's extreme bleeding.
        if dt >= 1.5:  
            return "Kinematic Feasibility Bleed"

    if drone_a == -1 or drone_b == -1 or not run_id:
        return "Myopic Evasion" # Fallback
        
    fuzz_dir = os.path.join(run_dir, 'fuzz_report', run_id)
    
    # CONDITION 2: Mutual Blind-spot Collision
    # Check if the colliding pair actually had stale RX trajectories (latency > 0.5s) right before collision
    latency_a_to_b = 999.0
    latency_b_to_a = 999.0
    
    def get_last_rx_time(self_id, target_id):
        rx_file = os.path.join(fuzz_dir, f"planner_rx_trace_drone_{self_id}.jsonl")
        last_time = 0.0
        if os.path.exists(rx_file):
            with open(rx_file, 'r') as f:
                for line in f:
                    try:
                        record = json.loads(line)
                        if record.get('src_drone_id') == target_id:
                            t = record.get('ros_time', 0.0)
                            if t < first_col:
                                last_time = max(last_time, t)
                    except: pass
        return last_time

    last_rx_for_a = get_last_rx_time(drone_a, drone_b)
    if last_rx_for_a > 0:
        latency_a_to_b = first_col - last_rx_for_a
        
    last_rx_for_b = get_last_rx_time(drone_b, drone_a)
    if last_rx_for_b > 0:
        latency_b_to_a = first_col - last_rx_for_b

    # If either drone hadn't received a trajectory from the other for > 0.5s, they were flying blind relative to each other
    if (latency_a_to_b > 0.5 and latency_a_to_b != 999.0) or (latency_b_to_a > 0.5 and latency_b_to_a != 999.0):
        return "Mutual Blind-spot Collision"

    # CONDITION 3: Spatial Squeeze Ripple (Threat Chain Propagation)
    # Check if Drone A or B was squeezed by a THIRD drone leading up to the collision.
    # E.g., A was dodging C, but in doing so backed into B.
    def is_squeezed_by_third_party(self_id, partner_id):
        safety_file = os.path.join(fuzz_dir, f"planner_safety_trace_drone_{self_id}.jsonl")
        recent_threats = set()
        if os.path.exists(safety_file):
            with open(safety_file, 'r') as f:
                for line in f:
                    try:
                        record = json.loads(line)
                        t = record.get('ros_time', 0.0)
                        other_id = record.get('other_id', -1)
                        # Look at events within 3 seconds of the collision
                        if first_col - 3.0 < t < first_col:
                            if other_id != -1 and other_id != partner_id:
                                recent_threats.add(other_id)
                    except: pass
        return len(recent_threats) > 0

    if is_squeezed_by_third_party(drone_a, drone_b) or is_squeezed_by_third_party(drone_b, drone_a):
        return "Spatial Squeeze Ripple"

    # Default if no physical sliding, no blind-spot, and no 3rd-party spatial squeeze
    return "Myopic Evasion"

def main():
    parser = argparse.ArgumentParser(
        description="Plot vulnerabilities_report.csv from an experiment directory."
    )
    parser.add_argument("result_dir", help="Directory containing vulnerabilities_report.csv")
    args = parser.parse_args()
    base_dir = os.path.abspath(os.path.expanduser(args.result_dir))
    report_path = os.path.join(base_dir, "vulnerabilities_report.csv")
        
    out_dir = os.path.dirname(report_path)
    
    if not os.path.exists(report_path):
        print(f"Report not found at {report_path}.")
        return
        
    df = pd.read_csv(report_path)
    
    # Filter out RUNTIME_FATAL_CRASH
    df_bugs = df[df['vulnerability_class'] != 'RUNTIME_FATAL_CRASH'].copy()
    
    # Sub-classify Ripple Effect
    subclasses = []
    base_dir = os.path.dirname(report_path)  # Directory containing the report and runs.
    
    for idx, row in df_bugs.iterrows():
        vuln_class = row['vulnerability_class']
        if vuln_class == 'VULN_1_RIPPLE_EFFECT':
            # The run directory from the report json path in CSV
            # Resolve the recorded run basename relative to the report directory.
            raw_run_dir = str(row.get('round_dir', ''))
            run_basename = os.path.basename(raw_run_dir.rstrip('/'))
            actual_run_dir = os.path.join(base_dir, run_basename)
            
            subclass = classify_ripple_effect(actual_run_dir)
            subclasses.append(subclass)
        else:
            subclasses.append("Not Ripple")
            
    df_bugs['ripple_subclass'] = subclasses
    
    # Count frequencies
    counts = df_bugs['vulnerability_class'].value_counts()
    ripple_counts = df_bugs[df_bugs['vulnerability_class'] == 'VULN_1_RIPPLE_EFFECT']['ripple_subclass'].value_counts()
    
    # Create a figure with three subplots
    fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(20, 6))
    
    # Palette
    colors = plt.get_cmap("Pastel1")(range(len(counts)))
    ripple_colors = plt.get_cmap("Set3")(range(len(ripple_counts)))
    
    # Pie Chart 1 - Main
    ax1.pie(counts, labels=counts.index, autopct='%1.1f%%', startangle=140, colors=colors)
    ax1.set_title('Main Vulnerabilities (Pie)')
    
    # Bar Chart 1 - Main
    ax2.barh(counts.index, counts.values, color=plt.get_cmap("viridis")(range(len(counts))))
    ax2.set_title('Main Vulnerabilities (Bar)')
    ax2.set_xlabel('Count')
    
    # Pie Chart 2 - Subclasses
    if not ripple_counts.empty:
        ax3.pie(ripple_counts, labels=ripple_counts.index, autopct='%1.1f%%', startangle=140, colors=ripple_colors)
        ax3.set_title('Ripple Effect Breakdowns')
    else:
        ax3.text(0.5, 0.5, "No Ripple Effects Found", horizontalalignment='center')
        ax3.set_axis_off()
    
    plt.tight_layout()
    
    # Save
    plot_path = os.path.join(out_dir, "vulnerability_distribution_detailed.png")
    plt.savefig(plot_path, dpi=300)
    print(f"Detailed plot saved successfully to: {plot_path}")

if __name__ == "__main__":
    main()
