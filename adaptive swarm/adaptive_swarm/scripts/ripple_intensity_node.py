#!/usr/bin/env python3
"""ROS configuration wrapper for isolated Python simulation. No flight/radio API."""
from pathlib import Path
import os
import sys
from urllib.parse import urlparse


def build_arguments(get):
    mode=get('~experiment','intensity')
    if mode not in ('intensity','accuracy'):raise ValueError('experiment must be intensity or accuracy')
    argv=['run','--target',get('~target','drone_drone'),'--group',get('~group','G1'),
        '--rules',get('~rules_file'), '--seed',str(get('~seed',20261111)),
        '--trial-id',str(get('~trial_id',1)),'--output-parent',get('~output_parent'),
        '--max-ticks',str(get('~max_ticks',4000)), '--motion-tolerance-m',str(get('~motion_tolerance_m',0.)),
        '--offset-tolerance-m',str(get('~offset_tolerance_m',0.))]
    if mode=='intensity':
        for i in range(1,5):argv.extend(['--g%d-delay-s'%i,str(get('~g%d_delay_s'%i,i*.2))])
    if not get('~enabled',False):argv.append('--control')
    if get('~diagnostic_position_adapter',False):argv.append('--diagnostic-position-adapter')
    for flag,param in [('--body-radius-m','~body_radius_m'),('--timeout-threshold-s','~timeout_threshold_s')]:
        if get(param,-1.)>0:argv.extend([flag,str(get(param))])
    return mode,argv


def main():
    master=urlparse(os.environ.get('ROS_MASTER_URI','http://localhost:11311'))
    if master.hostname not in ('localhost','127.0.0.1','::1'):
        raise RuntimeError('isolated experiment requires a localhost ROS master')
    import rospy
    import rospkg
    rospy.init_node('ripple_intensity',anonymous=False)
    package=Path(rospkg.RosPack().get_path('adaptive_swarm'))
    sys.path.insert(0,str(package.parent))
    mode,argv=build_arguments(rospy.get_param)
    if mode=='accuracy':
        from adaptive_swarm.scripts.layered_planner.latency_eval.accuracy_experiment import main as experiment
    else:
        from adaptive_swarm.scripts.layered_planner.latency_eval.intensity_experiment import main as experiment
    experiment(argv)


if __name__=='__main__':main()
