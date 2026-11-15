"""Generate an isolated include of the actual RACER swarm.launch."""
import xml.etree.ElementTree as ET
from .core import ROOT

def launch_for(output,plan,cfg,map_path):
 root=ET.Element('launch');inc=ET.SubElement(root,'include',file=str(ROOT/'swarm_exploration/exploration_manager/launch/swarm.launch'))
 args=dict(ripple_enabled=not plan.get('baseline',False),ripple_control=plan.get('baseline',False),ripple_target=plan['target'],
     ripple_group=plan.get('group','G1'),ripple_seed=plan['seed'],ripple_trial=plan['trial_id'],ripple_source=plan['source_id'],
     ripple_rules=plan['rules_file'],ripple_output=str(output),ripple_budget_s=cfg['task_budget_s'],map_file=str(map_path))
 if plan.get('accuracy_mode'):args['ripple_experiment']='accuracy'
 for name,value in args.items():ET.SubElement(inc,'arg',name=name,value=str(value).lower() if isinstance(value,bool) else str(value))
 solver=output/'solver';solver.mkdir()
 for i in cfg['active_ids']:
  for node in ('exploration_node_%d'%i,'tsp_solver_%d'%i,'acvrp_solver_%d'%i):
   ET.SubElement(root,'param',name='/'+node+'/exploration/mtsp_dir',value=str(solver))
  ET.SubElement(root,'param',name='/exploration_node_%d/exploration/tsp_dir'%i,value=str(solver))
 ET.SubElement(root,'param',name='/use_sim_time',value='false')
 path=output/'isolated.launch';ET.ElementTree(root).write(str(path),encoding='unicode');return path
