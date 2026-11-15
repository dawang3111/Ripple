#!/usr/bin/env python3
import json,sys,time,threading
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'tools'))
import rospy
from std_msgs.msg import String
from bspline.msg import Bspline
from latency_eval.rule_online import OnlineRules

def main():
 rospy.init_node('ripple_rule_monitor')
 packet=json.loads(Path(rospy.get_param('/racer_intensity/rules')).read_text())
 target=rospy.get_param('/racer_intensity/target');source=rospy.get_param('/racer_eval/source_id')
 monitor=OnlineRules(dict(source_id=source,target=target,rule_group=packet['groups'][target]),packet['config'])
 lock=threading.RLock();states={};command=rospy.Publisher('/racer_intensity/arm_%d'%source,String,queue_size=1)
 log=rospy.Publisher('/racer_eval/events',String,queue_size=100)
 def telemetry(msg):
  e=json.loads(msg.data);e['observed_ns']=time.monotonic_ns()
  with lock:
   monitor.observe(e)
   if e.get('id') in packet['config']['active_ids'] and 'fsm' in e:states[e['id']]=e['fsm']
 def trajectory(msg,i):
  e=dict(kind='trajectory',id=i,observed_ns=time.monotonic_ns(),ros_s=rospy.Time.now().to_sec(),traj_id=msg.traj_id,
     start_s=msg.start_time.to_sec(),order=msg.order,knots=list(msg.knots),pos_pts=[[p.x,p.y,p.z] for p in msg.pos_pts])
  with lock:monitor.observe(e)
 rospy.Subscriber('/racer_eval/events',String,telemetry,queue_size=20000)
 subs=[rospy.Subscriber('/planning/bspline_%d'%i,Bspline,trajectory,callback_args=i,queue_size=1000) for i in packet['config']['active_ids']]
 def tick(event):
  with lock:
   monitor.armed=False
   d=monitor.evaluate(time.monotonic_ns(),rospy.Time.now().to_sec(),states.get(source) in ('PLAN_TRAJ','PUB_TRAJ','EXEC_TRAJ'))
  if d:
   d.update(kind='intensity_rule_decision',id=source,fsm=states.get(source),ros_s=rospy.Time.now().to_sec())
   log.publish(String(data=json.dumps(d,allow_nan=False)))
   command.publish(String(data='%d %d'%(d['decision_ns'],d['expires_ns'])))
 rospy.Timer(rospy.Duration(.1),tick)
 rospy.spin()
if __name__=='__main__':main()
