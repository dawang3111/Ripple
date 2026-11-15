#include <exploration_manager/fast_exploration_fsm.h>
#include <exploration_manager/fast_exploration_manager.h>
#include <exploration_manager/expl_data.h>
#include <plan_manage/planner_manager.h>
#include <plan_env/sdf_map.h>
#include <active_perception/hgrid.h>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <ros/serialization.h>

namespace fast_planner {
namespace {
std::string vec(const Eigen::Vector3d& v) {
  std::ostringstream s;
  s << std::setprecision(17) << '[' << v.x() << ',' << v.y() << ',' << v.z() << ']';
  return s.str();
}
std::string number(double x) {
  if (!std::isfinite(x) || x < 0) return "null";
  std::ostringstream s;
  s << std::setprecision(17) << x;
  return s.str();
}
}  // namespace

int64_t FastExplorationFSM::evalNowNs() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}

void FastExplorationFSM::initEvaluation(ros::NodeHandle& nh) {
  nh.param("evaluation/enabled", eval_enabled_, false);
  if (!eval_enabled_) return;
  bool sim_clock = false;
  ros::param::param("/use_sim_time", sim_clock, false);
  if (sim_clock) {
    ROS_ERROR("RACER evaluation source adapter requires isolated wall-clock simulation; disabled");
    eval_enabled_ = false;
    return;
  }
  int source = -1;
  bool control = false;
  ros::param::param("/racer_eval/source_id", source, -1);
  ros::param::param("/racer_eval/control", control, false);
  ros::param::param("/racer_eval/offset_s", eval_gate_.offset_s, 10.0);
  ros::param::param("/racer_eval/fraction", eval_gate_.fraction, 0.5);
  ros::param::param("/racer_eval/run_budget_s", eval_run_budget_s_, 250.0);
  eval_gate_.enabled = !control && source == getId();
  ros::param::param("/racer_intensity/enabled", intensity_enabled_, false);
  if (intensity_enabled_) {
    eval_gate_.enabled = false;  // This mode delays trajectory broadcasts only.
    ros::param::param("/racer_intensity/group", intensity_gate_.group, 1);
    double seconds=0.2;
    ros::param::param("/racer_intensity/fixed_delay_s", seconds, 0.2);
    intensity_gate_.fixed_ns=int64_t(seconds*1e9);
    intensity_gate_.enabled=source==getId() && !control;
    ros::param::param("/racer_accuracy/enabled", accuracy_enabled_, false);
    if(accuracy_enabled_){
      double wait=0;ros::param::param("/racer_accuracy/wait_s",wait,0.0);
      if(!std::isfinite(wait)||wait<0){ROS_FATAL("Invalid accuracy window wait");ros::shutdown();return;}
      accuracy_gate_.wait_ns=int64_t(wait*1e9);
      intensity_gate_.group=5; // Identical standard Ripple strategy in all five accuracy groups.
    }
    intensity_arm_sub_=nh.subscribe<std_msgs::String>("/racer_intensity/arm_"+std::to_string(getId()),1,
      [this](const std_msgs::String::ConstPtr& m){
        int64_t decision=0,expires=0;std::istringstream input(m->data);auto now=evalNowNs();
        if(input>>decision>>expires && decision<=now && expires>now && expires-decision<=150000000){
          intensity_decision_ns_=decision;intensity_expires_ns_=expires;
        }
      });
    intensity_flush_sub_=nh.subscribe<std_msgs::Empty>("/racer_intensity/flush",1,
      [this](const std_msgs::Empty::ConstPtr&){intensityActions(intensity_gate_.flush());accuracyCancel("capture_shutdown");});
  }

  ros::param::param("/racer_eval/rule_mode", eval_gate_.wait_for_arm, false);
  if (eval_gate_.enabled && eval_gate_.wait_for_arm) {
    eval_rule_sub_ = nh.subscribe<std_msgs::String>("/racer_eval/rule_arm_" + std::to_string(getId()), 1,
        [this](const std_msgs::String::ConstPtr& msg) {
          int64_t decision = 0, expires = 0;
          std::istringstream input(msg->data);
          const auto now = evalNowNs();
          const bool active = state_ == PLAN_TRAJ || state_ == PUB_TRAJ || state_ == EXEC_TRAJ;
          if (eval_rule_decided_) return;
          if (!(input >> decision >> expires) || decision > now || expires <= now ||
              expires - decision > 150000000 || !eval_started_ || !active) {
            emitEvaluation("rule_arm_rejected", "\"reason\":\"invalid_expired_or_inactive\"");
            return;
          }
          eval_rule_decided_ = true;
          eval_rule_expiry_ns_ = expires;
          eval_gate_.armed = true;
          emitEvaluation("rule_armed", "\"decision_ns\":" + std::to_string(decision) +
              ",\"expires_ns\":" + std::to_string(expires));
        });
  }
  eval_pub_ = nh.advertise<std_msgs::String>("/racer_eval/events", 2000);
  eval_tx_timer_ = nh.createWallTimer(ros::WallDuration(0.001),
      [this](const ros::WallTimerEvent&) {
        evaluationActions(eval_gate_.timer(evalNowNs()));
        if(intensity_enabled_){
          if(state_==FINISH || (eval_started_ && (evalNowNs()-eval_start_ns_)*1e-9>=eval_run_budget_s_)) {
            intensityActions(intensity_gate_.flush());accuracyCancel("task_end_before_next_eligible_publication");
          }
          else intensityActions(intensity_gate_.timer(evalNowNs()));
        }
      });
  emitEvaluation("adapter_ready", "\"channel\":\"drone_state\",\"protocol_budget_s\":0.2");
}

void FastExplorationFSM::emitEvaluation(const std::string& kind, const std::string& fields) {
  std::ostringstream s;
  s << std::setprecision(17) << "{\"kind\":\"" << kind << "\",\"id\":" << getId()
    << ",\"event_seq\":" << ++eval_event_seq_
    << ",\"mono_ns\":" << evalNowNs() << ",\"ros_s\":" << ros::Time::now().toSec()
    << ",\"fsm\":\"" << fd_->state_str_[int(state_)] << "\",\"plan_result\":" << eval_last_plan_;
  if (!fields.empty()) s << ',' << fields;
  s << '}';
  std_msgs::String msg;
  msg.data = s.str();
  eval_pub_.publish(msg);
}

void FastExplorationFSM::evaluationActions(
    const std::vector<racer_eval::TxDelayGate<exploration_manager::DroneState>::Action>& actions) {
  for (const auto& a : actions) {
    if (eval_gate_.wait_for_arm && (a.kind == "scheduled" || a.kind == "skip")) eval_gate_.armed = false;
    // Actual publish timestamps are measured here, not when a timer was planned.
    const auto before = evalNowNs();
    const auto ros_before = ros::Time::now().toSec();
    if (a.kind == "release" || a.kind == "forward") drone_state_pub_.publish(a.message);
    const auto after = evalNowNs();
    const bool published = a.kind == "release" || a.kind == "forward";
    const auto delay = before - a.generated_ns;
    std::ostringstream s;
    s << std::setprecision(17) << "\"action\":\"" << a.kind << "\",\"reason\":\"" << a.reason
      << "\",\"channel\":\"drone_state\",\"seq\":" << a.seq
      << ",\"generated_ns\":" << a.generated_ns << ",\"publish_before_ns\":" << (published ? std::to_string(before) : "null")
      << ",\"publish_after_ns\":" << (published ? std::to_string(after) : "null")
      << ",\"publish_ros_s\":" << (published ? number(ros_before) : "null")
      << ",\"message_stamp_s\":" << a.message.stamp
      << ",\"planned_delay_ns\":" << a.planned_delay_ns << ",\"period_ns\":" << a.period_ns
      << ",\"budget_ns\":" << a.budget_ns
      << ",\"actual_delay_ns\":" << (a.kind == "release" ? std::to_string(delay) : "null")
      << ",\"actual_injected\":" << ((a.kind == "release" && delay > 0) ? "true" : "false")
      << ",\"budget_ok\":" << ((a.kind != "release" || after - a.generated_ns < a.budget_ns) ? "true" : "false");
    emitEvaluation("tx", s.str());
  }
}

void FastExplorationFSM::evaluationSend(const exploration_manager::DroneState& msg) {
  const auto now = evalNowNs();
  if (eval_gate_.wait_for_arm && eval_gate_.armed && now > eval_rule_expiry_ns_) {
    eval_gate_.armed = false;
    emitEvaluation("rule_arm_expired", "\"reason\":\"expired_before_source_opportunity\"");
  }
  const double elapsed = eval_started_ ? (now - eval_start_ns_) * 1e-9 : 0;
  const bool active = state_ == PLAN_TRAJ || state_ == PUB_TRAJ || state_ == EXEC_TRAJ;
  evaluationActions(eval_gate_.message(msg, now, eval_started_, active, elapsed,
      int64_t((eval_run_budget_s_ - elapsed) * 1e9),
      int64_t((ros::Time::now().toSec() - msg.stamp) * 1e9)));
}

void FastExplorationFSM::evaluationSnapshot() {
  if (!fd_->have_odom_) return;
  auto map = expl_manager_->sdf_map_;
  const bool known = map->getOccupancy(fd_->odom_pos_) != SDFMap::UNKNOWN &&
                     map->getOccupancy(fd_->odom_pos_) != -1;
  std::vector<int> active;
  expl_manager_->hgrid_->getActiveGrids(active);
  std::ostringstream s;
  s << "\"position\":" << vec(fd_->odom_pos_) << ",\"velocity\":" << vec(fd_->odom_vel_)
    << ",\"acceleration\":" << vec(eval_acc_) << ",\"yaw\":" << std::setprecision(17) << fd_->odom_yaw_
    << ",\"position_stamp_s\":" << eval_odom_stamp_s_
    << ",\"position_receipt_ns\":" << eval_odom_receipt_ns_
    << ",\"active_grid_count\":" << active.size()
    << ",\"frontier_count\":" << expl_manager_->ed_->frontiers_.size()
    << ",\"obstacle_distance\":" << (known ? number(map->getDistance(fd_->odom_pos_)) : "null")
    << ",\"map_time_known\":false,\"received_states\":[";
  bool first = true;
  for (size_t i = 0; i < expl_manager_->ed_->swarm_state_.size(); ++i) {
    if (int(i + 1) == getId()) continue;
    const auto& st = expl_manager_->ed_->swarm_state_[i];
    if (!(st.stamp_ > 0)) continue;
    if (!first) s << ',';
    first = false;
    s << "{\"id\":" << i + 1 << ",\"stamp_s\":" << st.stamp_
      << ",\"position\":" << vec(st.pos_) << ",\"velocity\":" << vec(st.vel_)
      << ",\"yaw\":" << st.yaw_ << '}';
  }
  s << ']';
  emitEvaluation("snapshot", s.str());
}

void FastExplorationFSM::intensityPublished(const bspline::Bspline& msg,const std::string& kind,uint64_t seq){
  std::ostringstream s;s<<std::setprecision(17)<<"\"sender_id\":"<<getId()<<",\"seq\":"<<seq
    <<",\"traj_id\":"<<msg.traj_id<<",\"start_s\":"<<msg.start_time.toSec()<<",\"order\":"<<msg.order<<",\"knots\":[";
  for(size_t i=0;i<msg.knots.size();++i){if(i)s<<',';s<<msg.knots[i];}s<<"],\"pos_pts\":[";
  for(size_t i=0;i<msg.pos_pts.size();++i){if(i)s<<',';s<<'['<<msg.pos_pts[i].x<<','<<msg.pos_pts[i].y<<','<<msg.pos_pts[i].z<<']';}
  s<<']';emitEvaluation(kind,s.str());
}
void FastExplorationFSM::intensityActions(const std::vector<racer_eval::IntensityDelayGate<bspline::Bspline>::Action>& actions){
  for(const auto& a:actions){
    const auto before=evalNowNs();const bool sent=a.kind=="forward"||a.kind=="release";
    uint32_t length=ros::serialization::serializationLength(a.message);std::vector<uint8_t> bytes(length);
    ros::serialization::OStream stream(bytes.data(),length);ros::serialization::serialize(stream,a.message);
    uint64_t hash=14695981039346656037ULL;for(auto b:bytes){hash^=b;hash*=1099511628211ULL;}
    if(sent)swarm_traj_pub_.publish(a.message);
    const auto after=evalNowNs();
    std::ostringstream s;s<<"\"action\":\""<<a.kind<<"\",\"reason\":\""<<a.reason<<"\",\"seq\":"<<a.seq
      <<",\"generated_ns\":"<<a.generated_ns<<",\"publish_before_ns\":"<<(sent?std::to_string(before):"null")
      <<",\"publish_after_ns\":"<<(sent?std::to_string(after):"null")<<",\"payload_fnv1a64\":\""<<hash<<"\""
      <<",\"selected\":"<<(a.selected?"true":"false")<<",\"planned_delay_ns\":"<<a.target_ns
      <<",\"decision_ns\":"<<a.decision_ns<<",\"actual_delay_ns\":"<<(a.kind=="release"?std::to_string(before-a.generated_ns):"null")
      <<",\"traj_id\":"<<a.message.traj_id;
    emitEvaluation("intensity_tx",s.str());
    if(sent)intensityPublished(a.message,"published_broadcast_trajectory",a.seq);
    if(accuracy_enabled_&&a.kind=="release"&&accuracy_gate_.state()==racer_eval::AccuracyWindowGate::IN_FLIGHT){
      accuracyEvent(a.reason=="task_end_flush"?"cancelled":"completed",before,a.reason);accuracy_gate_.reset();
    }
  }
}
void FastExplorationFSM::intensitySend(const bspline::Bspline& msg,bool new_trajectory){
  if(!intensity_enabled_){swarm_traj_pub_.publish(msg);return;}
  const auto now=evalNowNs();const bool active=state_==PLAN_TRAJ||state_==PUB_TRAJ||state_==EXEC_TRAJ;
  if(accuracy_enabled_){
    if(intensity_gate_.enabled){
      std::ostringstream s;s<<"\"seq\":"<<intensity_gate_.next_seq()<<",\"generated_ns\":"<<now
        <<",\"new_trajectory\":"<<(new_trajectory?"true":"false")
        <<",\"eligible\":"<<(eval_started_&&active?"true":"false")<<",\"traj_id\":"<<msg.traj_id;
      emitEvaluation("accuracy_publication",s.str());
    }
    auto event=accuracy_gate_.message(now,intensity_gate_.next_seq(),new_trajectory,
        intensity_gate_.enabled&&eval_started_&&active,
        intensity_decision_ns_>0&&now<=intensity_expires_ns_,intensity_decision_ns_,intensity_expires_ns_);
    if(event.triggered)accuracyEvent("triggered",now);
    if(event.selected)accuracyEvent("selected",now);
    const auto decision=accuracy_gate_.current().decision_ns;
    intensityActions(intensity_gate_.message(msg,now,event.selected,decision));
    return;
  }
  const bool select=new_trajectory&&intensity_gate_.enabled&&eval_started_&&active&&now<=intensity_expires_ns_;
  intensityActions(intensity_gate_.message(msg,now,select,intensity_decision_ns_));
}
void FastExplorationFSM::accuracyEvent(const std::string& action,int64_t now,const std::string& reason){
  const auto& w=accuracy_gate_.current();std::ostringstream s;
  s<<"\"action\":\""<<action<<"\",\"reason\":\""<<reason<<"\",\"window_id\":"<<w.id
   <<",\"trigger_ns\":"<<w.trigger_ns<<",\"decision_ns\":"<<w.decision_ns<<",\"expires_ns\":"<<w.expires_ns
   <<",\"wait_ns\":"<<w.wait_ns<<",\"actual_wait_ns\":"<<(now-w.trigger_ns)
   <<",\"original_seq\":"<<w.original_seq<<",\"selected_seq\":"<<w.selected_seq
   <<",\"skipped_original_window\":"<<(w.wait_ns>0?"true":"false")
   <<",\"action_ns\":"<<now<<",\"rule_recheck_required_at_selection\":false";
  emitEvaluation("accuracy_window",s.str());
}
void FastExplorationFSM::accuracyCancel(const std::string& reason){
  if(accuracy_enabled_&&accuracy_gate_.state()!=racer_eval::AccuracyWindowGate::MONITORING){
    accuracyEvent("cancelled",evalNowNs(),reason);accuracy_gate_.reset();
  }
}
}  // namespace fast_planner
