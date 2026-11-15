#ifndef _FAST_EXPLORATION_FSM_H_
#define _FAST_EXPLORATION_FSM_H_

#include <Eigen/Eigen>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <exploration_manager/DroneState.h>
#include <exploration_manager/PairOpt.h>
#include <exploration_manager/PairOptResponse.h>
#include <bspline/Bspline.h>
#include <std_msgs/String.h>
#include <exploration_manager/tx_delay_gate.h>
#include <exploration_manager/intensity_delay_gate.h>
#include <exploration_manager/accuracy_window_gate.h>

#include <algorithm>
#include <iostream>
#include <vector>
#include <memory>
#include <string>
#include <thread>

using Eigen::Vector3d;
using std::shared_ptr;
using std::string;
using std::unique_ptr;
using std::vector;

namespace fast_planner {
class FastPlannerManager;
class FastExplorationManager;
class PlanningVisualization;
struct FSMParam;
struct FSMData;

enum EXPL_STATE { INIT, WAIT_TRIGGER, PLAN_TRAJ, PUB_TRAJ, EXEC_TRAJ, FINISH, IDLE };

class FastExplorationFSM {

public:
  FastExplorationFSM(/* args */) {
  }
  ~FastExplorationFSM() {
  }

  void init(ros::NodeHandle& nh);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  /* helper functions */
  int callExplorationPlanner();
  void transitState(EXPL_STATE new_state, string pos_call);
  void visualize(int content);
  void clearVisMarker();
  int getId();
  void findUnallocated(const vector<int>& actives, vector<int>& missed);

  /* ROS functions */
  void FSMCallback(const ros::TimerEvent& e);
  void safetyCallback(const ros::TimerEvent& e);
  void frontierCallback(const ros::TimerEvent& e);
  void triggerCallback(const geometry_msgs::PoseStampedConstPtr& msg);
  void odometryCallback(const nav_msgs::OdometryConstPtr& msg);

  // Swarm
  void droneStateTimerCallback(const ros::TimerEvent& e);
  void droneStateMsgCallback(const exploration_manager::DroneStateConstPtr& msg);
  void optTimerCallback(const ros::TimerEvent& e);
  void optMsgCallback(const exploration_manager::PairOptConstPtr& msg);
  void optResMsgCallback(const exploration_manager::PairOptResponseConstPtr& msg);
  void swarmTrajCallback(const bspline::BsplineConstPtr& msg);
  void swarmTrajTimerCallback(const ros::TimerEvent& e);

  /* planning utils */
  shared_ptr<FastPlannerManager> planner_manager_;
  shared_ptr<FastExplorationManager> expl_manager_;
  shared_ptr<PlanningVisualization> visualization_;

  shared_ptr<FSMParam> fp_;
  shared_ptr<FSMData> fd_;
  EXPL_STATE state_;

  // Opt-in isolated simulation diagnostics; disabled by all existing launches.
  bool eval_enabled_ = false, eval_started_ = false;
  int eval_last_plan_ = -1;
  int64_t eval_start_ns_ = 0;
  int64_t eval_odom_receipt_ns_ = 0;
  uint64_t eval_event_seq_ = 0;
  double eval_odom_stamp_s_ = 0;
  double eval_run_budget_s_ = 250.0;
  Eigen::Vector3d eval_acc_ = Eigen::Vector3d::Zero();
  ros::Publisher eval_pub_;
  ros::WallTimer eval_tx_timer_;
  ros::Subscriber eval_rule_sub_;
  bool eval_rule_decided_ = false;
  int64_t eval_rule_expiry_ns_ = 0;
  racer_eval::TxDelayGate<exploration_manager::DroneState> eval_gate_;
  bool intensity_enabled_ = false;
  bool accuracy_enabled_ = false;
  racer_eval::AccuracyWindowGate accuracy_gate_;
  void accuracyEvent(const std::string& action, int64_t now, const std::string& reason = "");
  void accuracyCancel(const std::string& reason);
  int64_t intensity_decision_ns_ = 0, intensity_expires_ns_ = 0;
  ros::Subscriber intensity_arm_sub_, intensity_flush_sub_;
  racer_eval::IntensityDelayGate<bspline::Bspline> intensity_gate_;
  void intensitySend(const bspline::Bspline& msg, bool new_trajectory = false);
  void intensityActions(const std::vector<racer_eval::IntensityDelayGate<bspline::Bspline>::Action>& actions);
  void intensityPublished(const bspline::Bspline& msg, const std::string& kind, uint64_t seq);
  static int64_t evalNowNs();
  void initEvaluation(ros::NodeHandle& nh);
  void emitEvaluation(const std::string& kind, const std::string& fields = "");
  void evaluationSnapshot();
  void evaluationSend(const exploration_manager::DroneState& msg);
  void evaluationActions(const std::vector<racer_eval::TxDelayGate<exploration_manager::DroneState>::Action>& actions);

  /* ROS utils */
  ros::NodeHandle node_;
  ros::Timer exec_timer_, safety_timer_, vis_timer_, frontier_timer_;
  ros::Subscriber trigger_sub_, odom_sub_;
  ros::Publisher replan_pub_, new_pub_, bspline_pub_;

  // Swarm state
  ros::Publisher drone_state_pub_, opt_pub_, opt_res_pub_, swarm_traj_pub_, grid_tour_pub_,
      hgrid_pub_;
  ros::Subscriber drone_state_sub_, opt_sub_, opt_res_sub_, swarm_traj_sub_;
  ros::Timer drone_state_timer_, opt_timer_, swarm_traj_timer_;
};

}  // namespace fast_planner

#endif
