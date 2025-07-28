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

#include <algorithm>
#include <iostream>
#include <vector>
#include <memory>
#include <string>
#include <thread>

#include <numeric>  //添加
#include <math.h>
#include <exploration_manager/island_finder.h>
#include <mutex>
#include <limits>

// inspect
#include <exploration_manager/IslandBox.h>
#include <exploration_manager/IslandArray.h>
#include <exploration_manager/CamSwitch.h>

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

enum EXPL_STATE { INIT, WAIT_TRIGGER, OPT, PLAN_TRAJ, PUB_TRAJ, EXEC_TRAJ, FINISH, IDLE };

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
  int callCoveragePlanner();
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
  void callOptEndChecker();

  /* planning utils */
  shared_ptr<FastPlannerManager> planner_manager_;
  shared_ptr<FastExplorationManager> expl_manager_;
  shared_ptr<PlanningVisualization> visualization_;

  shared_ptr<IslandFinder> ifinder_;

  shared_ptr<FSMParam> fp_;
  shared_ptr<FSMData> fd_;
  EXPL_STATE state_;

  /* ROS utils */
  ros::NodeHandle node_;
  ros::Timer exec_timer_, safety_timer_, vis_timer_, frontier_timer_;
  ros::Subscriber trigger_sub_, odom_sub_;
  ros::Publisher replan_pub_, new_pub_, bspline_pub_;

  // Swarm state
  ros::Publisher drone_state_pub_, opt_pub_, opt_res_pub_, swarm_traj_pub_, grid_tour_pub_, hgrid_pub_;
  ros::Subscriber drone_state_sub_, opt_sub_, opt_res_sub_, swarm_traj_sub_;
  ros::Timer drone_state_timer_, opt_timer_, swarm_traj_timer_, island_timer;

  bool init_;

  //testhigh
  ros::Publisher cam_switch_pub_;

  // inspect
  ros::Publisher island_pub_;
  ros::Subscriber island_sub_;
  struct IslandMetadata {
    double volume = 0.0;           // 体积，用于衡量大小和变化
    double last_update_time = 0.0; // 上次更新的时间戳
    bool is_new = true;            // 是否是新发现的
  };

  bool have_island_;
  int inpsct_planner_type_ = -1;
  int current_inspection_island_id_ = -1; // 正在检视的当前岛屿ID，-1 表示空闲
  std::set<int> completed_island_ids_;    // 已完成检视的岛屿ID集合

  //std::map<int, vector<Eigen::Vector3d>> island_buffer_;
  std::mutex island_buffer_mutex_;
  std::map<int, IslandMetadata> island_metadata_;
  std::mutex metadata_mutex_; 
  

  double allocation_bar; 
  bool islands_need_merge;

  void islandMsgCallback(const exploration_manager::IslandArrayConstPtr& msg);
  void islandTimerCallback(const ros::TimerEvent& e);
  int calculateGlobalBestTarget();
  int callInspectPlanner();
  bool checkIslandFinished();
  int callLocalIslandUpdater(Eigen::Vector3d& growth_vector);
  bool updateLocalDisqueue();

  void switchDroneType(const int type); //test
  bool disqueue_replan;

  bool need_opt;
};



}  // namespace fast_planner

#endif