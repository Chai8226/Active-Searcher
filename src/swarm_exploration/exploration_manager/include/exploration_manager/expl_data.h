#ifndef _EXPL_DATA_H_
#define _EXPL_DATA_H_

#include <Eigen/Eigen>
#include <vector>
#include <bspline/Bspline.h>
//#include <unordered_set>
#include <queue>
#include <map>

#include <exploration_manager/island_finder.h>

using Eigen::Vector3d;
using std::vector;

namespace fast_planner {
struct FSMData {
  // FSM data
  bool trigger_, have_odom_, static_state_;
  vector<string> state_str_;

  Eigen::Vector3d odom_pos_, odom_vel_;  // odometry state
  Eigen::Quaterniond odom_orient_;
  double odom_yaw_;

  Eigen::Vector3d start_pt_, start_vel_, start_acc_, start_yaw_;  // start state
  vector<Eigen::Vector3d> start_poss;
  bspline::Bspline newest_traj_;

  // Swarm collision avoidance
  bool avoid_collision_, go_back_;
  ros::Time fsm_init_time_;
  ros::Time last_check_frontier_time_;

  Eigen::Vector3d start_pos_;
};

struct FSMParam {
  double replan_thresh1_;
  double replan_thresh2_;
  double replan_thresh3_;
  double replan_time_;  // second

  // Swarm
  double attempt_interval_;   // Min interval of opt attempt
  double pair_opt_interval_;  // Min interval of successful pair opt
  int repeat_send_num_;

  // inspect
  int drone_type_;  // 0: low, 1: high 
};

struct DroneState {
  int drone_type_;
  Eigen::Vector3d pos_;
  Eigen::Vector3d vel_;
  double yaw_;
  double stamp_;                // Stamp of pos,vel,yaw
  double recent_attempt_time_;  // Stamp of latest opt attempt with any drone

  vector<int> grid_ids_;         // Id of grid tour
  double recent_interact_time_;  // Stamp of latest opt with this drone

  double allocation_cost; 

  map<int, Island> island_buffer;
  
  bool opt;
};

// 新增：定义一个包含位置、速度、偏航角的航点目标状态结构体
struct WaypointTargetState {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d pos;
  Eigen::Vector3d vel;
  double yaw;
};

struct Destination {
  Eigen::Vector3d position;
  int type;  // 0:high, 1:low
  int id;    // grid id or island id
};


struct ExplorationData {
  vector<vector<Vector3d>> frontiers_;
  vector<vector<Vector3d>> dead_frontiers_;
  vector<pair<Vector3d, Vector3d>> frontier_boxes_;
  vector<Vector3d> points_;
  vector<Vector3d> averages_;
  vector<Vector3d> views_;
  vector<double> yaws_;
  vector<Vector3d> frontier_tour_;
  vector<vector<Vector3d>> other_tours_;

  vector<int> refined_ids_;
  vector<vector<Vector3d>> n_points_;
  vector<Vector3d> unrefined_points_;
  vector<Vector3d> refined_points_;
  vector<Vector3d> refined_views_;  // points + dir(yaw)
  vector<Vector3d> refined_views1_, refined_views2_;
  vector<Vector3d> refined_tour_;

  Vector3d next_goal_;
  vector<Vector3d> path_next_goal_, kino_path_;
  Vector3d next_pos_;
  double next_yaw_;

  // viewpoint planning
  // vector<Vector4d> views_;
  vector<Vector3d> views_vis1_, views_vis2_;
  vector<Vector3d> centers_, scales_;

  // Swarm, other drones' state
  vector<DroneState> swarm_state_;
  vector<double> pair_opt_stamps_, pair_opt_res_stamps_;
  vector<int> ego_ids_, other_ids_;
  double pair_opt_stamp_;
  bool reallocated_, wait_response_;

  // Coverage planning
  vector<Vector3d> grid_tour_, grid_tour2_;
  // int prev_first_id_;
  vector<int> last_grid_ids_;

  int plan_num_;

  //修改 用于无frontier全覆盖的位置确认
  //std::unordered_set<int> visited_grid_ids; 
  double allocation_cost, other_allocation_cost; 
  std::vector<WaypointTargetState> dis_queue;
  // std::vector<Vector3d> dis_queue;
  std::unordered_map<int, int> visited_grid_map;
  // vector<vector<Eigen::Vector3d>> islands; 

  double ground_height_;

  Eigen::Vector3d growth_vector_;
};

struct ExplorationParam {
  // params
  bool refine_local_;
  int refined_num_;
  double refined_radius_;
  int top_view_num_;
  double max_decay_;
  string tsp_dir_;   // resource dir of tsp solver
  string mtsp_dir_;  // resource dir of tsp solver
  double relax_time_;
  int init_plan_num_;
  double height_; //用来保持高度

  // Swarm
  int drone_num_;
  int drone_id_;
};

}  // namespace fast_planner

#endif