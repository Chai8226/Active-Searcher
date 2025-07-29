// #include <fstream>
#include <exploration_manager/fast_exploration_manager.h>
#include <thread>
#include <iostream>
#include <fstream>
#include <active_perception/graph_node.h>
#include <active_perception/graph_search.h>
#include <active_perception/perception_utils.h>
#include <active_perception/frontier_finder.h>
// #include <active_perception/uniform_grid.h>
#include <active_perception/hgrid.h>
#include <plan_env/raycast.h>
#include <plan_env/sdf_map.h>
#include <plan_env/edt_environment.h>
#include <plan_manage/planner_manager.h>
// #include <lkh_tsp_solver/lkh_interface.h>
// #include <lkh_mtsp_solver/lkh3_interface.h>
#include <lkh_tsp_solver/SolveTSP.h>
#include <lkh_mtsp_solver/SolveMTSP.h>

#include <exploration_manager/expl_data.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <visualization_msgs/Marker.h>

using namespace Eigen;

namespace fast_planner {
// SECTION interfaces for setup and query

FastExplorationManager::FastExplorationManager() {
}

FastExplorationManager::~FastExplorationManager() {
  ViewNode::astar_.reset();
  ViewNode::caster_.reset();
  ViewNode::map_.reset();
}

void FastExplorationManager::initialize(ros::NodeHandle& nh) {
  planner_manager_.reset(new FastPlannerManager);
  planner_manager_->initPlanModules(nh);

  edt_environment_ = planner_manager_->edt_environment_;
  sdf_map_ = edt_environment_->sdf_map_;
  frontier_finder_.reset(new FrontierFinder(edt_environment_, nh));
  // uniform_grid_.reset(new UniformGrid(edt_environment_, nh));
  hgrid_.reset(new HGrid(edt_environment_, nh));
  // view_finder_.reset(new ViewFinder(edt_environment_, nh));

  ed_.reset(new ExplorationData);
  ep_.reset(new ExplorationParam);

  nh.param("exploration/refine_local", ep_->refine_local_, true);
  nh.param("exploration/refined_num", ep_->refined_num_, -1);
  nh.param("exploration/refined_radius", ep_->refined_radius_, -1.0);
  nh.param("exploration/top_view_num", ep_->top_view_num_, -1);
  nh.param("exploration/max_decay", ep_->max_decay_, -1.0);
  nh.param("exploration/tsp_dir", ep_->tsp_dir_, string("null"));
  nh.param("exploration/mtsp_dir", ep_->mtsp_dir_, string("null"));
  nh.param("exploration/relax_time", ep_->relax_time_, 1.0);
  nh.param("exploration/drone_num", ep_->drone_num_, 1);
  nh.param("exploration/drone_id", ep_->drone_id_, 1);
  nh.param("exploration/init_plan_num", ep_->init_plan_num_, 2);
  nh.param("exploration/init_z", ep_->height_, 3.9);  // 保持高度不变

  ed_->swarm_state_.resize(ep_->drone_num_);
  ed_->pair_opt_stamps_.resize(ep_->drone_num_);
  ed_->pair_opt_res_stamps_.resize(ep_->drone_num_);
  for (int i = 0; i < ep_->drone_num_; ++i) {
    ed_->swarm_state_[i].stamp_ = 0.0;
    ed_->swarm_state_[i].opt = false;
    ed_->pair_opt_stamps_[i] = 0.0;
    ed_->pair_opt_res_stamps_[i] = 0.0;
  }
  planner_manager_->swarm_traj_data_.init(ep_->drone_id_, ep_->drone_num_);

  nh.param("exploration/vm", ViewNode::vm_, -1.0);
  nh.param("exploration/am", ViewNode::am_, -1.0);
  nh.param("exploration/yd", ViewNode::yd_, -1.0);
  nh.param("exploration/ydd", ViewNode::ydd_, -1.0);
  nh.param("exploration/w_dir", ViewNode::w_dir_, -1.0);

  ViewNode::astar_.reset(new Astar);
  ViewNode::astar_->init(nh, edt_environment_);
  ViewNode::map_ = sdf_map_;

  double resolution_ = sdf_map_->getResolution();
  Eigen::Vector3d origin, size;
  sdf_map_->getRegion(origin, size);
  ViewNode::caster_.reset(new RayCaster);
  ViewNode::caster_->setParams(resolution_, origin);

  planner_manager_->path_finder_->lambda_heu_ = 1.0;
  // planner_manager_->path_finder_->max_search_time_ = 0.05;
  planner_manager_->path_finder_->max_search_time_ = 1.0;

  tsp_client_ =
      nh.serviceClient<lkh_mtsp_solver::SolveMTSP>("/solve_tsp_" + to_string(ep_->drone_id_), true);
  acvrp_client_ = nh.serviceClient<lkh_mtsp_solver::SolveMTSP>(
      "/solve_acvrp_" + to_string(ep_->drone_id_), true);

  // Swarm
  for (auto& state : ed_->swarm_state_) {
    state.stamp_ = 0.0;
    state.recent_interact_time_ = 0.0;
    state.recent_attempt_time_ = 0.0;
    state.allocation_cost = -1;
  }
  ed_->last_grid_ids_ = {};
  ed_->reallocated_ = true;
  ed_->pair_opt_stamp_ = 0.0;
  ed_->wait_response_ = false;
  ed_->plan_num_ = 0;
  ed_->ground_height_ = 0;

  // Analysis
  // ofstream fout;
  // fout.open("/home/boboyu/Desktop/RAL_Time/frontier.txt");
  // fout.close();
}


/// @brief 根据所有无人机的当前位置，更新visited_grid_map
/// @param cur_pos 当前无人机的位置
/// @return 是否更新了本地的visited
bool FastExplorationManager::updateVisitedGrids(const Eigen::Vector3d& cur_pos) {
  bool local_flag = false;
  const auto& states = ed_->swarm_state_;
  auto& visited = ed_->visited_grid_map;
  vector<int> all_grid_ids;
  hgrid_->getActiveGrids(all_grid_ids);
  for (const auto& id : all_grid_ids) {
    if ((hgrid_->getCenter(id) - cur_pos).norm() < 0.75){
      visited[id] = 1;
      local_flag = true;
    }  
    else {
      for (int i = 0; i < states.size(); ++i) {
        if (ep_->drone_id_ == i + 1) continue;
        if ((hgrid_->getCenter(id) - states[i].pos_).norm() < 0.75) {
          visited[id] = 1;
        }
      }
    }
  }

  return local_flag;
}

void FastExplorationManager::getVisitedGrids(vector<int>& grid_ids) {
  vector<int> all_grid_ids;
  hgrid_->getActiveGrids(all_grid_ids);
  for (const auto& id : all_grid_ids) {
    if (ed_->visited_grid_map.find(id) == ed_->visited_grid_map.end()) continue;
    grid_ids.push_back(id);
  }
}

// /**
//  * @brief 根据swarm中所有没有被覆盖过的grid_ids规划一次访问顺序，并为每个航点计算目标状态（位置、速度、yaw）。
//  * @param pos           无人机当前位置
//  * @param growth_vector (可选) 区域增长向量
//  */
// void FastExplorationManager::planDisQueue(const Vector3d& pos, const Vector3d growth_vector) {
//   const Vector3d vel = Vector3d(0, 0, 0); // 假设起始速度为0，因为这是重新规划
//   vector<int> grid_ids;
//   vector<vector<int>> other_ids;

//   // 1. 像原来一样，找到需要访问的 grid 中心点的最优序列
//   findCoverageTourOfGrid({ pos }, { vel }, grid_ids, other_ids, true, growth_vector);

//   std::cout << "[planDisQueue] :" << grid_ids.size() << " grid centers to plan" << std::endl;

//   auto& dis_queue = ed_->dis_queue;
//   dis_queue.clear();

//   if (grid_ids.size() == 0) {
//       return;
//   }

//   // 2. 生成包含中间过渡点的详细航点列表
//   vector<Vector3d> detailed_waypoints;
//   for (int i = 0; i < grid_ids.size(); ++i) {
//       auto center = hgrid_->getCenter(grid_ids[i]);
//       detailed_waypoints.push_back(Vector3d(center(0), center(1), ep_->height_));

//       if (i < grid_ids.size() - 1) {
//           auto midpos1 = (hgrid_->getCenter(grid_ids[i + 1]) + 2 * hgrid_->getCenter(grid_ids[i])) / 3.0;
//           auto midpos2 = (2 * hgrid_->getCenter(grid_ids[i + 1]) + hgrid_->getCenter(grid_ids[i])) / 3.0;
//           detailed_waypoints.push_back(Vector3d(midpos1(0), midpos1(1), ep_->height_));
//           detailed_waypoints.push_back(Vector3d(midpos2(0), midpos2(1), ep_->height_));
//       }
//   }

//   if (detailed_waypoints.empty()) {
//       return;
//   }

//   // 3. 为每个详细航点计算目标状态（速度和Yaw）
//   const double max_speed = 1.5; // 从 planRapidCoverageMotion 移过来的常量

//   for (size_t i = 0; i < detailed_waypoints.size(); ++i) {
//       WaypointTargetState target_state;
//       target_state.pos = detailed_waypoints[i];

//       Eigen::Vector3d dir_prev = Eigen::Vector3d::Zero();
//       Eigen::Vector3d dir_next = Eigen::Vector3d::Zero();
      
//       // 计算前一段和后一段的方向向量
//       if (i > 0) {
//           dir_prev = (detailed_waypoints[i] - detailed_waypoints[i-1]);
//       } else {
//           dir_prev = (detailed_waypoints[i] - pos); // 第一段，相对于当前位置
//       }
      
//       if (i < detailed_waypoints.size() - 1) {
//           dir_next = (detailed_waypoints[i+1] - detailed_waypoints[i]);
//       }

//       // 计算目标 Yaw
//       // Yaw 朝向下一个航点
//       if (dir_next.norm() > 1e-6) {
//            target_state.yaw = atan2(dir_next.y(), dir_next.x());
//       } else if (dir_prev.norm() > 1e-6) {
//            // 如果是最后一个点，保持前一个航段的Yaw
//            target_state.yaw = atan2(dir_prev.y(), dir_prev.x());
//       } else {
//           // 如果只有一个点，朝向它
//            target_state.yaw = atan2(target_state.pos.y() - pos.y(), target_state.pos.x() - pos.x());
//       }

//       // 计算目标速度
//       // 默认速度为0（在终点或拐角处）
//       target_state.vel = Eigen::Vector3d::Zero(); 
      
//       bool is_corner = false;
//       if (i > 0 && i < detailed_waypoints.size() - 1) {
//           // 使用点积检查是否为拐角，与原逻辑保持一致
//           if (dir_prev.dot(dir_next) < 1e-3) { // 用一个小的阈值代替等于0
//                is_corner = true;
//           }
//       }

//       // 如果不是终点且不是拐角，则设定巡航速度
//       if (i < detailed_waypoints.size() - 1 && !is_corner) {
//           target_state.vel = dir_next.normalized() * max_speed / 3.0 * 2.0;
//       }

//       dis_queue.push_back(target_state);
//   }
// }

/**
 * @brief 根据swarm中所有没有被覆盖过的grid_ids规划一次访问顺序，并为每个航点计算目标状态（位置、速度、yaw）。
 * 修正：移除了在起点和第一个栅格中心之间不必要的中间点，以解决速度变慢的问题。
 * @param pos           无人机当前位置
 * @param growth_vector (可选) 区域增长向量
 */
void FastExplorationManager::planDisQueue(const Vector3d& pos, const Vector3d& growth_vector) {
  const Vector3d vel = Vector3d(0, 0, 0);
  vector<int> grid_ids;
  vector<vector<int>> other_ids;

  findCoverageTourOfGrid({ pos }, { vel }, grid_ids, other_ids, false, growth_vector);

  std::cout << "[planDisQueue] :" << grid_ids.size() << " grid centers to plan" << std::endl;

  auto& dis_queue = ed_->dis_queue;
  dis_queue.clear();

  if (grid_ids.size() == 0) {
      return;
  }

  // 2. 生成包含中间过渡点的详细航点列表
  vector<Vector3d> detailed_waypoints;

  // ======================== 修正点在这里 ========================
  // 不再在起点和第一个栅格中心之间插入中间点。
  // 直接将第一个栅格中心作为第一个目标点。
  // 这样可以让规划器在长的直线段上充分加速。
  // ===============================================================

  // 遍历栅格点，添加栅格中心点及其后的中间点
  for (int i = 0; i < grid_ids.size(); ++i) {
      auto center = hgrid_->getCenter(grid_ids[i]);
      detailed_waypoints.push_back(Vector3d(center(0), center(1), ep_->height_));

      // 只在两个栅格中心之间添加一个中间点，这个点更靠近当前段的起点(center)
      if (i < grid_ids.size() - 1) {
          auto next_center = hgrid_->getCenter(grid_ids[i + 1]);
          auto midpos = (next_center + 2 * center) / 3.0;
          detailed_waypoints.push_back(Vector3d(midpos(0), midpos(1), ep_->height_));
      }
  }

  if (detailed_waypoints.empty()) {
      return;
  }

  // 3. 为每个详细航点计算目标状态（速度和Yaw）
  const double max_speed = 1.5;

  for (size_t i = 0; i < detailed_waypoints.size(); ++i) {
      WaypointTargetState target_state;
      target_state.pos = detailed_waypoints[i];

      Eigen::Vector3d dir_prev = Eigen::Vector3d::Zero();
      Eigen::Vector3d dir_next = Eigen::Vector3d::Zero();
      
      if (i > 0) {
          dir_prev = (detailed_waypoints[i] - detailed_waypoints[i-1]);
      } else {
          dir_prev = (detailed_waypoints[i] - pos);
      }
      
      if (i < detailed_waypoints.size() - 1) {
          dir_next = (detailed_waypoints[i+1] - detailed_waypoints[i]);
      }

      if (dir_next.norm() > 1e-6) {
           target_state.yaw = atan2(dir_next.y(), dir_next.x());
      } else if (dir_prev.norm() > 1e-6) {
           target_state.yaw = atan2(dir_prev.y(), dir_prev.x());
      } else {
           target_state.yaw = atan2(target_state.pos.y() - pos.y(), target_state.pos.x() - pos.x());
      }

      target_state.vel = Eigen::Vector3d::Zero(); 
      
      // ======================== 修正点在这里 ========================
      // 修正了拐角的判断逻辑。
      // 原来的 is_corner 判断 (dir_prev.dot(dir_next) < 1e-3) 过于严格，
      // 只能检测到接近90度或更大的转弯，对于锐角转弯（如45度）会失效，
      // 导致无人机尝试高速通过非直角弯，这是不安全的。
      bool is_corner = false;
      if (i > 0 && i < detailed_waypoints.size() - 1) {
          // 使用归一化向量的点积（即夹角余弦）来判断。
          // 如果向量方向变化不大（夹角小，余弦值接近1），则不认为是拐角。
          // 如果方向变化明显（夹角大，余弦值小于一个阈值），则认为是拐角。
          if (dir_prev.norm() > 1e-6 && dir_next.norm() > 1e-6) {
              // 如果夹角余弦小于0.985 (约等于10度)，就认为是需要减速的拐角。
              if (dir_prev.normalized().dot(dir_next.normalized()) < 0.985) {
                  is_corner = true;
              }
          }
      }
      // ===============================================================

      // 如果不是终点且不是拐角，则设定巡航速度
      if (i < detailed_waypoints.size() - 1 && !is_corner) {
          target_state.vel = dir_next.normalized() * max_speed / 3.0 * 2.0;
      }

      dis_queue.push_back(target_state);
  }
}

// /**
//  * @brief 根据swarm中的所有没有被覆盖过的grid_ids规划一次访问顺序，转存在dis_queque队列之中，同时加入中间点来矫正轨迹
//  */
// void FastExplorationManager::planDisQueue(const Vector3d& pos, const Vector3d growth_vector = Vector3d(0, 0, 0)) {
//   // const auto& grid_ids = ed_->swarm_state_[ep_->drone_id_ - 1].grid_ids_;
//   const Vector3d vel = Vector3d(0, 0, 0);
//   vector<int> grid_ids;
//   vector<vector<int>> other_ids;
//   // static bool init = true;
//   //  findGlobalTour(pos, vel, yaw, indices);

//   // findGlobalTourOfGrid({ pos }, { vel }, grid_ids, other_ids, true);
//   findCoverageTourOfGrid({ pos }, { vel }, grid_ids, other_ids, true, growth_vector);
//   // init = false;

//   std::cout << "[planDisQueue] :" << grid_ids.size() << " grid centers to plan" << std::endl;

//   if (grid_ids.size() == 0) return;
//   auto& dis_queque = ed_->dis_queue;
//   dis_queque.clear();

//   for (int i = 0; i < grid_ids.size(); i++) {
//     auto center = hgrid_->getCenter(grid_ids[i]);
//     dis_queque.push_back(Vector3d(center(0), center(1), ep_->height_));
//     if (i < grid_ids.size() - 1) {
//       auto midpos1 = (hgrid_->getCenter(grid_ids[i + 1]) + 2 * hgrid_->getCenter(grid_ids[i])) /
//                      3.0;  // 三分点
//       auto midpos2 =
//           (2 * hgrid_->getCenter(grid_ids[i + 1]) + hgrid_->getCenter(grid_ids[i])) / 3.0;
//       dis_queque.push_back(Vector3d(midpos1(0), midpos1(1), ep_->height_));
//       dis_queque.push_back(Vector3d(midpos2(0), midpos2(1), ep_->height_));

//       // auto next_center = hgrid_->getCenter(grid_ids[i + 1]);  // 两分点
//       // auto midpos1 = (next_center + center) * 0.5;
//       // if(i > 0){
//       //   auto last_pos = hgrid_->getCenter(grid_ids[i - 1]);
//       //   auto dir1 = center - last_pos;
//       //   auto dir2 = next_center - center;
//       //   if(dir1.dot(dir2) == 0){ //转角
//       //     dis_queque.push_back(center + dir2.normalized() * 0.15);
//       //   }
//       // }
//       // dis_queque.push_back(Vector3d(midpos1(0), midpos1(1), ep_->height_));
//     }
//   }
// }

// int FastExplorationManager::planRapidCoverageMotion(const Vector3d& start_pos_, const Vector3d& pos,
//     const Vector3d& vel, const Vector3d& acc, const Vector3d& yaw) {
//   ros::Time t1 = ros::Time::now();
//   auto t2 = t1;

//   std::cout << "start pos: " << pos.transpose() << ", vel: " << vel.transpose()
//             << ", acc: " << acc.transpose() << std::endl;

//   // Do global and local tour planning and retrieve the next viewpoint

//   // ed_->frontier_tour_.clear();
//   Vector3d next_pos;
//   double next_yaw;
//   const double max_speed = 1.5;
//   Vector3d target_vel;
//   static bool start = true;
//   // Find the tour passing through viewpoints
//   // Optimal tour is returned as indices of frontier
//   // vector<int> grid_ids;
//   // vector<vector<int>> other_ids;
//   // // findGlobalTour(pos, vel, yaw, indices);
//   // findGlobalTourOfGrid({ pos }, { vel }, grid_ids, other_ids, true);
//   auto& grid_ids = ed_->swarm_state_[ep_->drone_id_ - 1].grid_ids_;
//   if (grid_ids.empty()) {

//     return NO_GRID;

//     // No grid is assigned to this drone, but keep moving is necessary
//     // Move to the closest targets
//     ROS_WARN("Empty grid");

//     // 修改 后面可能需要写一下措施
//     next_pos = pos;
//     next_yaw = yaw(0);

//   } else {
//     // 修改 直接用grid center队列实现遍历
//     auto& dis_queue = ed_->dis_queue;
//     if (dis_queue.empty()) {
//       ROS_WARN("id %d: grid tour finished!", ep_->drone_id_);
//       return NO_GRID;
//       ;
//     }

//     next_pos = dis_queue[0] + Eigen::Vector3d(0, 0, ed_->ground_height_);
//     // static bool start = true;
//     static Eigen::Vector3d last_pos;
//     static int initial_size;
//     if (start) {
//       start = false;
//       initial_size = dis_queue.size();
//       Eigen::Vector3d dir = (next_pos - pos);
//       next_yaw = atan2(dir[1], dir[0]);
//     } else if (dis_queue.size() <= 2) {
//       Eigen::Vector3d dir = (next_pos - last_pos);
//       next_yaw = atan2(dir[1], dir[0]);
//       target_vel = Vector3d(0, 0, 0);
//     } else if (dis_queue.size() == initial_size) {  // 处于第一段轨迹之中
//       auto next_next_pos = dis_queue[1];
//       target_vel = Vector3d(0, 0, 0);
//       Eigen::Vector3d dir = (next_next_pos - next_pos);
//       next_yaw = atan2(dir[1], dir[0]);
//     } else {
//       auto next_next_pos = dis_queue[1];
//       Eigen::Vector3d dir = next_next_pos - next_pos;
//       next_yaw = atan2(dir[1], dir[0]);
//       Eigen::Vector3d cur_dir = next_pos - last_pos;
//       if (cur_dir.dot(dir) == 0) {
//         // ROS_WARN("It is a corner!");
//         target_vel = Vector3d(0, 0, 0);
//       } else {
//         target_vel = dir.normalized() * max_speed / 3 * 2;
//       }
//     }
//     last_pos = next_pos;
//     // next_yaw = yaw(0);
//   }

//   std::cout << "Next view: " << next_pos.transpose() << ", " << next_yaw << std::endl;
//   ed_->next_pos_ = next_pos;
//   ed_->next_yaw_ = next_yaw;

//   if (planRapidCoverageTraj(pos, vel, acc, yaw, next_pos, next_yaw, target_vel) == FAIL) {
//     ROS_WARN("planRapidCoverageTraj Fail");
//     return FAIL;
//   }

//   // if (planTrajToView(pos, vel, acc, yaw, next_pos, next_yaw) == FAIL) {
//   //   ROS_WARN("planRapidCoverageTraj Fail");
//   //   return FAIL;
//   // }

//   double total = (ros::Time::now() - t2).toSec();
//   ROS_INFO("Total time: %lf", total);
//   ROS_ERROR_COND(total > 0.1, "Total time too long!!!");
//   return SUCCEED;
// }

int FastExplorationManager::planRapidCoverageMotion(const Vector3d& start_pos_, const Vector3d& pos,
  const Vector3d& vel, const Vector3d& acc, const Vector3d& yaw) {
  ros::Time t1 = ros::Time::now();

  std::cout << "start pos: " << pos.transpose() << ", vel: " << vel.transpose()
            << ", acc: " << acc.transpose() << std::endl;

  auto& dis_queue = ed_->dis_queue;

  // 1. 检查队列是否为空
  if (dis_queue.empty()) {
      auto& grid_ids = ed_->swarm_state_[ep_->drone_id_ - 1].grid_ids_;
      if (grid_ids.empty()) {
           ROS_WARN("Empty grid and empty queue, nothing to do.");
           return NO_GRID;
      } else {
           ROS_WARN("id %d: grid tour finished or queue is not planned!", ep_->drone_id_);
           return NO_GRID;
      }
  }

  // 2. 从队列头部获取预先计算好的目标状态
  // 注意：这里的 .front() 只是获取，并没有移除。
  // 您应该在您的状态机（FSM）中，当无人机接近或到达一个航点后，再将该航点从队列中 pop_front()。
  const auto& next_target = dis_queue.front();
  
  Vector3d next_pos = next_target.pos + Eigen::Vector3d(0, 0, ed_->ground_height_);
  double next_yaw = next_target.yaw;
  Vector3d target_vel = next_target.vel;

  // 更新 exploration data 用于可视化或其他模块
  ed_->next_pos_ = next_pos;
  ed_->next_yaw_ = next_yaw;
  std::cout << "Executing to next view: " << next_pos.transpose() 
            << ", yaw: " << next_yaw 
            << ", target_vel: " << target_vel.transpose() << std::endl;

  // 3. 调用底层轨迹规划器，传入目标状态
  if (planRapidCoverageTraj(pos, vel, acc, yaw, next_pos, next_yaw, target_vel) == FAIL) {
      ROS_WARN("planRapidCoverageTraj Fail");
      return FAIL;
  }
  
  double total = (ros::Time::now() - t1).toSec();
  ROS_INFO("Total time in planRapidCoverageMotion: %lf", total);
  ROS_ERROR_COND(total > 0.1, "Total time too long!!!");
  
  return SUCCEED;
}

int FastExplorationManager::planTrajToView(const Vector3d& pos, const Vector3d& vel,
    const Vector3d& acc, const Vector3d& yaw, const Vector3d& next_pos, const double& next_yaw) {

  // Plan trajectory (position and yaw) to the next viewpoint
  auto t1 = ros::Time::now();

  // Compute time lower bound of yaw and use in trajectory generation
  double diff0 = next_yaw - yaw[0];
  double diff1 = fabs(diff0);
  double time_lb = min(diff1, 2 * M_PI - diff1) / ViewNode::yd_;

  // Generate trajectory of x,y,z
  bool goal_unknown = (edt_environment_->sdf_map_->getOccupancy(next_pos) == SDFMap::UNKNOWN);
  // bool start_unknown = (edt_environment_->sdf_map_->getOccupancy(pos) == SDFMap::UNKNOWN);
  bool optimistic = ed_->plan_num_ < ep_->init_plan_num_;
  planner_manager_->path_finder_->reset();
  if (planner_manager_->path_finder_->search(pos, next_pos, optimistic) != Astar::REACH_END) {
    ROS_ERROR("id: %d from (%lf, %lf, %lf) to (%lf, %lf, %lf): No path to next viewpoint",
        ep_->drone_id_, pos(0), pos(1), pos(2), next_pos(0), next_pos(1), next_pos(2));
    return FAIL;
  }
  ed_->path_next_goal_ = planner_manager_->path_finder_->getPath();
  shortenPath(ed_->path_next_goal_);
  ed_->kino_path_.clear();

  const double radius_far = 7.0;
  const double radius_close = 1.5;
  const double len = Astar::pathLength(ed_->path_next_goal_);
  if (len < radius_close || optimistic) {
    // Next viewpoint is very close, no need to search kinodynamic path, just use waypoints-based
    // optimization
    planner_manager_->planInspectTraj(ed_->path_next_goal_, vel, acc, time_lb);
    ed_->next_goal_ = next_pos;
    // std::cout << "Close goal." << std::endl;
    if (ed_->plan_num_ < ep_->init_plan_num_) {
      ed_->plan_num_++;
      ROS_WARN("init plan.");
    }
  } else if (len > radius_far) {
    // Next viewpoint is far away, select intermediate goal on geometric path (this also deal with
    // dead end)
    std::cout << "Far goal." << std::endl;
    double len2 = 0.0;
    vector<Eigen::Vector3d> truncated_path = { ed_->path_next_goal_.front() };
    for (int i = 1; i < ed_->path_next_goal_.size() && len2 < radius_far; ++i) {
      auto cur_pt = ed_->path_next_goal_[i];
      len2 += (cur_pt - truncated_path.back()).norm();
      truncated_path.push_back(cur_pt);
    }
    ed_->next_goal_ = truncated_path.back();
    planner_manager_->planInspectTraj(truncated_path, vel, acc, time_lb);
  } else {
    // Search kino path to exactly next viewpoint and optimize
    std::cout << "Mid goal" << std::endl;
    ed_->next_goal_ = next_pos;

    if (!planner_manager_->kinodynamicReplan( pos, vel, acc, ed_->next_goal_, Vector3d(0, 0, 0), time_lb)) {
      ROS_ERROR("kinodynamicReplan FAIL!");
      return FAIL;
    }

    ed_->kino_path_ = planner_manager_->kino_path_finder_->getKinoTraj(0.02);
  }

  if (planner_manager_->local_data_.position_traj_.getTimeSum() < time_lb - 0.5)
    ROS_ERROR("Lower bound not satified!");

  double traj_plan_time = (ros::Time::now() - t1).toSec();

  t1 = ros::Time::now();
  planner_manager_->planYawExplore(yaw, next_yaw, true, ep_->relax_time_);
  double yaw_time = (ros::Time::now() - t1).toSec();
  ROS_INFO("Traj: %lf, yaw: %lf", traj_plan_time, yaw_time);

  return SUCCEED;
}

int FastExplorationManager::planRapidCoverageTraj(const Vector3d& pos, const Vector3d& vel,
    const Vector3d& acc, const Vector3d& yaw, const Vector3d& next_pos, const double& next_yaw,
    const Vector3d& target_vel = Vector3d(0, 0, 0)) {

  // Plan trajectory (position and yaw) to the next viewpoint
  auto t1 = ros::Time::now();

  // Compute time lower bound of yaw and use in trajectory generation
  double diff0 = next_yaw - yaw[0];
  double diff1 = fabs(diff0);
  double time_lb = min(diff1, 2 * M_PI - diff1) / ViewNode::yd_;

  // Generate trajectory of x,y,z
  bool goal_unknown = (edt_environment_->sdf_map_->getOccupancy(next_pos) == SDFMap::UNKNOWN);
  // bool start_unknown = (edt_environment_->sdf_map_->getOccupancy(pos) == SDFMap::UNKNOWN);
  bool optimistic = ed_->plan_num_ < ep_->init_plan_num_;
  planner_manager_->path_finder_->reset();
  if (planner_manager_->path_finder_->search(pos, next_pos, optimistic) != Astar::REACH_END) {
    ROS_ERROR("id: %d from (%lf, %lf, %lf) to (%lf, %lf, %lf): No path to next viewpoint",
        ep_->drone_id_, pos(0), pos(1), pos(2), next_pos(0), next_pos(1), next_pos(2));
    return FAIL;
  }
  ed_->path_next_goal_ = planner_manager_->path_finder_->getPath();
  shortenPath(ed_->path_next_goal_);
  ed_->kino_path_.clear();

  const double radius_far = 7.0;
  const double radius_close = 1.5;
  const double len = Astar::pathLength(ed_->path_next_goal_);
  if (len < radius_close || optimistic) {
    // Next viewpoint is very close, no need to search kinodynamic path, just use waypoints-based
    // optimization
    planner_manager_->planCoverageTraj(ed_->path_next_goal_, vel, acc, time_lb, target_vel);
    ed_->next_goal_ = next_pos;
    // std::cout << "Close goal." << std::endl;
    if (ed_->plan_num_ < ep_->init_plan_num_) {
      ed_->plan_num_++;
      ROS_WARN("init plan. id %d", ep_->drone_id_);
    }
  } else if (len > radius_far) {
    // Next viewpoint is far away, select intermediate goal on geometric path (this also deal with
    // dead end)
    std::cout << "Far goal." << std::endl;
    double len2 = 0.0;
    vector<Eigen::Vector3d> truncated_path = { ed_->path_next_goal_.front() };
    for (int i = 1; i < ed_->path_next_goal_.size() && len2 < radius_far; ++i) {
      auto cur_pt = ed_->path_next_goal_[i];
      len2 += (cur_pt - truncated_path.back()).norm();
      truncated_path.push_back(cur_pt);
    }
    ed_->next_goal_ = truncated_path.back();
    planner_manager_->planCoverageTraj(truncated_path, vel, acc, time_lb, target_vel);
  } else {
    // Search kino path to exactly next viewpoint and optimize
    std::cout << "Mid goal" << std::endl;
    ed_->next_goal_ = next_pos;

    if (!planner_manager_->kinodynamicReplan(pos, vel, acc, ed_->next_goal_, target_vel, time_lb)) {
      ROS_ERROR("kinodynamicReplan FAIL!");
      return FAIL;
    }

    ed_->kino_path_ = planner_manager_->kino_path_finder_->getKinoTraj(0.02);
  }

  if (planner_manager_->local_data_.position_traj_.getTimeSum() < time_lb - 0.5)
    ROS_ERROR("Lower bound not satified!");

  double traj_plan_time = (ros::Time::now() - t1).toSec();

  t1 = ros::Time::now();
  planner_manager_->planYawExplore(yaw, next_yaw, true, ep_->relax_time_);
  double yaw_time = (ros::Time::now() - t1).toSec();
  ROS_INFO("Traj: %lf, yaw: %lf", traj_plan_time, yaw_time);

  return SUCCEED;
}

int FastExplorationManager::updateFrontierStruct(const Eigen::Vector3d& pos) {
  auto t1 = ros::Time::now();
  auto t2 = t1;
  ed_->views_.clear();

  // Search frontiers and group them into clusters
  frontier_finder_->searchFrontiers();

  double frontier_time = (ros::Time::now() - t1).toSec();
  t1 = ros::Time::now();

  // Find viewpoints (x,y,z,yaw) for all clusters; find the informative ones
  frontier_finder_->computeFrontiersToVisit();

  // Retrieve the updated info
  frontier_finder_->getFrontiers(ed_->frontiers_);
  frontier_finder_->getDormantFrontiers(ed_->dead_frontiers_);
  frontier_finder_->getFrontierBoxes(ed_->frontier_boxes_);

  frontier_finder_->getTopViewpointsInfo(pos, ed_->points_, ed_->yaws_, ed_->averages_);
  for (int i = 0; i < ed_->points_.size(); ++i)
    ed_->views_.push_back(
        ed_->points_[i] + 2.0 * Vector3d(cos(ed_->yaws_[i]), sin(ed_->yaws_[i]), 0));

  if (ed_->frontiers_.empty()) {
    ROS_WARN("No coverable frontier.");
    return 0;
  }

  double view_time = (ros::Time::now() - t1).toSec();

  t1 = ros::Time::now();
  frontier_finder_->updateFrontierCostMatrix();

  double mat_time = (ros::Time::now() - t1).toSec();
  double total_time = frontier_time + view_time + mat_time;
  // ROS_INFO("Drone %d: frontier t: %lf, viewpoint t: %lf, mat: %lf", ep_->drone_id_, frontier_time,
      // view_time, mat_time);

  // ROS_INFO("Total t: %lf", (ros::Time::now() - t2).toSec());
  return ed_->frontiers_.size();
}

void FastExplorationManager::findGridAndFrontierPath(const Vector3d& cur_pos,
    const Vector3d& cur_vel, const Vector3d& cur_yaw, vector<int>& grid_ids,
    vector<int>& frontier_ids) {
  auto t1 = ros::Time::now();

  // Select nearby drones according to their states' stamp
  vector<Eigen::Vector3d> positions = { cur_pos };
  vector<Eigen::Vector3d> velocities = { cur_vel };
  vector<double> yaws = { cur_yaw[0] };

  // Partitioning-based tour planning
  vector<int> ego_ids;
  vector<vector<int>> other_ids;
  if (!findGlobalTourOfGrid(positions, velocities, ego_ids, other_ids)) {
    grid_ids = {};
    return;
  }
  grid_ids = ego_ids;

  double grid_time = (ros::Time::now() - t1).toSec();

  // Frontier-based single drone tour planning
  // Restrict frontier within the first visited grid
  t1 = ros::Time::now();

  vector<int> ftr_ids;
  // uniform_grid_->getFrontiersInGrid(ego_ids[0], ftr_ids);
  hgrid_->getFrontiersInGrid(ego_ids, ftr_ids);
  ROS_INFO("Find frontier tour, %d involved------------", ftr_ids.size());

  if (ftr_ids.empty()) {
    frontier_ids = {};
    return;
  }

  // Consider next grid in frontier tour planning
  Eigen::Vector3d grid_pos;
  double grid_yaw;
  vector<Eigen::Vector3d> grid_pos_vec;
  if (hgrid_->getNextGrid(ego_ids, grid_pos, grid_yaw)) {
    grid_pos_vec = { grid_pos };
  }

  findTourOfFrontier(cur_pos, cur_vel, cur_yaw, ftr_ids, grid_pos_vec, frontier_ids);
  double ftr_time = (ros::Time::now() - t1).toSec();
  ROS_INFO("Grid tour t: %lf, frontier tour t: %lf.", grid_time, ftr_time);
}

void FastExplorationManager::shortenPath(vector<Vector3d>& path) {
  if (path.empty()) {
    ROS_ERROR("Empty path to shorten");
    return;
  }
  // Shorten the tour, only critical intermediate points are reserved.
  const double dist_thresh = 3.0;
  vector<Vector3d> short_tour = { path.front() };
  for (int i = 1; i < path.size() - 1; ++i) {
    if ((path[i] - short_tour.back()).norm() > dist_thresh)
      short_tour.push_back(path[i]);
    else {
      // Add waypoints to shorten path only to avoid collision
      ViewNode::caster_->input(short_tour.back(), path[i + 1]);
      Eigen::Vector3i idx;
      while (ViewNode::caster_->nextId(idx) && ros::ok()) {
        if (edt_environment_->sdf_map_->getInflateOccupancy(idx) == 1 ||
            edt_environment_->sdf_map_->getOccupancy(idx) == SDFMap::UNKNOWN) {
          short_tour.push_back(path[i]);
          break;
        }
      }
    }
  }
  if ((path.back() - short_tour.back()).norm() > 1e-3) short_tour.push_back(path.back());

  // Ensure at least three points in the path
  if (short_tour.size() == 2)
    short_tour.insert(short_tour.begin() + 1, 0.5 * (short_tour[0] + short_tour[1]));
  path = short_tour;
}

void FastExplorationManager::findGlobalTour(const Vector3d& cur_pos, const Vector3d& cur_vel,
    const Vector3d cur_yaw, vector<int>& indices) {
  auto t1 = ros::Time::now();

  // Get cost matrix for current state and clusters
  Eigen::MatrixXd cost_mat;
  frontier_finder_->getFullCostMatrix(cur_pos, cur_vel, cur_yaw, cost_mat);
  const int dimension = cost_mat.rows();
  std::cout << "mat:   " << cost_mat.rows() << std::endl;

  double mat_time = (ros::Time::now() - t1).toSec();
  t1 = ros::Time::now();

  // Initialize TSP par file
  ofstream par_file(ep_->tsp_dir_ + "/drone_" + to_string(ep_->drone_id_) + ".par");
  par_file << "PROBLEM_FILE = " << ep_->tsp_dir_ + "/drone_" + to_string(ep_->drone_id_) + ".tsp\n";
  par_file << "GAIN23 = NO\n";
  par_file << "OUTPUT_TOUR_FILE ="
           << ep_->tsp_dir_ + "/drone_" + to_string(ep_->drone_id_) +
                  ".tou"
                  "r\n";
  par_file << "RUNS = 1\n";
  par_file.close();

  // Write params and cost matrix to problem file
  ofstream prob_file(ep_->tsp_dir_ + "/drone_" + to_string(ep_->drone_id_) + ".tsp");
  // Problem specification part, follow the format of TSPLIB
  string prob_spec;
  prob_spec = "NAME : single\nTYPE : ATSP\nDIMENSION : " + to_string(dimension) +
              "\nEDGE_WEIGHT_TYPE : "
              "EXPLICIT\nEDGE_WEIGHT_FORMAT : FULL_MATRIX\nEDGE_WEIGHT_SECTION\n";
  prob_file << prob_spec;
  // prob_file << "TYPE : TSP\n";
  // prob_file << "EDGE_WEIGHT_FORMAT : LOWER_ROW\n";
  // Problem data part
  const int scale = 100;
  for (int i = 0; i < dimension; ++i) {
    for (int j = 0; j < dimension; ++j) {
      int int_cost = cost_mat(i, j) * scale;
      prob_file << int_cost << " ";
    }
    prob_file << "\n";
  }
  prob_file << "EOF";
  prob_file.close();

  // solveTSPLKH((ep_->tsp_dir_ + "/drone_" + to_string(ep_->drone_id_) + ".par").c_str());
  lkh_tsp_solver::SolveTSP srv;
  if (!tsp_client_.call(srv)) {
    ROS_ERROR("Fail to solve TSP.");
    return;
  }

  // Read optimal tour from the tour section of result file
  ifstream res_file(ep_->tsp_dir_ + "/drone_" + to_string(ep_->drone_id_) + ".tour");
  string res;
  while (getline(res_file, res)) {
    // Go to tour section
    if (res.compare("TOUR_SECTION") == 0) break;
  }

  // Read path for ATSP formulation
  while (getline(res_file, res)) {
    // Read indices of frontiers in optimal tour
    int id = stoi(res);
    if (id == 1)  // Ignore the current state
      continue;
    if (id == -1) break;
    indices.push_back(id - 2);  // Idx of solver-2 == Idx of frontier
  }

  res_file.close();

  std::cout << "Tour " << ep_->drone_id_ << ": ";
  for (auto id : indices) std::cout << id << ", ";
  std::cout << "" << std::endl;

  // Get the path of optimal tour from path matrix
  frontier_finder_->getPathForTour(cur_pos, indices, ed_->frontier_tour_);

  double tsp_time = (ros::Time::now() - t1).toSec();
  ROS_INFO("Cost mat: %lf, TSP: %lf", mat_time, tsp_time);

  // if (tsp_time > 0.1) ROS_BREAK();
}

void FastExplorationManager::refineLocalTour(const Vector3d& cur_pos, const Vector3d& cur_vel,
    const Vector3d& cur_yaw, const vector<vector<Vector3d>>& n_points,
    const vector<vector<double>>& n_yaws, vector<Vector3d>& refined_pts,
    vector<double>& refined_yaws) {
  double create_time, search_time, parse_time;
  auto t1 = ros::Time::now();

  // Create graph for viewpoints selection
  GraphSearch<ViewNode> g_search;
  vector<ViewNode::Ptr> last_group, cur_group;

  // Add the current state
  ViewNode::Ptr first(new ViewNode(cur_pos, cur_yaw[0]));
  first->vel_ = cur_vel;
  g_search.addNode(first);
  last_group.push_back(first);
  ViewNode::Ptr final_node;

  // Add viewpoints
  std::cout << "Local refine graph size: 1, ";
  for (int i = 0; i < n_points.size(); ++i) {
    // Create nodes for viewpoints of one frontier
    for (int j = 0; j < n_points[i].size(); ++j) {
      ViewNode::Ptr node(new ViewNode(n_points[i][j], n_yaws[i][j]));
      g_search.addNode(node);
      // Connect a node to nodes in last group
      for (auto nd : last_group) g_search.addEdge(nd->id_, node->id_);
      cur_group.push_back(node);

      // Only keep the first viewpoint of the last local frontier
      if (i == n_points.size() - 1) {
        final_node = node;
        break;
      }
    }
    // Store nodes for this group for connecting edges
    std::cout << cur_group.size() << ", ";
    last_group = cur_group;
    cur_group.clear();
  }
  std::cout << "" << std::endl;
  create_time = (ros::Time::now() - t1).toSec();
  t1 = ros::Time::now();

  // Search optimal sequence
  vector<ViewNode::Ptr> path;
  g_search.DijkstraSearch(first->id_, final_node->id_, path);

  search_time = (ros::Time::now() - t1).toSec();
  t1 = ros::Time::now();

  // Return searched sequence
  for (int i = 1; i < path.size(); ++i) {
    refined_pts.push_back(path[i]->pos_);
    refined_yaws.push_back(path[i]->yaw_);
  }

  // Extract optimal local tour (for visualization)
  ed_->refined_tour_.clear();
  ed_->refined_tour_.push_back(cur_pos);
  ViewNode::astar_->lambda_heu_ = 1.0;
  ViewNode::astar_->setResolution(0.2);
  for (auto pt : refined_pts) {
    vector<Vector3d> path;
    if (ViewNode::searchPath(ed_->refined_tour_.back(), pt, path))
      ed_->refined_tour_.insert(ed_->refined_tour_.end(), path.begin(), path.end());
    else
      ed_->refined_tour_.push_back(pt);
  }
  ViewNode::astar_->lambda_heu_ = 10000;

  parse_time = (ros::Time::now() - t1).toSec();
  // ROS_WARN("create: %lf, search: %lf, parse: %lf", create_time, search_time, parse_time);
}

void FastExplorationManager::allocateGrids(const vector<Eigen::Vector3d>& positions,
    const vector<Eigen::Vector3d>& velocities, const vector<vector<int>>& first_ids,
    const vector<vector<int>>& second_ids, const vector<int>& grid_ids, vector<int>& ego_ids,
    vector<int>& other_ids) {
  // ROS_INFO("Allocate grid.");

  auto t1 = ros::Time::now();
  auto t2 = t1;

  if (grid_ids.size() == 1) {  // Only one grid, no need to run ACVRP
    auto pt = hgrid_->getCenter(grid_ids.front());
    // double d1 = (positions[0] - pt).norm();
    // double d2 = (positions[1] - pt).norm();
    vector<Eigen::Vector3d> path;
    double d1 = ViewNode::computeCost(positions[0], pt, 0, 0, Eigen::Vector3d(0, 0, 0), 0, path);
    double d2 = ViewNode::computeCost(positions[1], pt, 0, 0, Eigen::Vector3d(0, 0, 0), 0, path);
    if (d1 < d2) {
      ego_ids = grid_ids;
      other_ids = {};
    } else {
      ego_ids = {};
      other_ids = grid_ids;
    }
    return;
  }

  Eigen::MatrixXd mat;
  // uniform_grid_->getCostMatrix(positions, velocities, prev_first_ids, grid_ids, mat);
  hgrid_->getCostMatrix(positions, velocities, first_ids, second_ids, grid_ids, mat);

  // int unknown = hgrid_->getTotalUnknwon();
  int unknown;

  double mat_time = (ros::Time::now() - t1).toSec();

  // Find optimal path through AmTSP
  t1 = ros::Time::now();
  const int dimension = mat.rows();
  const int drone_num = positions.size();

  vector<int> unknown_nums;
  int capacity = 0;
  for (int i = 0; i < grid_ids.size(); ++i) {
    int unum = hgrid_->getUnknownCellsNum(grid_ids[i]);
    unknown_nums.push_back(unum);
    capacity += unum;
    // std::cout << "Grid " << i << ": " << unum << std::endl;
  }
  // std::cout << "Total: " << capacity << std::endl;
  capacity = capacity * 0.75 * 0.1;

  // int prob_type;
  // if (grid_ids.size() >= 3)
  //   prob_type = 2;  // Use ACVRP
  // else
  //   prob_type = 1;  // Use AmTSP

  const int prob_type = 2;

  // Create problem file--------------------------
  ofstream file(ep_->mtsp_dir_ + "/amtsp3_" + to_string(ep_->drone_id_) + ".atsp");
  file << "NAME : pairopt\n";

  if (prob_type == 1)
    file << "TYPE : ATSP\n";
  else if (prob_type == 2)
    file << "TYPE : ACVRP\n";

  file << "DIMENSION : " + to_string(dimension) + "\n";
  file << "EDGE_WEIGHT_TYPE : EXPLICIT\n";
  file << "EDGE_WEIGHT_FORMAT : FULL_MATRIX\n";

  if (prob_type == 2) {
    file << "CAPACITY : " + to_string(capacity) + "\n";   // ACVRP
    file << "VEHICLES : " + to_string(drone_num) + "\n";  // ACVRP
  }

  // Cost matrix
  file << "EDGE_WEIGHT_SECTION\n";
  for (int i = 0; i < dimension; ++i) {
    for (int j = 0; j < dimension; ++j) {
      int int_cost = 100 * mat(i, j);
      file << int_cost << " ";
    }
    file << "\n";
  }

  if (prob_type == 2) {  // Demand section, ACVRP only
    file << "DEMAND_SECTION\n";
    file << "1 0\n";
    for (int i = 0; i < drone_num; ++i) {
      file << to_string(i + 2) + " 0\n";
    }
    for (int i = 0; i < grid_ids.size(); ++i) {
      int grid_unknown = unknown_nums[i] * 0.1;
      file << to_string(i + 2 + drone_num) + " " + to_string(grid_unknown) + "\n";
    }
    file << "DEPOT_SECTION\n";
    file << "1\n";
    file << "EOF";
  }

  file.close();

  // Create par file------------------------------------------
  int min_size = int(grid_ids.size()) / 2;
  int max_size = ceil(int(grid_ids.size()) / 2.0);
  file.open(ep_->mtsp_dir_ + "/amtsp3_" + to_string(ep_->drone_id_) + ".par");
  file << "SPECIAL\n";
  file << "PROBLEM_FILE = " + ep_->mtsp_dir_ + "/amtsp3_" + to_string(ep_->drone_id_) + ".atsp\n";
  if (prob_type == 1) {
    file << "SALESMEN = " << to_string(drone_num) << "\n";
    file << "MTSP_OBJECTIVE = MINSUM\n";
    // file << "MTSP_OBJECTIVE = MINMAX\n";
    file << "MTSP_MIN_SIZE = " << to_string(min_size) << "\n";
    file << "MTSP_MAX_SIZE = " << to_string(max_size) << "\n";
    file << "TRACE_LEVEL = 0\n";
  } else if (prob_type == 2) {
    file << "TRACE_LEVEL = 1\n";  // ACVRP
    file << "SEED = 0\n";         // ACVRP
  }
  file << "RUNS = 1\n";
  file << "TOUR_FILE = " + ep_->mtsp_dir_ + "/amtsp3_" + to_string(ep_->drone_id_) + ".tour\n";

  file.close();

  auto par_dir = ep_->mtsp_dir_ + "/amtsp3_" + to_string(ep_->drone_id_) + ".atsp";
  t1 = ros::Time::now();

  lkh_mtsp_solver::SolveMTSP srv;
  srv.request.prob = 3;
  // if (!tsp_client_.call(srv)) {
  if (!acvrp_client_.call(srv)) {
    ROS_ERROR("Fail to solve ACVRP.");
    return;
  }
  // system("/home/boboyu/software/LKH-3.0.6/LKH
  // /home/boboyu/workspaces/hkust_swarm_ws/src/swarm_exploration/utils/lkh_mtsp_solver/resource/amtsp3_1.par");

  double mtsp_time = (ros::Time::now() - t1).toSec();
  std::cout << "Allocation time: " << mtsp_time << std::endl;

  // Read results
  t1 = ros::Time::now();

  ifstream fin(ep_->mtsp_dir_ + "/amtsp3_" + to_string(ep_->drone_id_) + ".tour");
  string res;
  vector<int> ids;
  while (getline(fin, res)) {
    if (res.compare("TOUR_SECTION") == 0) break;
  }
  while (getline(fin, res)) {
    int id = stoi(res);
    ids.push_back(id - 1);
    if (id == -1) break;
  }
  fin.close();

  // Parse the m-tour of grid
  vector<vector<int>> tours;
  vector<int> tour;
  for (auto id : ids) {
    if (id > 0 && id <= drone_num) {
      tour.clear();
      tour.push_back(id);
    } else if (id >= dimension || id <= 0) {
      tours.push_back(tour);
    } else {
      tour.push_back(id);
    }
  }
  // // Print tour ids
  // for (auto tr : tours) {
  //   std::cout << "tour: ";
  //   for (auto id : tr) std::cout << id << ", ";
  //   std::cout << "" << std::endl;
  // }

  for (int i = 1; i < tours.size(); ++i) {
    if (tours[i][0] == 1) {
      ego_ids.insert(ego_ids.end(), tours[i].begin() + 1, tours[i].end());
    } else {
      other_ids.insert(other_ids.end(), tours[i].begin() + 1, tours[i].end());
    }
  }
  for (auto& id : ego_ids) {
    id = grid_ids[id - 1 - drone_num];
  }
  for (auto& id : other_ids) {
    id = grid_ids[id - 1 - drone_num];
  }
  // // Remove repeated grid
  // unordered_map<int, int> ego_map, other_map;
  // for (auto id : ego_ids) ego_map[id] = 1;
  // for (auto id : other_ids) other_map[id] = 1;

  // ego_ids.clear();
  // other_ids.clear();
  // for (auto p : ego_map) ego_ids.push_back(p.first);
  // for (auto p : other_map) other_ids.push_back(p.first);

  // sort(ego_ids.begin(), ego_ids.end());
  // sort(other_ids.begin(), other_ids.end());
}

double FastExplorationManager::computeGridPathCost(const Eigen::Vector3d& pos,
    const vector<int>& grid_ids, const vector<int>& first, const vector<vector<int>>& firsts,
    const vector<vector<int>>& seconds, const double& w_f) {
  if (grid_ids.empty()) return 0.0;

  double cost = 0.0;
  vector<Eigen::Vector3d> path;
  cost += hgrid_->getCostDroneToGrid(pos, grid_ids[0], first);
  for (int i = 0; i < grid_ids.size() - 1; ++i) {
    cost += hgrid_->getCostGridToGrid(grid_ids[i], grid_ids[i + 1], firsts, seconds, firsts.size());
  }
  return cost;
}

/**
 * @brief 简化findGlobalTourOfGrid， 只保留初始化分配部分，如果是drone1 且未初始化
 * 自动修改swarm_data
 */
void FastExplorationManager::initOneTimeGridAllocation(vector<int>& first_ids, vector<int>& second_ids) {
  auto& grid_ids = ed_->swarm_state_[ep_->drone_id_ - 1].grid_ids_;
  hgrid_->initHgridAlloaction(ep_->drone_id_, grid_ids, {}, first_ids, second_ids);
}

/**
 * @brief 根据本机swarm data中分配到的grid id，
 * 调用求解器规划最佳访问顺序，输出到indices。ed_也会顺便同步（便于fsm中绘图时调用）
 */
bool FastExplorationManager::findGlobalTourOfGrid(const vector<Eigen::Vector3d>& positions,
    const vector<Eigen::Vector3d>& velocities, vector<int>& indices, vector<vector<int>>& others,
    bool init) {

  ROS_INFO("Find grid tour---------------");

  auto t1 = ros::Time::now();

  auto& grid_ids = ed_->swarm_state_[ep_->drone_id_ - 1]
                       .grid_ids_;  // 在updateGridData中会剔除不符合relevent&&active的grid

  // hgrid_->updateBaseCoor();  // Use the latest basecoor transform of swarm

  vector<int> first_ids, second_ids;
  // hgrid_->inputFrontiers(ed_->averages_);

  // hgrid_->updateGridData(
  // ep_->drone_id_, grid_ids, ed_->reallocated_, ed_->last_grid_ids_, first_ids, second_ids);

  if (grid_ids.empty()) {
    ROS_WARN("Empty dominance.");
    ed_->grid_tour_.clear();
    return false;
  }

  std::cout << "Allocated grid: ";
  for (auto id : grid_ids) std::cout << id << ", ";
  std::cout << "" << std::endl;

  Eigen::MatrixXd mat;
  // uniform_grid_->getCostMatrix(positions, velocities, first_ids, grid_ids, mat);
  if (!init)
    hgrid_->getCostMatrix(positions, velocities, { first_ids }, { second_ids }, grid_ids, mat);
  else
    hgrid_->getCostMatrix(positions, velocities, { {} }, { {} }, grid_ids, mat);

  double mat_time = (ros::Time::now() - t1).toSec();

  // Find optimal path through ATSP
  t1 = ros::Time::now();
  const int dimension = mat.rows();
  const int drone_num = 1;

  // Create problem file
  ofstream file(ep_->mtsp_dir_ + "/amtsp2_" + to_string(ep_->drone_id_) + ".atsp");
  file << "NAME : amtsp\n";
  file << "TYPE : ATSP\n";
  file << "DIMENSION : " + to_string(dimension) + "\n";
  file << "EDGE_WEIGHT_TYPE : EXPLICIT\n";
  file << "EDGE_WEIGHT_FORMAT : FULL_MATRIX\n";
  file << "EDGE_WEIGHT_SECTION\n";
  for (int i = 0; i < dimension; ++i) {
    for (int j = 0; j < dimension; ++j) {
      int int_cost = 100 * mat(i, j);
      file << int_cost << " ";
    }
    file << "\n";
  }
  file.close();

  // Create par file
  file.open(ep_->mtsp_dir_ + "/amtsp2_" + to_string(ep_->drone_id_) + ".par");
  file << "SPECIAL\n";
  file << "PROBLEM_FILE = " + ep_->mtsp_dir_ + "/amtsp2_" + to_string(ep_->drone_id_) + ".atsp\n";
  file << "SALESMEN = " << to_string(drone_num) << "\n";
  file << "MTSP_OBJECTIVE = MINSUM\n";
  // file << "MTSP_MIN_SIZE = " << to_string(min(int(ed_->frontiers_.size()) / drone_num, 4)) <<
  // "\n"; file << "MTSP_MAX_SIZE = "
  //      << to_string(max(1, int(ed_->frontiers_.size()) / max(1, drone_num - 1))) << "\n";
  file << "RUNS = 1\n";
  file << "TRACE_LEVEL = 0\n";
  file << "TOUR_FILE = " + ep_->mtsp_dir_ + "/amtsp2_" + to_string(ep_->drone_id_) + ".tour\n";
  file.close();

  auto par_dir = ep_->mtsp_dir_ + "/amtsp2_" + to_string(ep_->drone_id_) + ".atsp";
  t1 = ros::Time::now();

  lkh_mtsp_solver::SolveMTSP srv;
  srv.request.prob = 2;
  if (!tsp_client_.call(srv)) {
    ROS_ERROR("Fail to solve ATSP.");
    return false;
  }

  double mtsp_time = (ros::Time::now() - t1).toSec();
  // std::cout << "AmTSP time: " << mtsp_time << std::endl;

  // Read results
  t1 = ros::Time::now();

  ifstream fin(ep_->mtsp_dir_ + "/amtsp2_" + to_string(ep_->drone_id_) + ".tour");
  string res;
  vector<int> ids;
  while (getline(fin, res)) {
    if (res.compare("TOUR_SECTION") == 0) break;
  }
  while (getline(fin, res)) {
    int id = stoi(res);
    ids.push_back(id - 1);
    if (id == -1) break;
  }
  fin.close();

  // Parse the m-tour of grid
  vector<vector<int>> tours;
  vector<int> tour;
  for (auto id : ids) {
    if (id > 0 && id <= drone_num) {
      tour.clear();
      tour.push_back(id);
    } else if (id >= dimension || id <= 0) {
      tours.push_back(tour);
    } else {
      tour.push_back(id);
    }
  }

  // for (auto tr : tours) {
  //   std::cout << "tour: ";
  //   for (auto id : tr) std::cout << id << ", ";
  //   std::cout << "" << std::endl;
  // }
  others.resize(drone_num - 1);
  for (int i = 1; i < tours.size(); ++i) {
    if (tours[i][0] == 1) {
      indices.insert(indices.end(), tours[i].begin() + 1, tours[i].end());
    } else {
      others[tours[i][0] - 2].insert(
          others[tours[i][0] - 2].end(), tours[i].begin(), tours[i].end());
    }
  }
  for (auto& id : indices) {
    id -= 1 + drone_num;
  }
  for (auto& other : others) {
    for (auto& id : other) id -= 1 + drone_num;
  }
  std::cout << "Grid tour: ";
  for (auto& id : indices) {
    id = grid_ids[id];
    std::cout << id << ", ";
  }
  std::cout << "" << std::endl;

  // uniform_grid_->getGridTour(indices, ed_->grid_tour_);
  grid_ids = indices;
  hgrid_->getGridTour(grid_ids, positions[0], ed_->grid_tour_, ed_->grid_tour2_);

  ed_->last_grid_ids_ = grid_ids;
  ed_->reallocated_ = false;

  // hgrid_->checkFirstGrid(grid_ids.front());

  return true;
}

/**
 * @brief 根据本机swarm data中分配到的grid id，
 * 调用求解器规划最佳访问顺序，输出到indices。ed_也会顺便同步（便于fsm中绘图时调用）
 */
bool FastExplorationManager::findCoverageTourOfGrid(const vector<Eigen::Vector3d>& positions,
    const vector<Eigen::Vector3d>& velocities, vector<int>& indices, vector<vector<int>>& others,
    bool init, const Eigen::Vector3d growth_vector) {
  ROS_INFO("Find grid tour---------------");

  auto t1 = ros::Time::now();

  const auto& local_grid_ids = ed_->swarm_state_[ep_->drone_id_ - 1].grid_ids_;
  vector<int> grid_ids;
  for (int i = local_grid_ids.size() - 1; i >= 0; --i) {  // 从本地grid_ids中筛选出未访问的grid
    if (ed_->visited_grid_map.find(local_grid_ids[i]) == ed_->visited_grid_map.end()) {
      grid_ids.push_back(local_grid_ids[i]);
    }
  }

  // hgrid_->updateBaseCoor();  // Use the latest basecoor transform of swarm

  vector<int> first_ids, second_ids;
  // hgrid_->inputFrontiers(ed_->averages_);

  hgrid_->getConsistentGrid(ed_->last_grid_ids_, grid_ids,  first_ids, second_ids);

  if (grid_ids.empty()) {
    ROS_WARN("Empty dominance.");
    ed_->grid_tour_.clear();
    return false;
  }

  std::cout << "Allocated grid: ";
  for (auto id : grid_ids) std::cout << id << ", ";
  std::cout << "" << std::endl;

  Eigen::MatrixXd mat;
  // uniform_grid_->getCostMatrix(positions, velocities, first_ids, grid_ids, mat);
  if (!init)
    hgrid_->getCoverageCostMatrix(positions, velocities, { first_ids }, { second_ids }, grid_ids, mat, growth_vector);
  else
    hgrid_->getCoverageCostMatrix(positions, velocities, { {} }, { {} }, grid_ids, mat, growth_vector);

  double mat_time = (ros::Time::now() - t1).toSec();

  // Find optimal path through ATSP
  t1 = ros::Time::now();
  const int dimension = mat.rows();
  const int drone_num = 1;

  // Create problem file
  ofstream file(ep_->mtsp_dir_ + "/amtsp2_" + to_string(ep_->drone_id_) + ".atsp");
  file << "NAME : amtsp\n";
  file << "TYPE : ATSP\n";
  file << "DIMENSION : " + to_string(dimension) + "\n";
  file << "EDGE_WEIGHT_TYPE : EXPLICIT\n";
  file << "EDGE_WEIGHT_FORMAT : FULL_MATRIX\n";
  file << "EDGE_WEIGHT_SECTION\n";
  for (int i = 0; i < dimension; ++i) {
    for (int j = 0; j < dimension; ++j) {
      int int_cost = 100 * mat(i, j);
      file << int_cost << " ";
    }
    file << "\n";
  }
  file.close();

  // Create par file
  file.open(ep_->mtsp_dir_ + "/amtsp2_" + to_string(ep_->drone_id_) + ".par");
  file << "SPECIAL\n";
  file << "PROBLEM_FILE = " + ep_->mtsp_dir_ + "/amtsp2_" + to_string(ep_->drone_id_) + ".atsp\n";
  file << "SALESMEN = " << to_string(drone_num) << "\n";
  file << "MTSP_OBJECTIVE = MINSUM\n";
  // file << "MTSP_MIN_SIZE = " << to_string(min(int(ed_->frontiers_.size()) / drone_num, 4)) <<
  // "\n"; file << "MTSP_MAX_SIZE = "
  //      << to_string(max(1, int(ed_->frontiers_.size()) / max(1, drone_num - 1))) << "\n";
  file << "RUNS = 1\n";
  file << "TRACE_LEVEL = 0\n";
  file << "TOUR_FILE = " + ep_->mtsp_dir_ + "/amtsp2_" + to_string(ep_->drone_id_) + ".tour\n";
  file.close();

  auto par_dir = ep_->mtsp_dir_ + "/amtsp2_" + to_string(ep_->drone_id_) + ".atsp";
  t1 = ros::Time::now();

  lkh_mtsp_solver::SolveMTSP srv;
  srv.request.prob = 2;
  if (!tsp_client_.call(srv)) {
    ROS_ERROR("Fail to solve ATSP.");
    return false;
  }

  double mtsp_time = (ros::Time::now() - t1).toSec();
  // std::cout << "AmTSP time: " << mtsp_time << std::endl;

  // Read results
  t1 = ros::Time::now();

  ifstream fin(ep_->mtsp_dir_ + "/amtsp2_" + to_string(ep_->drone_id_) + ".tour");
  string res;
  vector<int> ids;
  while (getline(fin, res)) {
    if (res.compare("TOUR_SECTION") == 0) break;
  }
  while (getline(fin, res)) {
    int id = stoi(res);
    ids.push_back(id - 1);
    if (id == -1) break;
  }
  fin.close();

  // Parse the m-tour of grid
  vector<vector<int>> tours;
  vector<int> tour;
  for (auto id : ids) {
    if (id > 0 && id <= drone_num) {
      tour.clear();
      tour.push_back(id);
    } else if (id >= dimension || id <= 0) {
      tours.push_back(tour);
    } else {
      tour.push_back(id);
    }
  }

  // for (auto tr : tours) {
  //   std::cout << "tour: ";
  //   for (auto id : tr) std::cout << id << ", ";
  //   std::cout << "" << std::endl;
  // }
  others.resize(drone_num - 1);
  for (int i = 1; i < tours.size(); ++i) {
    if (tours[i][0] == 1) {
      indices.insert(indices.end(), tours[i].begin() + 1, tours[i].end());
    } else {
      others[tours[i][0] - 2].insert(
          others[tours[i][0] - 2].end(), tours[i].begin(), tours[i].end());
    }
  }
  for (auto& id : indices) {
    id -= 1 + drone_num;
  }
  for (auto& other : others) {
    for (auto& id : other) id -= 1 + drone_num;
  }
  std::cout << "Grid tour: ";
  for (auto& id : indices) {
    id = grid_ids[id];
    std::cout << id << ", ";
  }
  std::cout << "" << std::endl;

  // uniform_grid_->getGridTour(indices, ed_->grid_tour_);
  grid_ids = indices;
  hgrid_->getGridTour(grid_ids, positions[0], ed_->grid_tour_, ed_->grid_tour2_);

  ed_->last_grid_ids_ = grid_ids;
  ed_->reallocated_ = false;

  // hgrid_->checkFirstGrid(grid_ids.front());

  return true;
}

void FastExplorationManager::findTourOfFrontier(const Vector3d& cur_pos, const Vector3d& cur_vel,
    const Vector3d& cur_yaw, const vector<int>& ftr_ids, const vector<Eigen::Vector3d>& grid_pos,
    vector<int>& indices) {

  auto t1 = ros::Time::now();

  vector<Eigen::Vector3d> positions = { cur_pos };
  vector<Eigen::Vector3d> velocities = { cur_vel };
  vector<double> yaws = { cur_yaw[0] };

  // frontier_finder_->getSwarmCostMatrix(positions, velocities, yaws, mat);
  Eigen::MatrixXd mat;
  frontier_finder_->getSwarmCostMatrix(positions, velocities, yaws, ftr_ids, grid_pos, mat);
  const int dimension = mat.rows();
  // std::cout << "dim of frontier TSP mat: " << dimension << std::endl;

  double mat_time = (ros::Time::now() - t1).toSec();
  // ROS_INFO("mat time: %lf", mat_time);

  // Find optimal allocation through AmTSP
  t1 = ros::Time::now();

  // Create problem file
  ofstream file(ep_->mtsp_dir_ + "/amtsp_" + to_string(ep_->drone_id_) + ".atsp");
  file << "NAME : amtsp\n";
  file << "TYPE : ATSP\n";
  file << "DIMENSION : " + to_string(dimension) + "\n";
  file << "EDGE_WEIGHT_TYPE : EXPLICIT\n";
  file << "EDGE_WEIGHT_FORMAT : FULL_MATRIX\n";
  file << "EDGE_WEIGHT_SECTION\n";
  for (int i = 0; i < dimension; ++i) {
    for (int j = 0; j < dimension; ++j) {
      int int_cost = 100 * mat(i, j);
      file << int_cost << " ";
    }
    file << "\n";
  }
  file.close();

  // Create par file
  const int drone_num = 1;

  file.open(ep_->mtsp_dir_ + "/amtsp_" + to_string(ep_->drone_id_) + ".par");
  file << "SPECIAL\n";
  file << "PROBLEM_FILE = " + ep_->mtsp_dir_ + "/amtsp_" + to_string(ep_->drone_id_) + ".atsp\n";
  file << "SALESMEN = " << to_string(drone_num) << "\n";
  file << "MTSP_OBJECTIVE = MINSUM\n";
  file << "MTSP_MIN_SIZE = " << to_string(min(int(ed_->frontiers_.size()) / drone_num, 4)) << "\n";
  file << "MTSP_MAX_SIZE = "
       << to_string(max(1, int(ed_->frontiers_.size()) / max(1, drone_num - 1))) << "\n";
  file << "RUNS = 1\n";
  file << "TRACE_LEVEL = 0\n";
  file << "TOUR_FILE = " + ep_->mtsp_dir_ + "/amtsp_" + to_string(ep_->drone_id_) + ".tour\n";
  file.close();

  auto par_dir = ep_->mtsp_dir_ + "/amtsp_" + to_string(ep_->drone_id_) + ".atsp";
  t1 = ros::Time::now();

  lkh_mtsp_solver::SolveMTSP srv;
  srv.request.prob = 1;
  if (!tsp_client_.call(srv)) {
    ROS_ERROR("Fail to solve ATSP.");
    return;
  }

  double mtsp_time = (ros::Time::now() - t1).toSec();
  // ROS_INFO("AmTSP time: %lf", mtsp_time);

  // Read results
  t1 = ros::Time::now();

  ifstream fin(ep_->mtsp_dir_ + "/amtsp_" + to_string(ep_->drone_id_) + ".tour");
  string res;
  vector<int> ids;
  while (getline(fin, res)) {
    if (res.compare("TOUR_SECTION") == 0) break;
  }
  while (getline(fin, res)) {
    int id = stoi(res);
    ids.push_back(id - 1);
    if (id == -1) break;
  }
  fin.close();

  // Parse the m-tour
  vector<vector<int>> tours;
  vector<int> tour;
  for (auto id : ids) {
    if (id > 0 && id <= drone_num) {
      tour.clear();
      tour.push_back(id);
    } else if (id >= dimension || id <= 0) {
      tours.push_back(tour);
    } else {
      tour.push_back(id);
    }
  }

  vector<vector<int>> others(drone_num - 1);
  for (int i = 1; i < tours.size(); ++i) {
    if (tours[i][0] == 1) {
      indices.insert(indices.end(), tours[i].begin() + 1, tours[i].end());
    }
    // else {
    //   others[tours[i][0] - 2].insert(
    //       others[tours[i][0] - 2].end(), tours[i].begin() + 1, tours[i].end());
    // }
  }
  for (auto& id : indices) {
    id -= 1 + drone_num;
  }
  // for (auto& other : others) {
  //   for (auto& id : other)
  //     id -= 1 + drone_num;
  // }

  if (ed_->grid_tour_.size() > 2) {  // Remove id for next grid, since it is considered in the TSP
    indices.pop_back();
  }
  // Subset of frontier inside first grid
  for (int i = 0; i < indices.size(); ++i) {
    indices[i] = ftr_ids[indices[i]];
  }

  // Get the path of optimal tour from path matrix
  frontier_finder_->getPathForTour(cur_pos, indices, ed_->frontier_tour_);
  if (!grid_pos.empty()) {
    ed_->frontier_tour_.push_back(grid_pos[0]);
  }

  // ed_->other_tours_.clear();
  // for (int i = 1; i < positions.size(); ++i) {
  //   ed_->other_tours_.push_back({});
  //   frontier_finder_->getPathForTour(positions[i], others[i - 1], ed_->other_tours_[i - 1]);
  // }

  double parse_time = (ros::Time::now() - t1).toSec();
  // ROS_INFO("Cost mat: %lf, TSP: %lf, parse: %f, %d frontiers assigned.", mat_time, mtsp_time,
  //     parse_time, indices.size());
}


// ========================== INSPECT ===========================
int FastExplorationManager::planInspectMotion(const Vector3d& start_pos_, const Vector3d& pos,
    const Vector3d& vel, const Vector3d& acc, const Vector3d& yaw, const vector<Vector3d>& island_box) {
  if (island_box.size() < 2) return NO_VALID_ISLAND;

  ros::Time t1 = ros::Time::now();
  auto t2 = t1;
  ed_->frontier_tour_.clear();

  // 1. update frontier with island_box
  vector<int> relevant_frontier_ids;
  getFrontiersInBox(island_box[0], island_box[1], relevant_frontier_ids);
  if (relevant_frontier_ids.empty()) {
    ROS_WARN("[INSP] Island has no frontiers to inspect.");
    return NO_VALID_ISLAND;
  }
  ROS_INFO("[INSP] Found %zu frontiers for island.", relevant_frontier_ids.size());

  // 2. plan global path
  vector<int> frontier_tour_indices;
  Vector3d next_pos;
  double next_yaw;
  
  if (relevant_frontier_ids.size() == 1) {
    int frt_id = relevant_frontier_ids[0];
    vector<Viewpoint> vps;
    frontier_finder_->getFrontierViewpoints(frt_id, vps);

    double min_cost = 100000;
    int min_cost_id = -1;
    for (int i = 0; i < vps.size(); ++i) {
      vector<Eigen::Vector3d> path;
      double cost = ViewNode::computeCost(pos, vps[i].pos_, yaw[0], vps[i].yaw_, vel, 0.0, path);
      if (cost < min_cost) {
        min_cost = cost;
        min_cost_id = i;
      }
    }
    next_pos = vps[min_cost_id].pos_;
    next_yaw = vps[min_cost_id].yaw_;
    
  } else if (relevant_frontier_ids.size() > 1) {
    findInspectTourOfFrontier(pos, vel, yaw, relevant_frontier_ids, island_box, frontier_tour_indices);
    if (frontier_tour_indices.empty()) {
      ROS_WARN("[INSP] Failed to find a tour for island's frontiers, go to nearst one");

      int frt_id = relevant_frontier_ids[0];
      vector<Viewpoint> vps;
      frontier_finder_->getFrontierViewpoints(frt_id, vps);

      double min_cost = 100000;
      int min_cost_id = -1;
      for (int i = 0; i < vps.size(); ++i) {
        vector<Eigen::Vector3d> path;
        double cost = ViewNode::computeCost(pos, vps[i].pos_, yaw[0], vps[i].yaw_, vel, 0.0, path);
        if (cost < min_cost) {
          min_cost = cost;
          min_cost_id = i;
        }
      }
      next_pos = vps[min_cost_id].pos_;
      next_yaw = vps[min_cost_id].yaw_;
      // return INSP_FAIL;
    }

    // 3. get best next_pos and next_yaw
    // ed_->refined_ids_.clear();
    // ed_->unrefined_points_.clear();
    // int refine_num = min((int)frontier_tour_indices.size(), ep_->refined_num_);
    // for (int i = 0; i < refine_num; ++i) {
    //   int ftr_id = frontier_tour_indices[i];
    //   ed_->unrefined_points_.push_back(ed_->points_[ftr_id]);
    //   ed_->refined_ids_.push_back(ftr_id);
    // }
    // ed_->n_points_.clear();
    // vector<vector<double>> n_yaws;
    // frontier_finder_->getViewpointsInfo(
    //     pos, ed_->refined_ids_, ep_->top_view_num_, ep_->max_decay_, ed_->n_points_, n_yaws);

    // // 通过局部图搜索，在候选视点中找到最优的下一个目标
    // ed_->refined_points_.clear();
    // vector<double> refined_yaws;
    // refineLocalTour(pos, vel, yaw, ed_->n_points_, n_yaws, ed_->refined_points_, refined_yaws);
    // if (ed_->refined_points_.empty()) {
    //   ROS_ERROR("[INSP] Local refinement failed, no path generated.");
    //   return INSP_FAIL;
    // }
    // next_pos = ed_->refined_points_[0];
    // next_yaw = refined_yaws[0];
  }

  // 4. plan trajectory
  cout << "next pos: " << next_pos.transpose() << ", next yaw: " << next_yaw << endl;
  if (planTrajToView(pos, vel, acc, yaw, next_pos, next_yaw) == FAIL) {
    ROS_WARN("[INSP] Failed to plan trajectory to next view.");
    return INSP_FAIL;
  }

  double total = (ros::Time::now() - t2).toSec();
  return INSP_SUCCEED;
}

/**
 * @brief find best inspect frt tour
 */
void FastExplorationManager::findInspectTourOfFrontier(const Vector3d& cur_pos,
    const Vector3d& cur_vel, const Vector3d& cur_yaw, const vector<int>& ftr_ids,
    const vector<Vector3d>& island_box, vector<int>& indices) {
  auto t1 = ros::Time::now();

  vector<Eigen::Vector3d> positions = { cur_pos };
  vector<Eigen::Vector3d> velocities = { cur_vel };
  vector<double> yaws = { cur_yaw[0] };

  Eigen::MatrixXd mat;
  frontier_finder_->getInspectCostMatrix(positions, velocities, yaws, ftr_ids, island_box[0], island_box[1], mat);
  const int dimension = mat.rows();
  cout << "mat: " << mat << endl;

  double mat_time = (ros::Time::now() - t1).toSec();

  // Find optimal allocation through AmTSP
  t1 = ros::Time::now();

  // Create problem file
  ofstream file(ep_->mtsp_dir_ + "/insp_" + to_string(ep_->drone_id_) + ".atsp");
  file << "NAME : insp\n";
  file << "TYPE : ATSP\n";
  file << "DIMENSION : " + to_string(dimension) + "\n";
  file << "EDGE_WEIGHT_TYPE : EXPLICIT\n";
  file << "EDGE_WEIGHT_FORMAT : FULL_MATRIX\n";
  file << "EDGE_WEIGHT_SECTION\n";
  for (int i = 0; i < dimension; ++i) {
    for (int j = 0; j < dimension; ++j) {
      int int_cost = 100 * mat(i, j);
      file << int_cost << " ";
    }
    file << "\n";
  }
  file.close();

  // Create par file
  const int drone_num = 1;

  file.open(ep_->mtsp_dir_ + "/insp_" + to_string(ep_->drone_id_) + ".par");
  file << "SPECIAL\n";
  file << "PROBLEM_FILE = " + ep_->mtsp_dir_ + "/insp_" + to_string(ep_->drone_id_) + ".atsp\n";
  file << "SALESMEN = " << to_string(drone_num) << "\n";
  file << "MTSP_OBJECTIVE = MINSUM\n";
  file << "RUNS = 1\n";
  file << "TRACE_LEVEL = 0\n";
  file << "TOUR_FILE = " + ep_->mtsp_dir_ + "/insp_" + to_string(ep_->drone_id_) + ".tour\n";
  file.close();

  auto par_dir = ep_->mtsp_dir_ + "/insp_" + to_string(ep_->drone_id_) + ".atsp";
  t1 = ros::Time::now();

  lkh_mtsp_solver::SolveMTSP srv;
  srv.request.prob = 2;
  if (!tsp_client_.call(srv)) {
    ROS_ERROR("Fail to solve ATSP.");
    return;
  }

  double mtsp_time = (ros::Time::now() - t1).toSec();

  // Read results
  t1 = ros::Time::now();

  ifstream fin(ep_->mtsp_dir_ + "/insp_" + to_string(ep_->drone_id_) + ".tour");
  string res;
  vector<int> ids;
  while (getline(fin, res)) {
    if (res.compare("TOUR_SECTION") == 0) break;
  }
  while (getline(fin, res)) {
    int id = stoi(res);
    ids.push_back(id - 1);
    if (id == -1) break;
  }
  fin.close();

  // Parse the m-tour
  vector<vector<int>> tours;
  vector<int> tour;
  for (auto id : ids) {
    if (id > 0 && id <= drone_num) {
      tour.clear();
      tour.push_back(id);
    } else if (id >= dimension || id <= 0) {
      tours.push_back(tour);
    } else {
      tour.push_back(id);
    }
  }

  vector<vector<int>> others(drone_num - 1);
  for (int i = 1; i < tours.size(); ++i) {
    if (tours[i][0] == 1) {
      indices.insert(indices.end(), tours[i].begin() + 1, tours[i].end());
    }
  }
  for (auto& id : indices) {
    id -= 1 + drone_num;
  }

  if (ed_->grid_tour_.size() > 2) {  // Remove id for next grid, since it is considered in the TSP
    indices.pop_back();
  }
  // Subset of frontier inside first grid
  for (int i = 0; i < indices.size(); ++i) {
    indices[i] = ftr_ids[indices[i]];
  }

  // Get the path of optimal tour from path matrix
  frontier_finder_->getPathForTour(cur_pos, indices, ed_->frontier_tour_);

  double parse_time = (ros::Time::now() - t1).toSec();
}

void FastExplorationManager::getFrontiersInBox(const Eigen::Vector3d& bmin, const Eigen::Vector3d& bmax, vector<int>& ftr_ids_in_box) {
  ftr_ids_in_box.clear();
  vector<pair<Eigen::Vector3d, Eigen::Vector3d>> all_frontier_boxes;
  frontier_finder_->getFrontierBoxes(all_frontier_boxes);

  Eigen::Vector3d frontier_bmin, frontier_bmax;
  for (size_t i = 0; i < all_frontier_boxes.size(); ++i) {
    const auto& box_raw = all_frontier_boxes[i];
    const Eigen::Vector3d& center = box_raw.first;
    const Eigen::Vector3d& scale = box_raw.second;
    frontier_bmin = center - scale / 2.0;
    frontier_bmax = center + scale / 2.0;

    // 检查两个包围盒是否重叠 (AABB Overlap Test)
    bool has_overlap = true;
    for (int j = 0; j < 3; ++j) {
      if (std::max(bmin[j], frontier_bmin[j]) > std::min(bmax[j], frontier_bmax[j]) + 1e-3) {
        has_overlap = false;
        break;
      }
    }

    if (has_overlap) {
      cout << "Frontier box id:" << i << " box: ";
      cout << frontier_bmin.transpose() << "   " << frontier_bmax.transpose() << endl; 
      // frontier_finder_ 中的 frontier id 与其在列表中的索引是一致的
      ftr_ids_in_box.push_back(i);
    }
  }
}

}  // namespace fast_planner
