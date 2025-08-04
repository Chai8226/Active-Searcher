
#include <exploration_manager/fast_exploration_fsm.h>
#include <plan_manage/planner_manager.h>
#include <exploration_manager/fast_exploration_manager.h>
#include <traj_utils/planning_visualization.h>

#include <exploration_manager/expl_data.h>
#include <exploration_manager/HGrid.h>
#include <exploration_manager/GridTour.h>

#include <plan_env/edt_environment.h>
#include <plan_env/sdf_map.h>
#include <plan_env/multi_map_manager.h>
#include <active_perception/perception_utils.h>
#include <active_perception/hgrid.h>
// #include <active_perception/uniform_grid.h>
// #include <lkh_tsp_solver/lkh_interface.h>
// #include <lkh_mtsp_solver/lkh3_interface.h>

#include <fstream>

using Eigen::Vector4d;

namespace fast_planner {
void FastExplorationFSM::init(ros::NodeHandle& nh) {
  fp_.reset(new FSMParam);
  fd_.reset(new FSMData);

  /*  Fsm param  */
  nh.param("fsm/thresh_replan1", fp_->replan_thresh1_, -1.0);  // 参数服务器
  nh.param("fsm/thresh_replan2", fp_->replan_thresh2_, -1.0);
  nh.param("fsm/thresh_replan3", fp_->replan_thresh3_, -1.0);
  nh.param("fsm/replan_time", fp_->replan_time_, -1.0);
  nh.param("fsm/attempt_interval", fp_->attempt_interval_, 0.2);
  nh.param("fsm/pair_opt_interval", fp_->pair_opt_interval_, 1.0);
  nh.param("fsm/repeat_send_num", fp_->repeat_send_num_, 10);

  // 修改 读取地图信息
  Eigen::Vector3d min_, max_;
  nh.param("sdf_map/box_min_x", min_[0], 0.0);
  nh.param("sdf_map/box_min_y", min_[1], 0.0);
  nh.param("sdf_map/box_min_z", min_[2], 0.0);
  nh.param("sdf_map/box_max_x", max_[0], 0.0);
  nh.param("sdf_map/box_max_y", max_[1], 0.0);
  nh.param("sdf_map/box_max_z", max_[2], 0.0);
  double grid_size;
  nh.param("partitioning/grid_size", grid_size, 3.9);  // 4.2

  // inspect
  nh.param("uav_mode", fp_->drone_type_, 1);

  // FIXME box?
  auto map_size = max_ - min_;
  if (map_size(0) > map_size(1)) {  // 区分长宽，计算简易的牛耕长度,xy面上
    allocation_bar = map_size(1) * 2 + ceil(map_size(1) / grid_size) * map_size(0);
  } else {
    allocation_bar = map_size(0) * 2 + ceil(map_size(0) / grid_size) * map_size(1);
  }

  /* Initialize main modules */
  expl_manager_.reset(new FastExplorationManager);
  expl_manager_->initialize(nh);
  visualization_.reset(new PlanningVisualization(nh));

  ifinder_.reset(new IslandFinder());

  ifinder_->init(expl_manager_->sdf_map_, expl_manager_->frontier_finder_ , getId(), expl_manager_->ep_->drone_num_);

  planner_manager_ = expl_manager_->planner_manager_;
  state_ = EXPL_STATE::INIT;
  fd_->have_odom_ = false;
  fd_->state_str_ = { "INIT", "WAIT_TRIGGER", "OPT", "PLAN_TRAJ", "PUB_TRAJ", "EXEC_TRAJ", "FINISH",
    "IDLE" };
  fd_->static_state_ = true;
  fd_->trigger_ = false;
  fd_->avoid_collision_ = false;
  fd_->go_back_ = false;

  /* Ros sub, pub and timer */
  exec_timer_ = nh.createTimer(ros::Duration(0.01), &FastExplorationFSM::FSMCallback, this);
  safety_timer_ = nh.createTimer(ros::Duration(0.05), &FastExplorationFSM::safetyCallback, this);
  frontier_timer_ = nh.createTimer(ros::Duration(0.1), &FastExplorationFSM::frontierCallback, this);

  trigger_sub_ =
      nh.subscribe("/move_base_simple/goal", 1, &FastExplorationFSM::triggerCallback, this);
  odom_sub_ = nh.subscribe("/odom_world", 1, &FastExplorationFSM::odometryCallback, this);

  replan_pub_ = nh.advertise<std_msgs::Empty>("/planning/replan", 10);
  new_pub_ = nh.advertise<std_msgs::Empty>("/planning/new", 10);
  bspline_pub_ = nh.advertise<bspline::Bspline>("/planning/bspline", 10);

  // Swarm, timer, pub and sub
  drone_state_timer_ =
      nh.createTimer(ros::Duration(0.04), &FastExplorationFSM::droneStateTimerCallback, this);
  drone_state_pub_ =
      nh.advertise<exploration_manager::DroneState>("/swarm_expl/drone_state_send", 10);
  drone_state_sub_ = nh.subscribe(
      "/swarm_expl/drone_state_recv", 10, &FastExplorationFSM::droneStateMsgCallback, this);

  opt_timer_ = nh.createTimer(ros::Duration(0.05), &FastExplorationFSM::optTimerCallback, this);
  opt_pub_ = nh.advertise<exploration_manager::PairOpt>("/swarm_expl/pair_opt_send", 10);
  opt_sub_ = nh.subscribe("/swarm_expl/pair_opt_recv", 100, &FastExplorationFSM::optMsgCallback,
      this, ros::TransportHints().tcpNoDelay());

  opt_res_pub_ =
      nh.advertise<exploration_manager::PairOptResponse>("/swarm_expl/pair_opt_res_send", 10);
  opt_res_sub_ = nh.subscribe("/swarm_expl/pair_opt_res_recv", 100,
      &FastExplorationFSM::optResMsgCallback, this, ros::TransportHints().tcpNoDelay());

  swarm_traj_pub_ = nh.advertise<bspline::Bspline>("/planning/swarm_traj_send", 100);
  swarm_traj_sub_ =
      nh.subscribe("/planning/swarm_traj_recv", 100, &FastExplorationFSM::swarmTrajCallback, this);
  swarm_traj_timer_ =
      nh.createTimer(ros::Duration(0.1), &FastExplorationFSM::swarmTrajTimerCallback, this);

  hgrid_pub_ = nh.advertise<exploration_manager::HGrid>("/swarm_expl/hgrid_send", 10);
  grid_tour_pub_ = nh.advertise<exploration_manager::GridTour>("/swarm_expl/grid_tour_send", 10);

  init_ = true;
  disqueue_replan = false;

  need_opt = false;
  // inspect
  have_island_ = false;
  islands_need_merge = false;
  island_timer = nh.createTimer(ros::Duration(0.1), &FastExplorationFSM::islandTimerCallback, this);
  island_pub_ =
      nh.advertise<exploration_manager::IslandArray>("/swarm_expl/island_send", 10);  // TODO
  island_sub_ =
      nh.subscribe("/swarm_expl/island_recv", 10, &FastExplorationFSM::islandMsgCallback, this);

  cam_switch_pub_ = nh.advertise<exploration_manager::CamSwitch>("/pcl_render_node/cam_switch", 1);
}

int FastExplorationFSM::getId() {
  return expl_manager_->ep_->drone_id_;
}

/**
 * @brief 主执行函数，根据当前FSM状态进行动作
 */
void FastExplorationFSM::FSMCallback(const ros::TimerEvent& e) {
  ROS_INFO_STREAM_THROTTLE(1.0, "[FSM]: Drone " << getId() << " type: " << fp_->drone_type_
                                                << " state: " << fd_->state_str_[int(state_)]);

  if (!completed_island_ids_.empty()) {
    cout << "Complete island: ";
    for (auto &ci : completed_island_ids_)  cout << ci << ", ";
    cout << endl;  
  }
                                          

  switch (state_) {
    case INIT: {
      if (!fd_->have_odom_) {
        ROS_WARN_THROTTLE(1.0, "no odom");
        return;
      }
      if ((ros::Time::now() - fd_->fsm_init_time_).toSec() < 2.0) {
        ROS_WARN_THROTTLE(1.0, "wait for init");
        return;
      }
      transitState(WAIT_TRIGGER, "FSM");
      break;
    }

    case WAIT_TRIGGER: {
      ROS_WARN_THROTTLE(1.0, "wait for trigger.");
      break;
    }

    case FINISH: {
      ROS_INFO_THROTTLE(1.0, "finish exploration.");
      break;
    }

    case IDLE: {
      double check_interval = (ros::Time::now() - fd_->last_check_frontier_time_).toSec();
      if (check_interval > 50.0) {
        ROS_WARN("Go back to (0,0,1)");
        expl_manager_->ed_->next_pos_ = fd_->start_pos_;
        expl_manager_->ed_->next_yaw_ = 0.0;

        fd_->go_back_ = true;
        transitState(PLAN_TRAJ, "FSM");
      }
      break;
    }

    case OPT: {
      if (fp_->drone_type_ == 0) {
        transitState(PLAN_TRAJ, "FSM");
      }
      ROS_WARN("High level drone %d doing opt", getId());
      // // switchDroneType(1); //testhigh

      // ROS_WARN_THROTTLE(1.0, "[FSM] doing opt.");

      // const auto& states = expl_manager_->ed_->swarm_state_;
      // double total = 0;
      // int n = 0;

      // for (auto state : states) {
      //   if (state.allocation_cost > 0) {
      //     total += state.allocation_cost;
      //     n++;
      //   }
      // }
      // double avrg = total / n;  // testhigh 暂时把分母写成1
      // // double avrg = total;
      // double stdev = 0;
      // if (avrg > 0) {
      //   expl_manager_->planDisQueue(fd_->start_pos_);
      //   transitState(PLAN_TRAJ, "FSM");

      //   //   std::for_each(states.begin(), states.end(), [&](DroneState st) {
      //   //     if (st.allocation_cost > 0)
      //   //       stdev += (st.allocation_cost - avrg) * (st.allocation_cost - avrg);
      //   //     else
      //   //       stdev += avrg * avrg;
      //   //   });
      //   //   double cv = sqrt(stdev) / avrg;  // 考察变异系数

      //   //   if (/*cv > 0 && cv <= 0.5 && */ total <= allocation_bar * 1.1 && total > 0) {
      //   //     ROS_WARN("opt done: drone %d, [opt done] cv = %lf, total = %lf.", getId(), cv,
      //   //     total); expl_manager_->makeDisQueue(); transitState(PLAN_TRAJ, "FSM");
      //   //   } else {
      //   //     ROS_WARN_THROTTLE(1.0, "continue opt: cv = %lf, total = %lf, expect: %lf - / under
      //   //     %lf -",
      //   //         getId(), cv, total, 2.8, allocation_bar * 1.1);
      //   //   }
      //   // } else {
      //   //   ROS_WARN("[WRONG] id: %d, avrg = %lf, total num: %d", getId(), avrg, n);
      // }
      break;
    }

    case PLAN_TRAJ: {
      if (fd_->static_state_) {
        // Plan from static state (hover)
        fd_->start_pt_ = fd_->odom_pos_;
        fd_->start_vel_ = fd_->odom_vel_;
        fd_->start_acc_.setZero();
        fd_->start_yaw_ << fd_->odom_yaw_, 0, 0;
      } else {
        // Replan from non-static state, starting from 'replan_time' seconds later
        LocalTrajData* info = &planner_manager_->local_data_;
        double t_r = (ros::Time::now() - info->start_time_).toSec() + fp_->replan_time_;
        fd_->start_pt_ = info->position_traj_.evaluateDeBoorT(t_r);
        fd_->start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_r);
        fd_->start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_r);
        fd_->start_yaw_(0) = info->yaw_traj_.evaluateDeBoorT(t_r)[0];
        fd_->start_yaw_(1) = info->yawdot_traj_.evaluateDeBoorT(t_r)[0];
        fd_->start_yaw_(2) = info->yawdotdot_traj_.evaluateDeBoorT(t_r)[0];
      }

      // Inform traj_server the replanning
      replan_pub_.publish(std_msgs::Empty());
      if (fp_->drone_type_ == 1) {
        int res = callCoveragePlanner();
        if (res == SUCCEED) {
          ROS_WARN("id %d into pub_traj!", getId());
          transitState(PUB_TRAJ, "FSM");
        } else if (res == FAIL) {  // Keep trying to replan
          fd_->static_state_ = true;
          ROS_WARN("[FSM] Plan fail");
        } else if (res == NO_GRID) {
          if (expl_manager_->ed_->dis_queue.empty()) {
            ROS_WARN("[FSM] coverage done!");
            switchDroneType(0); // 切换状态
            //transitState(OPT, "FSM");
            // need_opt = true;  // 需要重新优化
            visualize(1);
            break;
          }
          fd_->static_state_ = true;
          fd_->last_check_frontier_time_ = ros::Time::now();
          ROS_WARN("[FSM] No grid");
          transitState(IDLE, "FSM");
          visualize(1);
        }
      } else {
        ROS_WARN("drone %d into inspect planner", getId());
        int res = callInspectPlanner();
        if (res == INSP_SUCCEED) {
          transitState(PUB_TRAJ, "FSM");
          ROS_WARN("[FSM] Inspect plan succeed, into pub traj");
        } else if (res == INSP_FAIL) {  // Keep trying to replan
          fd_->static_state_ = true;
          ROS_WARN("[FSM] Plan fail");
        } else if (res == NO_VALID_ISLAND) {
          fd_->static_state_ = true;
          completed_island_ids_.insert(current_inspection_island_id_);
          ROS_WARN("[FSM] this island no frontier, next one.");
          inpsct_planner_type_ = 1;
        }
      }
      break;
    }

    case PUB_TRAJ: {
      double dt = (ros::Time::now() - fd_->newest_traj_.start_time).toSec();
      if (dt > 0) {
        bspline_pub_.publish(fd_->newest_traj_);
        fd_->static_state_ = false;

        // fd_->newest_traj_.drone_id = planner_manager_->swarm_traj_data_.drone_id_;
        fd_->newest_traj_.drone_id = expl_manager_->ep_->drone_id_;
        swarm_traj_pub_.publish(fd_->newest_traj_);

        thread vis_thread(&FastExplorationFSM::visualize, this, 2);
        vis_thread.detach();
        transitState(EXEC_TRAJ, "FSM");
      }
      break;
    }

    case EXEC_TRAJ: {
      auto tn = ros::Time::now();
      LocalTrajData* info = &planner_manager_->local_data_;
      double t_cur = (tn - info->start_time_).toSec();

      if (!fd_->go_back_) {
        bool need_replan = false;
        if (fp_->drone_type_ == 1) {
          if (info->duration_ - t_cur < fp_->replan_thresh1_) {
            // Replan if traj is almost fully executed
            ROS_WARN("Replan: traj fully executed=================================");
            need_replan = true;
            updateLocalDisqueue();
          } else if (disqueue_replan) {
            ROS_WARN("Replan: reached disqueque head=================================");
            need_replan = true;
            disqueue_replan = false;
          }
          if (need_opt) {
            transitState(OPT, "FSM");
            need_opt = false;
          } else if (need_replan) {
            if (expl_manager_->updateFrontierStruct(fd_->odom_pos_) != 0) {
              // Update frontier and plan new motion
              thread vis_thread(&FastExplorationFSM::visualize, this, 1);
              vis_thread.detach();
              transitState(PLAN_TRAJ, "FSM");
            } 
          }

        } else {
          if (checkIslandFinished()) {
            ROS_WARN("Replan: this island finished=====================================");
            completed_island_ids_.insert(current_inspection_island_id_);
            inpsct_planner_type_ = 1;
            need_replan = true;
          } else if (t_cur > fp_->replan_thresh2_ && expl_manager_->frontier_finder_->isFrontierCovered()) {
            ROS_WARN("Replan: cluster covered=====================================");
            need_replan = true;
            inpsct_planner_type_ = 2;
          } else if (info->duration_ - t_cur < fp_->replan_thresh1_) {
            ROS_WARN("Replan: traj fully executed=================================");
            need_replan = true;
            inpsct_planner_type_ = 2;
          } else if (t_cur > fp_->replan_thresh3_) {
            ROS_WARN("Replan: periodic call=======================================");
            need_replan = true;
            inpsct_planner_type_ = 2;
          }

          if (need_replan) {
            if (expl_manager_->updateFrontierStruct(fd_->odom_pos_) != 0) {
              // Update frontier and plan new motion
              thread vis_thread(&FastExplorationFSM::visualize, this, 1);
              vis_thread.detach();
              transitState(PLAN_TRAJ, "FSM");
            } else {
              // No frontier detected, finish exploration
              fd_->last_check_frontier_time_ = ros::Time::now();
              transitState(IDLE, "FSM");
              ROS_WARN("[FSM] Idle since no frontier is detected");
              fd_->static_state_ = true;
              replan_pub_.publish(std_msgs::Empty());
              visualize(1);
            }
          }
        }
      } else {
        // Check if reach goal
        auto pos = info->position_traj_.evaluateDeBoorT(t_cur);
        if ((pos - expl_manager_->ed_->next_pos_).norm() < 1.0) {
          replan_pub_.publish(std_msgs::Empty());
          clearVisMarker();
          transitState(FINISH, "FSM");
          return;
        }
        if (t_cur > fp_->replan_thresh3_ || info->duration_ - t_cur < fp_->replan_thresh1_) {
          // Replan for going back
          replan_pub_.publish(std_msgs::Empty());
          transitState(PLAN_TRAJ, "FSM");
          thread vis_thread(&FastExplorationFSM::visualize, this, 1);
          vis_thread.detach();
        }
      }

      break;
    }
  }
}

/**
 * @brief PLAN_TRAJ调用 规划到下一个状态（位置、速度、姿态 ）的轨迹
 */
int FastExplorationFSM::callCoveragePlanner() {
  ros::Time time_r = ros::Time::now() + ros::Duration(fp_->replan_time_);
  // ROS_WARN("id %d, planRapidCoverageMotion begin", getId());
  int res;
  if (fd_->avoid_collision_ || fd_->go_back_) {  // Only replan trajectory
    res = expl_manager_->planTrajToView(fd_->start_pt_, fd_->start_vel_, fd_->start_acc_,
        fd_->start_yaw_, expl_manager_->ed_->next_pos_, expl_manager_->ed_->next_yaw_);
    fd_->avoid_collision_ = false;
  } else {  // Do full planning normally 自主探索，包含planTrajToView
    res = expl_manager_->planRapidCoverageMotion(
        fd_->start_pos_, fd_->start_pt_, fd_->start_vel_, fd_->start_acc_, fd_->start_yaw_);
  }

  if (res == SUCCEED) {
    ROS_WARN("id %d, planRapidCoverageMotion succeed", getId());
    auto info = &planner_manager_->local_data_;
    info->start_time_ = (ros::Time::now() - time_r).toSec() > 0 ? ros::Time::now() : time_r;

    bspline::Bspline bspline;
    bspline.order = planner_manager_->pp_.bspline_degree_;
    bspline.start_time = info->start_time_;
    bspline.traj_id = info->traj_id_;
    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    for (int i = 0; i < pos_pts.rows(); ++i) {
      geometry_msgs::Point pt;
      pt.x = pos_pts(i, 0);
      pt.y = pos_pts(i, 1);
      pt.z = pos_pts(i, 2);
      bspline.pos_pts.push_back(pt);
    }
    Eigen::VectorXd knots = info->position_traj_.getKnot();
    for (int i = 0; i < knots.rows(); ++i) {
      bspline.knots.push_back(knots(i));
    }
    Eigen::MatrixXd yaw_pts = info->yaw_traj_.getControlPoint();
    for (int i = 0; i < yaw_pts.rows(); ++i) {
      double yaw = yaw_pts(i, 0);
      bspline.yaw_pts.push_back(yaw);
    }
    bspline.yaw_dt = info->yaw_traj_.getKnotSpan();
    fd_->newest_traj_ = bspline;
    ROS_WARN("id %d, bspline succeed", getId());
  }

  return res;
}

/**
 * @brief 利用类的数据自动绘图
 */
void FastExplorationFSM::visualize(int content) {
  // content 1: frontier; 2 paths & trajs
  auto info = &planner_manager_->local_data_;
  // auto plan_data = &planner_manager_->plan_data_;
  auto ed_ptr = expl_manager_->ed_;

  // auto getColorVal = [&](const int& id, const int& num, const int& drone_id) {
  //   double a = (drone_id - 1) / double(num + 1);
  //   double b = 1 / double(num + 1);
  //   return a + b * double(id) / ed_ptr->frontiers_.size();
  // };

  if (content == 1) {
    // Draw frontier
    static int last_ftr_num = 0;
    // static int last_dftr_num = 0;
    for (int i = 0; i < ed_ptr->frontiers_.size(); ++i) {
      visualization_->drawCubes(ed_ptr->frontiers_[i], 0.1,
          visualization_->getColor(double(i) / ed_ptr->frontiers_.size(), 0.4), "frontier", i, 4);

      // getColorVal(i, expl_manager_->ep_->drone_num_, expl_manager_->ep_->drone_id_)
      // double(i) / ed_ptr->frontiers_.size()

      // visualization_->drawBox(ed_ptr->frontier_boxes_[i].first,
      // ed_ptr->frontier_boxes_[i].second,
      //     Vector4d(0.5, 0, 1, 0.3), "frontier_boxes", i, 4);
    }
    for (int i = ed_ptr->frontiers_.size(); i < last_ftr_num; ++i) {
      visualization_->drawCubes({}, 0.1, Vector4d(0, 0, 0, 1), "frontier", i, 4);
      // visualization_->drawBox(Vector3d(0, 0, 0), Vector3d(0, 0, 0), Vector4d(1, 0, 0, 0.3),
      // "frontier_boxes", i, 4);
    }
    last_ftr_num = ed_ptr->frontiers_.size();

    // for (int i = 0; i < ed_ptr->dead_frontiers_.size(); ++i)
    //   visualization_->drawCubes(
    //       ed_ptr->dead_frontiers_[i], 0.1, Vector4d(0, 0, 0, 0.5), "dead_frontier", i, 4);
    // for (int i = ed_ptr->dead_frontiers_.size(); i < last_dftr_num; ++i)
    //   visualization_->drawCubes({}, 0.1, Vector4d(0, 0, 0, 0.5), "dead_frontier", i, 4);
    // last_dftr_num = ed_ptr->dead_frontiers_.size();

    // // Draw updated box
    // Vector3d bmin, bmax;
    // planner_manager_->edt_environment_->sdf_map_->getUpdatedBox(bmin, bmax, false);
    // visualization_->drawBox(
    //     (bmin + bmax) / 2.0, bmax - bmin, Vector4d(0, 1, 0, 0.3), "updated_box", 0, 4);

    // vector<Eigen::Vector3d> bmins, bmaxs;
    // planner_manager_->edt_environment_->sdf_map_->mm_->getChunkBoxes(bmins, bmaxs, false);
    // for (int i = 0; i < bmins.size(); ++i) {
    //   visualization_->drawBox((bmins[i] + bmaxs[i]) / 2.0, bmaxs[i] - bmins[i],
    //       Vector4d(0, 1, 1, 0.3), "updated_box", i + 1, 4);
    // }

  } else if (content == 2) {

    // Hierarchical grid and global tour --------------------------------
    // vector<Eigen::Vector3d> pts1, pts2;
    // expl_manager_->uniform_grid_->getPath(pts1, pts2);
    // visualization_->drawLines(pts1, pts2, 0.05, Eigen::Vector4d(1, 0.3, 0, 1), "partition", 0,
    // 6);

    if (expl_manager_->ep_->drone_id_ == 1) {
      vector<Eigen::Vector3d> pts1, pts2;
      expl_manager_->hgrid_->getGridMarker(pts1, pts2);
      visualization_->drawLines(pts1, pts2, 0.05, Eigen::Vector4d(1, 0, 1, 0.5), "partition", 1, 6);

      vector<Eigen::Vector3d> pts;
      vector<string> texts;
      expl_manager_->hgrid_->getGridMarker2(pts, texts);
      static int last_text_num = 0;
      for (int i = 0; i < pts.size(); ++i) {
        visualization_->drawText(pts[i], texts[i], 1, Eigen::Vector4d(0, 0, 0, 1), "text", i, 6);
      }
      for (int i = pts.size(); i < last_text_num; ++i) {
        visualization_->drawText(
            Eigen::Vector3d(0, 0, 0), string(""), 1, Eigen::Vector4d(0, 0, 0, 1), "text", i, 6);
      }
      last_text_num = pts.size();

      // // Pub hgrid to ground node
      // exploration_manager::HGrid hgrid;
      // hgrid.stamp = ros::Time::now().toSec();
      // for (int i = 0; i < pts1.size(); ++i) {
      //   geometry_msgs::Point pt1, pt2;
      //   pt1.x = pts1[i][0];
      //   pt1.y = pts1[i][1];
      //   pt1.z = pts1[i][2];
      //   hgrid.points1.push_back(pt1);
      //   pt2.x = pts2[i][0];
      //   pt2.y = pts2[i][1];
      //   pt2.z = pts2[i][2];
      //   hgrid.points2.push_back(pt2);
      // }
      // hgrid_pub_.publish(hgrid);
    }

    auto grid_tour = expl_manager_->ed_->grid_tour_;
    // auto grid_tour = expl_manager_->ed_->grid_tour2_;
    // for (auto& pt : grid_tour) pt = pt + trans;

    visualization_->drawLines(grid_tour, 0.05,
        PlanningVisualization::getColor(
            (expl_manager_->ep_->drone_id_ - 1) / double(expl_manager_->ep_->drone_num_)),
        "grid_tour", 0, 6);

    // Publish grid tour to ground node
    exploration_manager::GridTour tour;
    for (int i = 0; i < grid_tour.size(); ++i) {
      geometry_msgs::Point point;
      point.x = grid_tour[i][0];
      point.y = grid_tour[i][1];
      point.z = grid_tour[i][2];
      tour.points.push_back(point);
    }
    tour.drone_id = expl_manager_->ep_->drone_id_;
    tour.stamp = ros::Time::now().toSec();
    grid_tour_pub_.publish(tour);

    // visualization_->drawSpheres(
    //     expl_manager_->ed_->grid_tour_, 0.3, Eigen::Vector4d(0, 1, 0, 1), "grid_tour", 1, 6);
    // visualization_->drawLines(
    //     expl_manager_->ed_->grid_tour2_, 0.05, Eigen::Vector4d(0, 1, 0, 0.5), "grid_tour", 2, 6);

    // Top viewpoints and frontier tour-------------------------------------

    // visualization_->drawSpheres(ed_ptr->points_, 0.2, Vector4d(0, 0.5, 0, 1), "points", 0, 6);
    // visualization_->drawLines(
    //     ed_ptr->points_, ed_ptr->views_, 0.05, Vector4d(0, 1, 0.5, 1), "view", 0, 6);
    // visualization_->drawLines(
    //     ed_ptr->points_, ed_ptr->averages_, 0.03, Vector4d(1, 0, 0, 1), "point-average", 0, 6);

    // auto frontier = ed_ptr->frontier_tour_;
    // for (auto& pt : frontier) pt = pt + trans;
    // visualization_->drawLines(frontier, 0.07,
    //     PlanningVisualization::getColor(
    //         (expl_manager_->ep_->drone_id_ - 1) / double(expl_manager_->ep_->drone_num_), 0.6),
    //     "frontier_tour", 0, 6);

    // for (int i = 0; i < ed_ptr->other_tours_.size(); ++i) {
    //   visualization_->drawLines(
    //       ed_ptr->other_tours_[i], 0.07, Eigen::Vector4d(0, 0, 1, 1), "other_tours", i, 6);
    // }

    // Locally refined viewpoints and refined tour-------------------------------

    // visualization_->drawSpheres(
    //     ed_ptr->refined_points_, 0.2, Vector4d(0, 0, 1, 1), "refined_pts", 0, 6);
    // visualization_->drawLines(
    //     ed_ptr->refined_points_, ed_ptr->refined_views_, 0.05, Vector4d(0.5, 0, 1, 1),
    //     "refined_view", 0, 6);
    // visualization_->drawLines(
    //     ed_ptr->refined_tour_, 0.07,
    //     PlanningVisualization::getColor(
    //         (expl_manager_->ep_->drone_id_ - 1) / double(expl_manager_->ep_->drone_num_), 0.6),
    //     "refined_tour", 0, 6);

    // visualization_->drawLines(ed_ptr->refined_views1_, ed_ptr->refined_views2_, 0.04, Vector4d(0,
    // 0, 0, 1),
    //                           "refined_view", 0, 6);
    // visualization_->drawLines(ed_ptr->refined_points_, ed_ptr->unrefined_points_, 0.05,
    // Vector4d(1, 1, 0, 1),
    //                           "refine_pair", 0, 6);
    // for (int i = 0; i < ed_ptr->n_points_.size(); ++i)
    //   visualization_->drawSpheres(ed_ptr->n_points_[i], 0.1,
    //                               visualization_->getColor(double(ed_ptr->refined_ids_[i]) /
    //                               ed_ptr->frontiers_.size()),
    //                               "n_points", i, 6);
    // for (int i = ed_ptr->n_points_.size(); i < 15; ++i)
    //   visualization_->drawSpheres({}, 0.1, Vector4d(0, 0, 0, 1), "n_points", i, 6);

    // Trajectory-------------------------------------------

    // visualization_->drawSpheres(
    //     { ed_ptr->next_goal_ /* + trans */ }, 0.3, Vector4d(0, 0, 1, 1), "next_goal", 0, 6);

    // vector<Eigen::Vector3d> next_yaw_vis;
    // next_yaw_vis.push_back(ed_ptr->next_goal_ /* + trans */);
    // next_yaw_vis.push_back(
    //     ed_ptr->next_goal_ /* + trans */ +
    //     2.0 * Eigen::Vector3d(cos(ed_ptr->next_yaw_), sin(ed_ptr->next_yaw_), 0));
    // visualization_->drawLines(next_yaw_vis, 0.1, Eigen::Vector4d(0, 0, 1, 1), "next_goal", 1, 6);
    // visualization_->drawSpheres(
    //     { ed_ptr->next_pos_ /* + trans */ }, 0.3, Vector4d(0, 1, 0, 1), "next_pos", 0, 6);

    // Eigen::MatrixXd ctrl_pt = info->position_traj_.getControlPoint();
    // for (int i = 0; i < ctrl_pt.rows(); ++i) {
    //   for (int j = 0; j < 3; ++j) ctrl_pt(i, j) = ctrl_pt(i, j) + trans[j];
    // }
    // NonUniformBspline position_traj(ctrl_pt, 3, info->position_traj_.getKnotSpan());

    visualization_->drawBspline(info->position_traj_, 0.1,
        PlanningVisualization::getColor(
            (expl_manager_->ep_->drone_id_ - 1) / double(expl_manager_->ep_->drone_num_)),
        false, 0.15, Vector4d(1, 1, 0, 1));

    // visualization_->drawLines(
    //     expl_manager_->ed_->path_next_goal_, 0.1, Eigen::Vector4d(0, 1, 0, 1), "astar", 0, 6);
    // visualization_->drawSpheres(
    //     expl_manager_->ed_->kino_path_, 0.1, Eigen::Vector4d(0, 0, 1, 1), "kino", 0, 6);
    // visualization_->drawSpheres(plan_data->kino_path_, 0.1, Vector4d(1, 0, 1, 1), "kino_path", 0,
    // 0); visualization_->drawLines(ed_ptr->path_next_goal_, 0.05, Vector4d(0, 1, 1, 1),
    // "next_goal", 1, 6);

    // // Draw trajs of other drones
    // vector<NonUniformBspline> trajs;
    // planner_manager_->swarm_traj_data_.getValidTrajs(trajs);
    // for (int k = 0; k < trajs.size(); ++k) {
    //   visualization_->drawBspline(trajs[k], 0.1, Eigen::Vector4d(1, 1, 0, 1), false, 0.15,
    //       Eigen::Vector4d(0, 0, 1, 1), k + 1);
    // }
  } else if (content == 3) {
    // draw my islands
    vector<Island> islands;
    const auto& local_islands = expl_manager_->ed_->swarm_state_[getId() - 1].island_buffer;
    for (const auto& pair : local_islands) {
      islands.push_back(pair.second);  // 怎么防止撞库，暂时不管
    }
    vector<vector<Eigen::Vector2d>> ploys;
    for (const auto& is : islands) {
      auto& bmax = is.box[1];
      auto& bmin = is.box[0];
      vector<Eigen::Vector2d> ploy;

      Eigen::Vector2d a(bmin(0), bmax(1));
      ploy.push_back(a);
      Eigen::Vector2d b(bmax(0), bmax(1));
      ploy.push_back(b);
      Eigen::Vector2d c(bmax(0), bmin(1));
      ploy.push_back(c);
      Eigen::Vector2d d(bmin(0), bmin(1));
      ploy.push_back(d);

      ploys.push_back(ploy);
    }
    vector<Eigen::Vector2d> update_box;
    Eigen::Vector3d bmax, bmin;
    expl_manager_->sdf_map_->getUpdatedBox(bmin, bmax, true);
    Eigen::Vector2d a(bmin(0), bmax(1));
    update_box.push_back(a);
    Eigen::Vector2d b(bmax(0), bmax(1));
    update_box.push_back(b);
    Eigen::Vector2d c(bmax(0), bmin(1));
    update_box.push_back(c);
    Eigen::Vector2d d(bmin(0), bmin(1));
    update_box.push_back(d);
    ploys.push_back(update_box);

    visualization_->drawMultiplePolygons(ploys, 0.1,
        PlanningVisualization::getColor(
            (expl_manager_->ep_->drone_id_ - 1) / double(expl_manager_->ep_->drone_num_)),
        "grid_tour", 7);
  }
}

void FastExplorationFSM::clearVisMarker() {
  for (int i = 0; i < 10; ++i) {
    visualization_->drawCubes({}, 0.1, Vector4d(0, 0, 0, 1), "frontier", i, 4);
    // visualization_->drawCubes({}, 0.1, Vector4d(0, 0, 0, 1), "dead_frontier", i, 4);
    // visualization_->drawBox(Vector3d(0, 0, 0), Vector3d(0, 0, 0), Vector4d(1, 0, 0, 0.3),
    // "frontier_boxes", i, 4);
  }
  // visualization_->drawSpheres({}, 0.2, Vector4d(0, 0.5, 0, 1), "points", 0, 6);
  visualization_->drawLines({}, 0.07, Vector4d(0, 0.5, 0, 1), "frontier_tour", 0, 6);
  visualization_->drawLines({}, 0.07, Vector4d(0, 0.5, 0, 1), "grid_tour", 0, 6);
  // visualization_->drawSpheres({}, 0.2, Vector4d(0, 0, 1, 1), "refined_pts", 0, 6);
  // visualization_->drawLines({}, {}, 0.05, Vector4d(0.5, 0, 1, 1), "refined_view", 0, 6);
  // visualization_->drawLines({}, 0.07, Vector4d(0, 0, 1, 1), "refined_tour", 0, 6);
  visualization_->drawSpheres({}, 0.1, Vector4d(0, 0, 1, 1), "B-Spline", 0, 0);

  // visualization_->drawLines({}, {}, 0.03, Vector4d(1, 0, 0, 1), "current_pose", 0, 6);
}

/**
 * @brief
 * 在WAIT_TRIGGER状态下进行的callback函数,进行可视化部分的初始化。从这里初始化drone1后，pairwise
 * opt从这里初始化drone1后启动
 */
void FastExplorationFSM::frontierCallback(const ros::TimerEvent& e) {
  if (state_ == WAIT_TRIGGER) {
    auto ft = expl_manager_->frontier_finder_;
    auto ed = expl_manager_->ed_;

    // auto getColorVal = [&](const int& id, const int& num, const int& drone_id) {
    //   double a = (drone_id - 1) / double(num + 1);
    //   double b = 1 / double(num + 1);
    //   return a + b * double(id) / ed->frontiers_.size();
    // };

    expl_manager_->updateFrontierStruct(fd_->odom_pos_);
    // cout << "odom: " << fd_->odom_pos_.transpose() << endl;

    if (init_) {  // 分配初始化
      if (getId() == 1) {
        auto& state = expl_manager_->ed_->swarm_state_[getId() - 1];
        vector<int> tmp_id1, tmp_id2;
        expl_manager_->initOneTimeGridAllocation(tmp_id1, tmp_id2);
        expl_manager_->planDisQueue(fd_->odom_pos_, Eigen::Vector3d(0, 0, 0));
        state.allocation_cost =
            expl_manager_->computeGridPathCost(state.pos_, state.grid_ids_, {}, {}, {}, true);
        ROS_WARN("Drone 1 inits allocation!, cost %lf, v %ld", state.allocation_cost,
            state.grid_ids_.size());
      }
      init_ = false;
    }

    // Draw frontier and bounding box
    for (int i = 0; i < ed->frontiers_.size(); ++i) {
      visualization_->drawCubes(ed->frontiers_[i], 0.1,
          visualization_->getColor(double(i) / ed->frontiers_.size(), 0.4), "frontier", i, 4);
    }
    for (int i = ed->frontiers_.size(); i < 50; ++i) {
      visualization_->drawCubes({}, 0.1, Vector4d(0, 0, 0, 1), "frontier", i, 4);
    }

    // Draw grid tour
  } else if (state_ != INIT && state_ != FINISH) {
    if (fp_->drone_type_ == 1) {
      // // 修改 grid排除机制
      // auto& dis_queque = expl_manager_->ed_->dis_queue;
      // if (!dis_queque.empty()) {
      //   if ((fd_->odom_pos_ - dis_queque[0].position).norm() < 0.75) {
      //     auto& grid_ids = expl_manager_->ed_->swarm_state_[getId() - 1].grid_ids_;
      //     for(int i = 0 ; i< grid_ids.size(); i++){
      //       if(expl_manager_->hgrid_->getCenter(grid_ids[i]) == dis_queque[0].position){
      //         grid_ids.erase(grid_ids.begin() + i);
      //       }
      //     }
      //     dis_queque.erase(dis_queque.begin());
      //     disqueue_replan = true;  // 重规划
      //   }
      // }
      expl_manager_->updateVisitedGrids(fd_->odom_pos_);
      disqueue_replan = updateLocalDisqueue();
      Eigen::Vector3d growth_vector;
      int tmp = callLocalIslandUpdater(growth_vector);
      if(growth_vector.norm() >= 1) {
        expl_manager_->planDisQueue(fd_->odom_pos_, growth_vector);
        disqueue_replan = true;  // 重规划
      } 
      if (tmp == 1) {
        visualize(3);
        // ROS_WARN("id %d :island! center1(%lf, %lf)", getId(), center1(0), center1(1));
      } else if (tmp == 0) {
        ROS_WARN("[High-Drone] no island");
      } else if (tmp == -1) {
        ROS_WARN("[High-Drone] island wrong");
      }
    }
    callOptEndChecker();
  }
}

/// @brief 根据当前位置判断是否应该选择下一个导航点
/// @return 是否产生了disqueue更新
bool FastExplorationFSM::updateLocalDisqueue(){
  bool update_flag = false;
  auto& dis_queque = expl_manager_->ed_->dis_queue;
  if (!dis_queque.empty()) {
    if ((fd_->odom_pos_ - dis_queque[0].pos).norm() < 0.75) {  // 0.75
      // auto& grid_ids = expl_manager_->ed_->swarm_state_[getId() - 1].grid_ids_;
      dis_queque.erase(dis_queque.begin());
      update_flag = true;  // 重规划
    }
  }
  return update_flag;
}

int FastExplorationFSM::callLocalIslandUpdater(Eigen::Vector3d& growth_vector) {
  std::cout << "[High-Drone] ifinder" << std::endl;
  // double begin_time = ros::Time::now().toSec();
  // auto tmp = ifinder_->searchCannyUpdatedIslands();
  // auto tmp = ifinder_->searchSVDUpdatedIslands(); //入口
  auto tmp = ifinder_->searchMSERUpdatedIslands();
  vector<int> dropped_ids;
  ifinder_->refineLocalIslands(dropped_ids);
  map<int, Island> new_islands;
  ifinder_->getIslandToPub(new_islands);
  expl_manager_->ed_->ground_height_ = 0;

  auto& local_buffer = expl_manager_->ed_->swarm_state_[getId() - 1].island_buffer;
  ifinder_->getAllIslandBoxs(local_buffer);  // 更新本地island库
  
  growth_vector = Eigen::Vector3d(0, 0, 0);
  if (!new_islands.empty()) {
    for (const auto& pair : new_islands) {
      //local_buffer[pair.first] = pair.second;
      auto tmp = ifinder_->getGrowthVector(pair.first, fd_->odom_pos_);
      if(tmp.norm() > 0.02)  // 如果增长向量大于0.02
        growth_vector += tmp;
    }
     //testhigh
    //cout << "id " << getId() << "find newe island, switch to OPT" << endl;
    // need_opt = true;  // 需要优化
  }

  return tmp;
}

/// @brief 定时广播所有本机管理的island
void FastExplorationFSM::islandTimerCallback(const ros::TimerEvent& e) {
  if (state_ == WAIT_TRIGGER || state_ == INIT) return;
  if (fp_->drone_type_ == 0) return;
  // 发布island array消息
  bool have_stable = false;
  map<int, Island> island_buffer;
  ifinder_->getAllIslandBoxs(island_buffer);
  exploration_manager::IslandArray island_msg;
  island_msg.header.stamp = ros::Time::now();
  island_msg.header.frame_id = "world";
  island_msg.drone_id = getId();
  for (auto& pair : island_buffer) {
    auto& myisland = pair.second;
    if (myisland.box.size() != 2) {
      ROS_ERROR("WRONG ISLAND islandTimerCallback");
      continue;
    }
    exploration_manager::IslandBox islandbox;
    islandbox.island_id = pair.first;
    // islandbox.status = (myisland.state == NEW) ? exploration_manager::IslandBox::STATUS_NEW :
    //                                              exploration_manager::IslandBox::STATUS_MERGED;
    islandbox.bmin.x = myisland.box[0](0);
    islandbox.bmin.y = myisland.box[0](1);
    islandbox.bmin.z = myisland.box[0](2);

    islandbox.bmax.x = myisland.box[1](0);
    islandbox.bmax.y = myisland.box[1](1);
    islandbox.bmax.z = myisland.box[1](2);
    if (myisland.state == MERGED) {
      islandbox.merged_ids = myisland.merged_ids;
    }
    island_msg.islands.push_back(islandbox);
    // ROS_WARN("IS IT STABLE? %d", myisland.state);
    if (ifinder_->isIslandStable(pair.first)) {
      have_stable = true;  // 有稳定的island
      ROS_WARN("set completed island %d", pair.first);
      ifinder_->setCompleted(pair.first);  // 稳定的island发布出去
    }
  }
  if (have_stable) {
    need_opt = true;  // 需要重新优化
    // transitState(OPT, "islandTimerCallback");
    ROS_WARN("id %d: have found nnew stable island, need re-opt", getId());
  }
  island_pub_.publish(island_msg);
}

/// @brief 接受其他无人机管理的island，同步到本地的island库
/// @param msg
void FastExplorationFSM::islandMsgCallback(const exploration_manager::IslandArrayConstPtr& msg) {
  if (msg->drone_id == getId()) {
    return;  // 不处理自己的island消息
  }
  map<int, Island> from_island_buffer;
  const auto& from_island = msg->islands;
  for (const auto& islandbox : from_island) {
    Island tmp;

    tmp.box.push_back(Eigen::Vector3d(islandbox.bmin.x, islandbox.bmin.y, islandbox.bmin.z));
    tmp.box.push_back(Eigen::Vector3d(islandbox.bmax.x, islandbox.bmax.y, islandbox.bmax.z));
    tmp.state = COMPLETED;
    from_island_buffer[islandbox.island_id] = tmp;
  }
  ifinder_->updateGlobalIslandMaps(from_island_buffer, msg->drone_id);
}

/**
 * @brief 接受到触发源的回调函数，开始任务。初始化起点
 */
void FastExplorationFSM::triggerCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
  if (state_ != WAIT_TRIGGER) return;
  if (init_) {
    ROS_WARN("Allocation isn't initialized!");
    return;
  }

  fd_->trigger_ = true;
  cout << "Triggered!" << endl;
  fd_->start_pos_ = fd_->odom_pos_;  // 初始化起点
  ROS_WARN_STREAM("Start expl pos: " << fd_->start_pos_.transpose());

  expl_manager_->updateFrontierStruct(fd_->odom_pos_);
  transitState(OPT, "triggerCallback");
}

void FastExplorationFSM::safetyCallback(const ros::TimerEvent& e) {
  if (state_ == EXPL_STATE::EXEC_TRAJ) {
    // Check safety and trigger replan if necessary
    double dist;
    bool safe = planner_manager_->checkTrajCollision(dist);
    if (!safe) {
      ROS_WARN("Replan: collision detected==================================");
      fd_->avoid_collision_ = true;
      transitState(PLAN_TRAJ, "safetyCallback");
    }
  }
}

/**
 * @brief 里程计（odometry）数据更新的回调函数。时刻保持位置、速度、方向数据更新
 */
void FastExplorationFSM::odometryCallback(const nav_msgs::OdometryConstPtr& msg) {
  fd_->odom_pos_(0) = msg->pose.pose.position.x;
  fd_->odom_pos_(1) = msg->pose.pose.position.y;
  fd_->odom_pos_(2) = msg->pose.pose.position.z;

  fd_->odom_vel_(0) = msg->twist.twist.linear.x;
  fd_->odom_vel_(1) = msg->twist.twist.linear.y;
  fd_->odom_vel_(2) = msg->twist.twist.linear.z;

  fd_->odom_orient_.w() = msg->pose.pose.orientation.w;
  fd_->odom_orient_.x() = msg->pose.pose.orientation.x;
  fd_->odom_orient_.y() = msg->pose.pose.orientation.y;
  fd_->odom_orient_.z() = msg->pose.pose.orientation.z;

  Eigen::Vector3d rot_x = fd_->odom_orient_.toRotationMatrix().block<3, 1>(0, 0);
  fd_->odom_yaw_ = atan2(rot_x(1), rot_x(0));

  if (!fd_->have_odom_) {
    fd_->have_odom_ = true;
    fd_->fsm_init_time_ = ros::Time::now();
  }
}

void FastExplorationFSM::transitState(EXPL_STATE new_state, string pos_call) {
  int pre_s = int(state_);
  state_ = new_state;
  ROS_INFO_STREAM("[" + pos_call + "]: Drone "
                  << getId()
                  << " from " + fd_->state_str_[pre_s] + " to " + fd_->state_str_[int(new_state)]);
}

/**
 * @brief 周期性广播本机state
 */
void FastExplorationFSM::droneStateTimerCallback(const ros::TimerEvent& e) {
  // Broadcast own state periodically
  exploration_manager::DroneState msg;
  msg.drone_id = getId();

  auto& state = expl_manager_->ed_->swarm_state_[msg.drone_id - 1];

  if (fd_->static_state_) {
    state.pos_ = fd_->odom_pos_;
    state.vel_ = fd_->odom_vel_;
    state.yaw_ = fd_->odom_yaw_;
  } else {
    LocalTrajData* info = &planner_manager_->local_data_;
    double t_r = (ros::Time::now() - info->start_time_).toSec();
    state.pos_ = info->position_traj_.evaluateDeBoorT(t_r);
    state.vel_ = info->velocity_traj_.evaluateDeBoorT(t_r);
    state.yaw_ = info->yaw_traj_.evaluateDeBoorT(t_r)[0];
  }
  state.stamp_ = ros::Time::now().toSec();
  msg.pos = { float(state.pos_[0]), float(state.pos_[1]), float(state.pos_[2]) };
  msg.vel = { float(state.vel_[0]), float(state.vel_[1]), float(state.vel_[2]) };
  msg.yaw = state.yaw_;
  for (auto id : state.grid_ids_) msg.grid_ids.push_back(id);
  msg.recent_attempt_time = state.recent_attempt_time_;
  msg.stamp = state.stamp_;
  msg.allocation_cost = state.allocation_cost;
  //-------------------------------------------------------------
  // 发布自己已经有的 island array消息
  auto& local_islands = expl_manager_->ed_->swarm_state_[getId() - 1].island_buffer;
  if (!local_islands.empty()) {
    exploration_manager::IslandArray island_msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "world";
    for (auto& pair : local_islands) {  // testhigh 同步所有island
      auto& myisland = pair.second;
      exploration_manager::IslandBox islandbox;
      islandbox.island_id = pair.first;
      islandbox.bmin.x = myisland.box[0](0);
      islandbox.bmin.y = myisland.box[0](1);
      islandbox.bmin.z = myisland.box[0](2);

      islandbox.bmax.x = myisland.box[1](0);
      islandbox.bmax.y = myisland.box[1](1);
      islandbox.bmax.z = myisland.box[1](2);
      msg.islands.push_back(islandbox);
    }
  }
  drone_state_pub_.publish(msg);
  // island_pub_.publish(island_msg);
}

/**
 * @brief 接受到droneStateMsg后
 */
void FastExplorationFSM::droneStateMsgCallback(const exploration_manager::DroneStateConstPtr& msg) {
  // Update other drones' states
  if (msg->drone_id == getId()) return;

  // Simulate swarm communication loss
  Eigen::Vector3d msg_pos(msg->pos[0], msg->pos[1], msg->pos[2]);
  // if ((msg_pos - fd_->odom_pos_).norm() > 6.0) return;

  // 同步对于对方机的了解
  auto& drone_state = expl_manager_->ed_->swarm_state_[msg->drone_id - 1];
  if (drone_state.stamp_ + 1e-4 >= msg->stamp) return;  // Avoid unordered msg

  drone_state.pos_ = Eigen::Vector3d(msg->pos[0], msg->pos[1], msg->pos[2]);
  drone_state.vel_ = Eigen::Vector3d(msg->vel[0], msg->vel[1], msg->vel[2]);
  drone_state.yaw_ = msg->yaw;
  drone_state.grid_ids_.clear();
  for (auto id : msg->grid_ids) drone_state.grid_ids_.push_back(id);
  drone_state.stamp_ = msg->stamp;
  drone_state.recent_attempt_time_ = msg->recent_attempt_time;

  drone_state.allocation_cost = msg->allocation_cost;

  // std::cout << "Drone " << getId() << " get drone " << int(msg->drone_id) << "'s state" <<
  // std::endl; std::cout << drone_state.pos_.transpose() << std::endl;

  auto& from_island_buffer = expl_manager_->ed_->swarm_state_[msg->drone_id - 1].island_buffer;
  from_island_buffer.clear();  // 由于直接同步所有island，覆盖即可
  for (const auto& msg_island : msg->islands) {
    int id = msg_island.island_id;
    Island island;
    // island.state = (msg_island.status == exploration_manager::IslandBox::STATUS_NEW) ? NEW :
    // MERGED;
    island.box.push_back(Eigen::Vector3d(msg_island.bmin.x, msg_island.bmin.y, msg_island.bmin.z));
    island.box.push_back(Eigen::Vector3d(msg_island.bmax.x, msg_island.bmax.y, msg_island.bmax.z));
    from_island_buffer[id] = island;
  }

  expl_manager_->updateVisitedGrids(fd_->odom_pos_);

  callOptEndChecker();
}

/// @brief 进行优化结束条件检查，符合条件则转移到PLAN_TRAJ状态
void FastExplorationFSM::callOptEndChecker() {
  if (state_ != OPT) return;  // 没当接受到一个dronestate，检查是否继续opt
    // ROS_WARN_THROTTLE(1.0, "[MsgCallback] doing opt check. id %d", getId());

    const auto& states = expl_manager_->ed_->swarm_state_;
    double total = 0;
    int n = 0;

    for (auto state : states) {
      if (state.allocation_cost > 0) {
        total += state.allocation_cost;
        n++;
      }
    }
    double avrg = total / n;
    // double avrg = total;
    double stdev = 0;
    // ROS_WARN("[MsgCallback] doing opt check. id %d, avrg = %lf, total = %lf, EXPECT N = %d", getId(), avrg, total, expl_manager_->ep_->drone_num_);
    if (avrg > 0 && n == expl_manager_->ep_->drone_num_) {
      // expl_manager_->planDisQueue(fd_->start_pos_);
      ROS_WARN("opt done: drone %d, avrg = %lf, total num: %d", getId(), avrg, n);
      transitState(PLAN_TRAJ, "FSM");

      //   std::for_each(states.begin(), states.end(), [&](DroneState st) {
      //     if (st.allocation_cost > 0)
      //       stdev += (st.allocation_cost - avrg) * (st.allocation_cost - avrg);
      //     else
      //       stdev += avrg * avrg;
      //   });
      //   double cv = sqrt(stdev) / avrg;  // 考察变异系数

      //   if (/*cv > 0 && cv <= 0.5 && */ total <= allocation_bar * 1.1 && total > 0) {
      //     ROS_WARN("opt done: drone %d, [opt done] cv = %lf, total = %lf.", getId(), cv,
      //     total); expl_manager_->planDisQueue(); transitState(PLAN_TRAJ, "FSM");
      //   } else {
      //     ROS_WARN_THROTTLE(1.0, "continue opt: cv = %lf, total = %lf, expect: %lf - / under
      //     %lf -",
      //         getId(), cv, total, 2.8, allocation_bar * 1.1);
      //   }
      // } else {
      //   ROS_WARN("[WRONG] id: %d, avrg = %lf, total num: %d", getId(), avrg, n);
  }
}

  /**
   * @brief 每隔一段时间基于已有信息进行pairwise规划
   */
  void FastExplorationFSM::optTimerCallback(const ros::TimerEvent& e) {
    if (state_ != OPT) return;

    // Select nearby drone not interacting with recently
    auto& states = expl_manager_->ed_->swarm_state_;
    auto& state1 = states[getId() - 1];  // 本无人机的state
    // bool urgent = (state1.grid_ids_.size() <= 1 /* && !state1.grid_ids_.empty() */);
    // bool urgent = state1.grid_ids_.empty();
    auto tn = ros::Time::now().toSec();

    // Avoid frequent attempt
    if (tn - state1.recent_attempt_time_ < fp_->attempt_interval_) return;

    int select_id = -1;
    double max_interval = -1.0;

    // 找到一个符合通信要求的对象，接下来的任务分配与它进行
    for (int i = 0; i < states.size(); ++i) {
      if (i + 1 <= getId()) continue;

      // Check if have communication recently
      // or the drone just experience another opt
      // or the drone is interacted with recently /* !urgent &&  */
      // or the candidate drone dominates enough grids
      if (tn - states[i].stamp_ > 0.2) continue;
      if (tn - states[i].recent_attempt_time_ < fp_->attempt_interval_) continue;
      if (tn - states[i].recent_interact_time_ < fp_->pair_opt_interval_) continue;
      if (states[i].grid_ids_.size() + state1.grid_ids_.size() == 0) {
        ROS_WARN("%d : %d, both drone size 0, continue, %d, %d", getId(), i + 1,
            state1.grid_ids_.size(), states[i].grid_ids_.size());
        continue;
      }

      double interval = tn - states[i].recent_interact_time_;
      if (interval <= max_interval) continue;
      select_id = i + 1;
      max_interval = interval;
    }
    if (select_id == -1) {
      ROS_WARN("id %d: no one matches", getId());
      return;
    }

    std::cout << "\nSelect: " << select_id << std::endl;
    ROS_WARN("Pair opt %d & %d", getId(), select_id);

    // Do pairwise optimization with selected drone, allocate the union of their domiance grids
    unordered_map<int, char> opt_ids_map;
    auto& state2 = states[select_id - 1];  // pairwise interaction对象的state
    for (auto id : state1.grid_ids_) opt_ids_map[id] = 1;
    for (auto id : state2.grid_ids_) opt_ids_map[id] = 1;  // 记录加入优化的grid,排除重复

    vector<int> visited_ids;
    expl_manager_->getVisitedGrids(visited_ids);  // testhigh 删除所有已经visited过的
    for (auto visited : visited_ids) {
      if (opt_ids_map.find(visited) != opt_ids_map.end()) {
        opt_ids_map.erase(visited);
      }
    }

    vector<int> opt_ids;
    for (auto pair : opt_ids_map) opt_ids.push_back(pair.first);

    std::cout << "Pair Opt id: ";
    for (auto id : opt_ids) std::cout << id << ", ";
    std::cout << "" << std::endl;

    // Find missed grids to reallocated them
    // vector<int> actives, missed;
    // expl_manager_->hgrid_->getActiveGrids(actives);
    // findUnallocated(actives, missed);
    // std::cout << "Missed: ";
    // for (auto id : missed) std::cout << id << ", ";
    // std::cout << "" << std::endl;
    // opt_ids.insert(opt_ids.end(), missed.begin(), missed.end());

    // Do partition of the grid
    vector<Eigen::Vector3d> positions = { state1.pos_, state2.pos_ };
    vector<Eigen::Vector3d> velocities = { Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0) };
    vector<int> first_ids1, second_ids1, first_ids2, second_ids2;
    if (state_ != WAIT_TRIGGER) {  // 连续性分析准备。得到无人机1、2的前两个粗grid
      expl_manager_->hgrid_->getConsistentGrid(
          state1.grid_ids_, state1.grid_ids_, first_ids1, second_ids1);
      expl_manager_->hgrid_->getConsistentGrid(
          state2.grid_ids_, state2.grid_ids_, first_ids2, second_ids2);
    }

    auto t1 = ros::Time::now();

    vector<int> ego_ids, other_ids;
    expl_manager_->allocateGrids(positions, velocities, { first_ids1, first_ids2 },
        { second_ids1, second_ids2 }, opt_ids, ego_ids, other_ids);

    double alloc_time = (ros::Time::now() - t1).toSec();

    std::cout << "Ego1  : ";
    for (auto id : state1.grid_ids_) std::cout << id << ", ";
    std::cout << "\nOther1: ";
    for (auto id : state2.grid_ids_) std::cout << id << ", ";
    std::cout << "\nEgo2  : ";
    for (auto id : ego_ids) std::cout << id << ", ";
    std::cout << "\nOther2: ";
    for (auto id : other_ids) std::cout << id << ", ";
    std::cout << "" << std::endl;

    // Check results
    double prev_app1 = expl_manager_->computeGridPathCost(state1.pos_, state1.grid_ids_, first_ids1,
        { first_ids1, first_ids2 }, { second_ids1, second_ids2 }, true);
    double prev_app2 = expl_manager_->computeGridPathCost(state2.pos_, state2.grid_ids_, first_ids2,
        { first_ids1, first_ids2 }, { second_ids1, second_ids2 }, true);
    std::cout << "prev cost: " << prev_app1 << ", " << prev_app2 << ", " << prev_app1 + prev_app2
              << std::endl;  // 分配之前的cost
    double cur_app1 = expl_manager_->computeGridPathCost(state1.pos_, ego_ids, first_ids1,
        { first_ids1, first_ids2 }, { second_ids1, second_ids2 }, true);
    double cur_app2 = expl_manager_->computeGridPathCost(state2.pos_, other_ids, first_ids2,
        { first_ids1, first_ids2 }, { second_ids1, second_ids2 }, true);
    std::cout << "cur cost : " << cur_app1 << ", " << cur_app2 << ", " << cur_app1 + cur_app2
              << std::endl;  // 分配过后的cost
    // if (cur_app1 + cur_app2 > prev_app1 + prev_app2 + 0.1) {
    //   ROS_ERROR("Larger cost after reallocation");
    //   if (state_ != WAIT_TRIGGER) {
    //     return;
    //   }
    // }

    if (!state1.grid_ids_.empty() && !ego_ids.empty() &&
        !expl_manager_->hgrid_->isConsistent(state1.grid_ids_[0], ego_ids[0])) {
      ROS_ERROR("Path 1 inconsistent");  // 分配后的路径和原本路径起始grid不连续
    }
    if (!state2.grid_ids_.empty() && !other_ids.empty() &&
        !expl_manager_->hgrid_->isConsistent(state2.grid_ids_[0], other_ids[0])) {
      ROS_ERROR("Path 2 inconsistent");
    }

    // Update ego and other dominace grids
    auto last_ids2 = state2.grid_ids_;

    // Send the result to selected drone and wait for confirmation
    exploration_manager::PairOpt opt;
    opt.from_drone_id = getId();
    opt.to_drone_id = select_id;
    // opt.msg_type = 1;
    opt.stamp = tn;
    for (auto id : ego_ids) opt.ego_ids.push_back(id);
    for (auto id : other_ids) opt.other_ids.push_back(id);

    opt.to_allocation_cost = cur_app2;  // 将计算出的cost同步到对方
    opt.from_allocation_cost = cur_app1;

    for (int i = 0; i < fp_->repeat_send_num_; ++i) opt_pub_.publish(opt);

    // ROS_WARN("Drone %d send opt request to %d, pair opt t: %lf, allocate t: %lf, cur_app1: %lf,
    // cur_app2: %lf, cv: %lf", getId(), select_id, ros::Time::now().toSec() - tn, alloc_time,
    // cur_app1, cur_app2, cv);
    ROS_WARN("Drone %d send opt request to %d, pair opt t: %lf, allocate t: %lf, cur_app1: %lf, "
             "cur_app2: %lf",
        getId(), select_id, ros::Time::now().toSec() - tn, alloc_time, cur_app1, cur_app2);

    // Reserve the result and wait...
    auto ed = expl_manager_->ed_;  // 收尾工作
    ed->ego_ids_ = ego_ids;
    ed->other_ids_ = other_ids;
    ed->pair_opt_stamp_ = opt.stamp;
    ed->wait_response_ = true;
    ed->allocation_cost = cur_app1;
    ed->other_allocation_cost = cur_app2;
    state1.recent_attempt_time_ = tn;
  }

  /**
   * @brief 找到actives中swarm data未分配的grid
   * @param actives
   * @param missed 存放得到的未分配点
   */
  void FastExplorationFSM::findUnallocated(const vector<int>& actives, vector<int>& missed) {
    // Create map of all active
    unordered_map<int, char> active_map;
    for (auto ativ : actives) {
      active_map[ativ] = 1;
    }

    // Remove allocated ones
    for (auto state : expl_manager_->ed_->swarm_state_) {
      for (auto id : state.grid_ids_) {
        if (active_map.find(id) != active_map.end()) {
          active_map.erase(id);
        } else {
          // ROS_ERROR("Inactive grid %d is allocated.", id);
        }
      }
    }

    missed.clear();
    for (auto p : active_map) {
      missed.push_back(p.first);
    }
  }

  /**
   * @brief 接受到opt之后，同步信息，发送opt
   * ack信息。接受到opt消息会将自己强制转为opt状态，来实现一个全局的opt状态同步
   */
  void FastExplorationFSM::optMsgCallback(const exploration_manager::PairOptConstPtr& msg) {
    if (msg->from_drone_id == getId() || msg->to_drone_id != getId()) return;

    // Check stamp to avoid unordered/repeated msg
    if (msg->stamp <= expl_manager_->ed_->pair_opt_stamps_[msg->from_drone_id - 1] + 1e-4) return;
    expl_manager_->ed_->pair_opt_stamps_[msg->from_drone_id - 1] = msg->stamp;

    auto& state1 = expl_manager_->ed_->swarm_state_[msg->from_drone_id - 1];  // 对面
    auto& state2 = expl_manager_->ed_->swarm_state_[getId() - 1];             // 自己

    // auto tn = ros::Time::now().toSec();
    exploration_manager::PairOptResponse response;
    response.from_drone_id = msg->to_drone_id;
    response.to_drone_id = msg->from_drone_id;
    response.stamp = msg->stamp;  // reply with the same stamp for verificaiton

    if (msg->stamp - state2.recent_attempt_time_ < fp_->attempt_interval_) {
      // Just made another pair opt attempt, should reject this attempt to avoid frequent changes
      ROS_WARN("Reject frequent attempt");
      response.status = 2;
    } else {
      // No opt attempt recently, and the grid info between drones are consistent, the pair opt
      // request can be accepted
      response.status = 1;

      // Update from the opt result
      state1.grid_ids_.clear();
      state2.grid_ids_.clear();
      for (auto id : msg->ego_ids) state1.grid_ids_.push_back(id);
      for (auto id : msg->other_ids) state2.grid_ids_.push_back(id);
      expl_manager_->planDisQueue(fd_->odom_pos_, Eigen::Vector3d(0, 0, 0));
      std::cout << "[optMsgCallback]" << expl_manager_->ed_->dis_queue.size() << "grid in disqueue"
                << std::endl;

      disqueue_replan = true;
      need_opt = true;  // 也需要重新优化

      state1.allocation_cost = msg->from_allocation_cost;
      state2.allocation_cost = msg->to_allocation_cost;

      state1.recent_interact_time_ = msg->stamp;
      state2.recent_attempt_time_ = ros::Time::now().toSec();
      expl_manager_->ed_->reallocated_ = true;

      if (state_ == IDLE && !state2.grid_ids_.empty()) {
        transitState(PLAN_TRAJ, "optMsgCallback");
        ROS_WARN("Restart after opt!");
      }

      // if (!check_consistency(tmp1, tmp2)) {
      //   response.status = 2;
      //   ROS_WARN("Inconsistent grid info, reject pair opt");
      // } else {
      // }
    }
    for (int i = 0; i < fp_->repeat_send_num_; ++i) opt_res_pub_.publish(response);
  }

  /**
   * @brief 接受到opt ack的callback函数
   */
  void FastExplorationFSM::optResMsgCallback(const exploration_manager::PairOptResponseConstPtr& msg) {
    if (msg->from_drone_id == getId() || msg->to_drone_id != getId()) return;

    // Check stamp to avoid unordered/repeated msg
    if (msg->stamp <= expl_manager_->ed_->pair_opt_res_stamps_[msg->from_drone_id - 1] + 1e-4)
      return;
    expl_manager_->ed_->pair_opt_res_stamps_[msg->from_drone_id - 1] = msg->stamp;

    auto ed = expl_manager_->ed_;
    // Verify the consistency of pair opt via time stamp
    if (!ed->wait_response_ || fabs(ed->pair_opt_stamp_ - msg->stamp) > 1e-5) return;

    ed->wait_response_ = false;
    ROS_WARN("get response %d", int(msg->status));

    if (msg->status != 1) return;  // Receive 1 for valid opt

    auto& state1 = ed->swarm_state_[getId() - 1];
    auto& state2 = ed->swarm_state_[msg->from_drone_id - 1];
    state1.grid_ids_ = ed->ego_ids_;
    state2.grid_ids_ = ed->other_ids_;
    expl_manager_->planDisQueue(fd_->odom_pos_, Eigen::Vector3d(0, 0, 0));
    disqueue_replan = true;

    // 更新自己的allocation_cost_ 数据
    state1.allocation_cost = ed->allocation_cost;
    state2.allocation_cost = ed->other_allocation_cost;
    state2.recent_interact_time_ = ros::Time::now().toSec();
    ed->reallocated_ = true;

    if (state_ == IDLE && !state1.grid_ids_.empty()) {
      transitState(PLAN_TRAJ, "optResMsgCallback");
      ROS_WARN("Restart after opt!");
    }
  }

  void FastExplorationFSM::swarmTrajCallback(const bspline::BsplineConstPtr& msg) {
    // Get newest trajs from other drones, for inter-drone collision avoidance
    auto& sdat = planner_manager_->swarm_traj_data_;

    // Ignore self trajectory
    if (msg->drone_id == sdat.drone_id_) return;

    // Ignore outdated trajectory
    if (sdat.receive_flags_[msg->drone_id - 1] == true &&
        msg->start_time.toSec() <= sdat.swarm_trajs_[msg->drone_id - 1].start_time_ + 1e-3)
      return;

    // Convert the msg to B-spline
    Eigen::MatrixXd pos_pts(msg->pos_pts.size(), 3);
    Eigen::VectorXd knots(msg->knots.size());
    for (int i = 0; i < msg->knots.size(); ++i) knots(i) = msg->knots[i];

    for (int i = 0; i < msg->pos_pts.size(); ++i) {
      pos_pts(i, 0) = msg->pos_pts[i].x;
      pos_pts(i, 1) = msg->pos_pts[i].y;
      pos_pts(i, 2) = msg->pos_pts[i].z;
    }

    // // Transform of drone's basecoor, optional step (skip if use swarm_pilot)
    // Eigen::Vector4d tf;
    // planner_manager_->edt_environment_->sdf_map_->getBaseCoor(msg->drone_id, tf);
    // double yaw = tf[3];
    // Eigen::Matrix3d rot;
    // rot << cos(yaw), -sin(yaw), 0, sin(yaw), cos(yaw), 0, 0, 0, 1;
    // Eigen::Vector3d trans = tf.head<3>();
    // for (int i = 0; i < pos_pts.rows(); ++i) {
    //   Eigen::Vector3d tmp = pos_pts.row(i);
    //   tmp = rot * tmp + trans;
    //   pos_pts.row(i) = tmp;
    // }

    sdat.swarm_trajs_[msg->drone_id - 1].setUniformBspline(pos_pts, msg->order, 0.1);
    sdat.swarm_trajs_[msg->drone_id - 1].setKnot(knots);
    sdat.swarm_trajs_[msg->drone_id - 1].start_time_ = msg->start_time.toSec();
    sdat.receive_flags_[msg->drone_id - 1] = true;

    if (state_ == EXEC_TRAJ) {
      // Check collision with received trajectory
      if (!planner_manager_->checkSwarmCollision(msg->drone_id)) {
        ROS_ERROR("Drone %d collide with drone %d.", sdat.drone_id_, msg->drone_id);
        fd_->avoid_collision_ = true;
        transitState(PLAN_TRAJ, "swarmTrajCallback");
      }
    }
  }

  void FastExplorationFSM::swarmTrajTimerCallback(const ros::TimerEvent& e) {
    // Broadcast newest traj of this drone to others
    if (state_ == EXEC_TRAJ) {
      swarm_traj_pub_.publish(fd_->newest_traj_);

    } else if (state_ == WAIT_TRIGGER) {
      // Publish a virtual traj at current pose, to avoid collision
      bspline::Bspline bspline;
      bspline.order = planner_manager_->pp_.bspline_degree_;
      bspline.start_time = ros::Time::now();
      bspline.traj_id = planner_manager_->local_data_.traj_id_;

      Eigen::MatrixXd pos_pts(4, 3);
      for (int i = 0; i < 4; ++i) pos_pts.row(i) = fd_->odom_pos_.transpose();

      for (int i = 0; i < pos_pts.rows(); ++i) {
        geometry_msgs::Point pt;
        pt.x = pos_pts(i, 0);
        pt.y = pos_pts(i, 1);
        pt.z = pos_pts(i, 2);
        bspline.pos_pts.push_back(pt);
      }

      NonUniformBspline tmp(pos_pts, planner_manager_->pp_.bspline_degree_, 1.0);
      Eigen::VectorXd knots = tmp.getKnot();
      for (int i = 0; i < knots.rows(); ++i) {
        bspline.knots.push_back(knots(i));
      }
      bspline.drone_id = expl_manager_->ep_->drone_id_;
      swarm_traj_pub_.publish(bspline);
    }
  }

  // ============================== inspect =====================================
  /**
   * @brief 计算全局最优的inspection island
   */
  int FastExplorationFSM::callInspectPlanner() {
    ROS_INFO("[INSP] Executing inspection planning.");

    ros::Time time_r = ros::Time::now() + ros::Duration(fp_->replan_time_);

    auto& island_buffer_ = expl_manager_->ed_->swarm_state_[getId() - 1].island_buffer;
    if (island_buffer_.empty()) {
      ROS_WARN("[Planner] No islands available for inspection planning.");
      return INSP_FAIL;
    }

    int res = INSP_FAIL;
    if (inpsct_planner_type_ == 1) {  // --- 全局规划 ---
      int best_target_id = calculateGlobalBestTarget();

      if (best_target_id != -1) {
        current_inspection_island_id_ = best_target_id;
        res = expl_manager_->planInspectMotion(fd_->start_pos_, fd_->start_pt_, fd_->start_vel_,
              fd_->start_acc_, fd_->start_yaw_, island_buffer_[current_inspection_island_id_].box);
        cout << "[Planner] Global planning for inspection island ID: " << current_inspection_island_id_
             << ", result: " << res << std::endl;
      } else {
        ROS_WARN("[Planner] No valid island found for global planning.");
        res = INSP_FAIL;
      }
    } else if (inpsct_planner_type_ == 2) {  // --- 局部规划 ---
      if (current_inspection_island_id_ != -1) {
        res = expl_manager_->planInspectMotion(fd_->start_pos_, fd_->start_pt_, fd_->start_vel_, 
                                               fd_->start_acc_, fd_->start_yaw_, island_buffer_[current_inspection_island_id_].box);
        cout << "[Planner] Local planning for inspection island ID: " << current_inspection_island_id_
             << ", result: " << res << std::endl;
      } else {
        ROS_ERROR("[Planner] Local replan triggered but current_inspection_island_id_ is -1!");
        res = INSP_FAIL;
      }
    }

    if (res == INSP_SUCCEED) {
      auto info = &planner_manager_->local_data_;
      info->start_time_ = (ros::Time::now() - time_r).toSec() > 0 ? ros::Time::now() : time_r;

      bspline::Bspline bspline;
      bspline.order = planner_manager_->pp_.bspline_degree_;
      bspline.start_time = info->start_time_;
      bspline.traj_id = info->traj_id_;
      Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
      for (int i = 0; i < pos_pts.rows(); ++i) {
        geometry_msgs::Point pt;
        pt.x = pos_pts(i, 0);
        pt.y = pos_pts(i, 1);
        pt.z = pos_pts(i, 2);
        bspline.pos_pts.push_back(pt);
      }
      Eigen::VectorXd knots = info->position_traj_.getKnot();
      for (int i = 0; i < knots.rows(); ++i) {
        bspline.knots.push_back(knots(i));
      }
      Eigen::MatrixXd yaw_pts = info->yaw_traj_.getControlPoint();
      for (int i = 0; i < yaw_pts.rows(); ++i) {
        double yaw = yaw_pts(i, 0);
        bspline.yaw_pts.push_back(yaw);
      }
      bspline.yaw_dt = info->yaw_traj_.getKnotSpan();
      fd_->newest_traj_ = bspline;
    }

    return res;
  }

  /** // TODO
   * @brief 在所有未完成的岛屿中，计算出全局最佳目标。当前策略是：选择欧式距离最近的岛屿。
   * @return 最佳目标的ID；如果没有可用目标，则返回-1。
   */
  int FastExplorationFSM::calculateGlobalBestTarget() {
    // 使用 lock_guard 来安全地访问共享数据
    std::lock_guard<std::mutex> lock(island_buffer_mutex_);

    int best_target_id = -1;
    double min_dist_sq = 100000000;

    const auto& island_buffer_ = expl_manager_->ed_->swarm_state_[getId() - 1].island_buffer;

    // 遍历所有已知的岛屿
    for (const auto& pair : island_buffer_) {
      int id = pair.first;
      const auto& box_corners = pair.second.box;

      // 如果岛屿已经检视完成，则跳过
      if (completed_island_ids_.count(id)) {
        continue;
      }

      // 计算到岛屿中心的距离
      Eigen::Vector3d island_center = (box_corners[0] + box_corners[1]) / 2.0;
      double dist_sq = (island_center - fd_->odom_pos_).squaredNorm();

      // 寻找最近的岛屿
      if (dist_sq < min_dist_sq) {
        min_dist_sq = dist_sq;
        best_target_id = id;
      }
    }

    return best_target_id;
  }

  /**
   * @brief 检测现在检视的island是否完成
   * @return 完成-true，没完成-false
   */
  bool FastExplorationFSM::checkIslandFinished() {
    if (current_inspection_island_id_ == -1) return true;

    const auto& island_buffer_ = expl_manager_->ed_->swarm_state_[getId() - 1].island_buffer;
    // 1. 获取当前正在检视的岛屿的包围盒
    std::unique_lock<std::mutex> island_lock(island_buffer_mutex_);
    if (island_buffer_.find(current_inspection_island_id_) == island_buffer_.end()) {
      ROS_ERROR("[FSM] checkIslandFinished: Current inspection ID %d not found in buffer.",
                current_inspection_island_id_);
      return true;
    }
    // 复制一份数据，然后立即解锁
    const auto& island_corners = island_buffer_.at(current_inspection_island_id_).box;
    const Eigen::Vector3d island_bmin = island_corners[0];
    const Eigen::Vector3d island_bmax = island_corners[1];
    island_lock.unlock();

    // 2. 获取当前地图中所有前沿点的包围盒
    vector<pair<Eigen::Vector3d, Eigen::Vector3d>> all_frontier_boxes_raw;
    expl_manager_->frontier_finder_->getFrontierBoxes(all_frontier_boxes_raw);

    if (all_frontier_boxes_raw.empty()) {
      ROS_INFO("[FSM] All frontier is finished.");
      return true;
    }

    // 3. 遍历所有前沿点，检查是否存在重叠
    for (const auto& box_raw : all_frontier_boxes_raw) {
      const Eigen::Vector3d& center = box_raw.first;
      const Eigen::Vector3d& scale = box_raw.second;
      const Eigen::Vector3d frontier_bmin = center - scale / 2.0;
      const Eigen::Vector3d frontier_bmax = center + scale / 2.0;

      // 4. 检查岛屿包围盒与前沿点包围盒是否重叠
      bool has_overlap = true;
      for (int i = 0; i < 3; ++i) {
        if (std::max(island_bmin[i], frontier_bmin[i]) >
            std::min(island_bmax[i], frontier_bmax[i]) + 1e-3) {
          has_overlap = false;
          break;
        }
      }

      if (has_overlap) {
        return false;
      }
    }

    // 5. 如果遍历完所有前沿点都没有发现重叠，说明该岛屿已检视完成
    ROS_INFO("[FSM] Island %d finished: No overlapping frontiers found.", current_inspection_island_id_);
    return true;
  }


  /// @brief 0：低空无人机，1：高空无人机
  /// @param type 
  void FastExplorationFSM::switchDroneType(const int type) {
    if (fp_->drone_type_ == type){
      ROS_WARN("drone %d, already in type %d", getId(), type);
      return;
    } 
    exploration_manager::CamSwitch msg;
    ROS_WARN("drone %d, from %d to %d", getId(), fp_->drone_type_, type);
    if (type == 0) {
      msg.to_drone_type = exploration_manager::CamSwitch::LOW;
      cam_switch_pub_.publish(msg);
      fp_->drone_type_ = 0;
      inpsct_planner_type_ = 1;
    } else if (type == 1) {
      msg.to_drone_type = exploration_manager::CamSwitch::HIGH;
      cam_switch_pub_.publish(msg);
      fp_->drone_type_ = 1;
    }
  }

}  // namespace fast_planner
