#include <exploration_manager/island_finder.h>

void IslandFinder::init(shared_ptr<fast_planner::SDFMap> map,
    shared_ptr<fast_planner::FrontierFinder> frontier_finder, int drone_id, int drone_num) {
  double lowThreshold = 12;
  double highThreshold = 35;
  int apertureSize = 7;
  int closing_kernel = 8;
  int min_area_thresh = 3;
  this->sdf_map_ = map;
  this->frontier_finder_ = frontier_finder;
  this->occ_process.reset(new OccmapProcessing());
  this->occ_process->init(
      map, lowThreshold, highThreshold, apertureSize, closing_kernel, min_area_thresh);
  this->drone_id_ = drone_id;
  this->avrg_h = 0;
  this->island_counter = 0;
  this->drone_num_ = drone_num;
  island_boxs.resize(drone_num);
}

int IslandFinder::searchMSERUpdatedIslands() {
  // const auto& occbuffer = this->sdf_map->md_->occupancy_buffer_;
  Eigen::Vector3d bmin_pos, bmax_pos;
  Eigen::Vector3i bmin, bmax;

  this->sdf_map_->getUpdatedBox(bmin_pos, bmax_pos, false);

  Eigen::Vector3d min_boundary, max_boundary;
  this->sdf_map_->getMapBoundary(min_boundary, max_boundary);
  bmin_pos(0) = max(bmin_pos(0), min_boundary(0));
  bmin_pos(1) = max(bmin_pos(1), min_boundary(1));
  bmax_pos(0) = min(bmax_pos(0), max_boundary(0));
  bmax_pos(1) = min(bmax_pos(1), max_boundary(1));

  this->sdf_map_->posToIndex(bmin_pos, bmin);
  this->sdf_map_->posToIndex(bmax_pos, bmax);

  // std::cout << "make matrix, bmax_pos(" << bmax_pos(0) << "," << bmax_pos(1) << ") bmin_pos("
  //           << bmin_pos(0) << "," << bmin_pos(1) << "," <<bmin_pos(2) << std::endl;
  // std::cout << "make matrix, bmax(" << bmax(0) << "," << bmax(1) << ") bmin(" << bmin(0) << ","
  //           << bmin(1) << "," << bmin(2) << std::endl;

  if ((bmax(0) - bmin(0)) <= 0 || (bmax(1) - bmin(1)) <= 0) return -1;
  // std::cout << "pass check" << std::endl;
  Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> in_matrix(
      bmax(0) - bmin(0) + 1, bmax(1) - bmin(1) + 1);
  Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> in_matrix1(
      bmax(0) - bmin(0) + 1, bmax(1) - bmin(1) + 1);
  // std::cout << "matrix done" << std::endl;
  in_matrix.fill(-1);
  in_matrix1.fill(-1);  // 位置区域用-1初始化，便于裁剪
  bool flag = false;
  double z_min = DBL_MAX, z_max = -1;
  for (int x = bmin(0); x <= bmax(0); ++x) {
    for (int y = bmin(1); y <= bmax(1); ++y) {
      for (int z = bmin(2); z <= bmax(2); ++z) {
        Eigen::Vector3i idx = Eigen::Vector3i(x, y, z);
        // std::cout << "getOccupancy" << std::endl;
        if (this->sdf_map_->getOccupancy(idx) == fast_planner::SDFMap::OCCUPIED) {
          // std::cout << "in_matrix" << std::endl;
          Eigen::Vector3d pos;
          this->sdf_map_->indexToPos(idx, pos);
          z_min = min(z_min, pos(2));
          z_max = max(z_max, pos(2));
          if (F2I(pos(2), 0.75) > in_matrix(x - bmin(0), y - bmin(1))) {
            in_matrix(x - bmin(0), y - bmin(1)) = F2I(pos(2), 0.75);  // z向下取整
            flag = true;
            in_matrix1(x - bmin(0), y - bmin(1)) = F2I(pos(2), 0.75);
          }
        }
      }
    }
  }
  if (!flag) {
    std::cout << "i saw nothing" << std::endl;
    return 0;
  }

  Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> refined_matrix;
  Eigen::Vector2i basedelta;
  if (occ_process->cutnfillAABBImg(in_matrix, refined_matrix, basedelta) == -1) {
    std::cout << "cut zero" << std::endl;
    return 0;
  }
  bmin(0) += basedelta(0);
  bmin(1) += basedelta(1);

  // // refined_matrix = in_matrix.cast<unsigned char>();
  // Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
  // std::cout << "raw one" << std::endl;
  // std::cout << in_matrix.format(CleanFmt) << std::endl;
  // std::cout << "cutted one" << std::endl;
  // std::cout << refined_matrix.cast<int>().format(CleanFmt) << std::endl;

  std::cout << "into mser_celculate_boxs" << std::endl;
  vector<vector<Eigen::Vector2i>> boxs_buffer;
  // ros::Time time_begin = ros::Time::now();
  if (occ_process->MSERCelculateBoxs(refined_matrix * 10, boxs_buffer) != 1) return 0;
  vector<vector<Eigen::Vector3d>> island_buffer;  // 暂存
  // this->island_boxs.clear();
  for (auto& box : boxs_buffer) {
    // cout << box[0].transpose() << "  " << box[1].transpose() << endl;
    // for (int i = box[0][0]; i <= box[1][0]; i++)
    //   for (int j = box[0][1]; j <= box[1][1]; j++) {
    //     box_matrix(i, j) = 1;
    //   }
    vector<Eigen::Vector3d> pos_box;
    Eigen::Vector3d b;
    Eigen::Vector2i box_min = box[0] + Eigen::Vector2i(bmin(0), bmin(1));
    Eigen::Vector2i box_max = box[1] + Eigen::Vector2i(bmin(0), bmin(1));
    this->sdf_map_->indexToPos(Eigen::Vector3i(box_min(0), box_min(1), 0), b);
    pos_box.push_back(Eigen::Vector3d(b(0), b(1), z_min));
    this->sdf_map_->indexToPos(Eigen::Vector3i(box_max(0), box_max(1), 0), b);
    pos_box.push_back(Eigen::Vector3d(b(0), b(1), z_max));
    island_buffer.push_back(pos_box);
    // this->island_boxs.push_back(pos_box);
  }
  // cout << endl;
  // std::cout << box_matrix.format(CleanFmt) << std::endl;
  // double res = getGroundHeight(refined_matrix, island_buffer, bmin);
  mergeNewIslands(island_buffer, true);

  return 1;
}

/// @brief 用canny算法处理被更新区域范围之中的occupancy buffer，
/// 将xy面中的z值(index)最大点放入matrix，传入canny algo。输出得到划定好的AABBs
int IslandFinder::searchCannyUpdatedIslands() {
  // const auto& occbuffer = this->sdf_map->md_->occupancy_buffer_;
  Eigen::Vector3d bmin_pos, bmax_pos;
  Eigen::Vector3i bmin, bmax;

  this->sdf_map_->getUpdatedBox(bmin_pos, bmax_pos, false);

  Eigen::Vector3d min_boundary, max_boundary;
  this->sdf_map_->getMapBoundary(min_boundary, max_boundary);
  bmin_pos(0) = max(bmin_pos(0), min_boundary(0));
  bmin_pos(1) = max(bmin_pos(1), min_boundary(1));
  bmax_pos(0) = min(bmax_pos(0), max_boundary(0));
  bmax_pos(1) = min(bmax_pos(1), max_boundary(1));

  this->sdf_map_->posToIndex(bmin_pos, bmin);
  this->sdf_map_->posToIndex(bmax_pos, bmax);

  // std::cout << "make matrix, bmax_pos(" << bmax_pos(0) << "," << bmax_pos(1) << ") bmin_pos("
  //           << bmin_pos(0) << "," << bmin_pos(1) << "," <<bmin_pos(2) << std::endl;
  // std::cout << "make matrix, bmax(" << bmax(0) << "," << bmax(1) << ") bmin(" << bmin(0) << ","
  //           << bmin(1) << "," << bmin(2) << std::endl;

  if ((bmax(0) - bmin(0)) <= 0 || (bmax(1) - bmin(1)) <= 0) return -1;
  // std::cout << "pass check" << std::endl;
  Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> in_matrix(
      bmax(0) - bmin(0) + 1, bmax(1) - bmin(1) + 1);
  Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> in_matrix1(
      bmax(0) - bmin(0) + 1, bmax(1) - bmin(1) + 1);
  // std::cout << "matrix done" << std::endl;
  in_matrix.setZero();
  in_matrix1.setZero();
  int flag = 0;
  double z_min = DBL_MAX, z_max = 0;
  for (int x = bmin(0); x <= bmax(0); ++x) {
    for (int y = bmin(1); y <= bmax(1); ++y) {
      for (int z = bmin(2); z <= bmax(2); ++z) {
        Eigen::Vector3i id = Eigen::Vector3i(x, y, z);
        // std::cout << "getOccupancy" << std::endl;
        if (this->sdf_map_->getOccupancy(id) == fast_planner::SDFMap::OCCUPIED) {
          // std::cout << "in_matrix" << std::endl;
          Eigen::Vector3d pos;
          this->sdf_map_->indexToPos(id, pos);
          z_min = min(z_min, pos(2));
          z_max = max(z_max, pos(2));
          if (floor(pos(2)) > in_matrix(x - bmin(0), y - bmin(1))) {
            in_matrix(x - bmin(0), y - bmin(1)) = floor(pos(2));  // z向下取整
            flag = 1;
            in_matrix1(x - bmin(0), y - bmin(1)) = floor(pos(2));
          }
        }
      }
    }
  }
  if (!flag) {
    std::cout << "all plain" << std::endl;
    return 0;
  }  // 全部为地面数据，不用录入

  // std::cout << "drone id: " << this->drone_id_ << std::endl;

  // Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
  // std::cout << in_matrix1.format(CleanFmt) << std::endl;

  std::cout << "into canny_celculate_boxs" << std::endl;
  vector<vector<Eigen::Vector2i>> boxs_buffer;
  // ros::Time time_begin = ros::Time::now();
  if (occ_process->cannyCelculateBoxs(in_matrix, boxs_buffer) != 1) return 0;
  vector<vector<Eigen::Vector3d>> island_buffer;  // 暂存
  // this->island_boxs.clear();
  for (auto& box : boxs_buffer) {
    // cout << box[0].transpose() << "  " << box[1].transpose() << endl;
    // for (int i = box[0][0]; i <= box[1][0]; i++)
    //   for (int j = box[0][1]; j <= box[1][1]; j++) {
    //     box_matrix(i, j) = 1;
    //   }
    vector<Eigen::Vector3d> pos_box;
    Eigen::Vector3d b;
    Eigen::Vector2i box_min = box[0] + Eigen::Vector2i(bmin(0), bmin(1));
    Eigen::Vector2i box_max = box[1] + Eigen::Vector2i(bmin(0), bmin(1));
    this->sdf_map_->indexToPos(Eigen::Vector3i(box_min(0), box_min(1), 0), b);
    pos_box.push_back(Eigen::Vector3d(b(0), b(1), z_min));
    this->sdf_map_->indexToPos(Eigen::Vector3i(box_max(0), box_max(1), 0), b);
    pos_box.push_back(Eigen::Vector3d(b(0), b(1), z_max));
    island_buffer.push_back(pos_box);
    // this->island_boxs.push_back(pos_box);
  }
  // cout << endl;
  // std::cout << box_matrix.format(CleanFmt) << std::endl;
  mergeNewIslands(island_buffer, true);
  return 1;
}

/// @brief SVD+canny，SVD算法计算坡度，再用canny处理;  6-8ms
int IslandFinder::searchSVDUpdatedIslands() {
  Eigen::Vector3d bmin_pos, bmax_pos;
  Eigen::Vector3i bmin, bmax;
  double z_max = 0, z_min = DBL_MAX;

  this->sdf_map_->getUpdatedBox(bmin_pos, bmax_pos, false);

  Eigen::Vector3d min_boundary, max_boundary;
  this->sdf_map_->getMapBoundary(min_boundary, max_boundary);
  bmin_pos(0) = max(bmin_pos(0), min_boundary(0));
  bmin_pos(1) = max(bmin_pos(1), min_boundary(1));
  bmin_pos(2) = max(bmin_pos(2), min_boundary(2));
  bmax_pos(0) = min(bmax_pos(0), max_boundary(0));
  bmax_pos(1) = min(bmax_pos(1), max_boundary(1));
  bmax_pos(2) = min(bmax_pos(2), max_boundary(2));

  // cout << "Update box pos: " << bmin_pos.transpose() << " " << bmax_pos.transpose() << endl;

  this->sdf_map_->posToIndex(bmin_pos, bmin);
  this->sdf_map_->posToIndex(bmax_pos, bmax);

  // cout << "Update box index: " << bmin.transpose() << " " << bmax.transpose() << endl;

  // this->sdf_map_->indexToPos(bmin, bmin_pos);
  // this->sdf_map_->indexToPos(bmax, bmax_pos);
  // cout << "Update new box pos: " << bmin_pos.transpose() << " " << bmax_pos.transpose() << endl;

  if ((bmax(0) - bmin(0)) <= 0 || (bmax(1) - bmin(1)) <= 0) return -1;

  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> raw_matrix(
      bmax(0) - bmin(0), bmax(1) - bmin(1));
  // Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> in_matrix1(
  //     bmax(0) - bmin(0) + 1, bmax(1) - bmin(1) + 1);
  Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> box_matrix(
      bmax(0) - bmin(0), bmax(1) - bmin(1));
  // cout << bmax(0) - bmin(0) + 1 << " " << bmax(1) - bmin(1) + 1 << endl;
  raw_matrix.setConstant(0);  // 未知区域初始化-1
  // in_matrix1.setZero();
  box_matrix.setZero();
  for (int x = 0; x < bmax(0) - bmin(0); ++x) {
    for (int y = 0; y < bmax(1) - bmin(1); ++y) {
      for (int z = 0; z < bmax(2) - bmin(2); ++z) {
        Eigen::Vector3i id = Eigen::Vector3i(x + bmin(0), y + bmin(1), z + bmin(2));
        if (this->sdf_map_->getOccupancy(id) == fast_planner::SDFMap::OCCUPIED) {
          Eigen::Vector3d pos;
          this->sdf_map_->indexToPos(id, pos);
          z_max = max(z_max, pos(2));
          z_min = min(z_min, pos(2));
          if (pos(2) > raw_matrix(x, y)) {
            raw_matrix(x, y) = pos(2);
            // in_matrix1(x - bmin(0), y - bmin(1)) = floor(pos(2));
            // in_matrix1(x, y) = pos(2);
          }
        }
      }
    }
  }

  // Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
  // std::cout << raw_matrix.format(CleanFmt) << std::endl;

  vector<vector<Eigen::Vector2i>> boxs_buffer;
  Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> angle_matrix;
  if (occ_process->getAngleMatrix(raw_matrix, angle_matrix, bmin) == -1) return 0;
  std::cout << "into cannyCelculateBoxs" << std::endl;
  if (occ_process->cannyCelculateBoxs(angle_matrix, boxs_buffer) == -1) return 0;
  std::cout << "celculate done" << std::endl;
  vector<vector<Eigen::Vector3d>> island_buffer;  // 暂存

  for (auto& box : boxs_buffer) {
    // cout << box[0].transpose() << "  " << box[1].transpose() << endl;
    for (int i = box[0][0]; i <= box[1][0]; i++)
      for (int j = box[0][1]; j <= box[1][1]; j++) {
        box_matrix(i, j) = 1;
      }
    vector<Eigen::Vector3d> pos_box;
    Eigen::Vector3d b;
    Eigen::Vector2i box_min = box[0] + Eigen::Vector2i(bmin(0), bmin(1));
    Eigen::Vector2i box_max = box[1] + Eigen::Vector2i(bmin(0), bmin(1));
    this->sdf_map_->indexToPos(Eigen::Vector3i(box_min(0), box_min(1), 0), b);
    pos_box.push_back(Eigen::Vector3d(b(0), b(1), z_min));
    this->sdf_map_->indexToPos(Eigen::Vector3i(box_max(0), box_max(1), 0), b);
    pos_box.push_back(Eigen::Vector3d(b(0), b(1), z_max));
    island_buffer.push_back(pos_box);
  }
  // cout << endl;
  // std::cout << box_matrix.format(CleanFmt) << std::endl;

  auto size = island_buffer.size();
  int i = mergeNewIslands(island_buffer, true);

  std::cout << "A size = " << size << std::endl;
  std::cout << "B size = " << i << std::endl;
  return 1;
}

/// @brief 把新发现的island和旧的融合，存储在类的island_boxs
/// @param island_buffer 新的island
/// @param refine 如果需要进行筛选，置1
/// @return 融合之后的island_boxs大小
int IslandFinder::mergeNewIslands(
    const vector<vector<Eigen::Vector3d>>& island_buffer, bool refine = true) {

  for (const auto& new_island : island_buffer) {
    // vector<int> list;
    double max_x = new_island[1](0), max_y = new_island[1](1), max_z = new_island[1](2),
           min_x = new_island[0](0), min_y = new_island[0](1), min_z = new_island[0](2);

    Island island;
    int min_id;
    for (auto& pair : this->island_boxs[this->drone_id_ - 1]) {
      double overlap_area =
          getOverlapArea(new_island[0], new_island[1], pair.second.box[0], pair.second.box[1]);
      double S1 = getArea(new_island[0], new_island[1]);
      double S2 = getArea(pair.second.box[0], pair.second.box[1]);
      double S = min(S1, S2);
      if (overlap_area >= 0.5 * S) {  // 所占面积大于一个比例，才有必要融合
        if (island.merged_ids.empty())
          min_id = pair.first;
        else
          min_id = min(min_id, pair.first);

        const auto& is = pair.second;
        max_x = max(max_x, is.box[1](0));
        max_y = max(max_y, is.box[1](1));
        max_z = max(max_z, is.box[1](2));
        min_x = min(min_x, is.box[0](0));
        min_y = min(min_y, is.box[0](1));
        min_z = min(min_z, is.box[0](2));
        island.merged_ids.push_back(pair.first);
        // this->island_boxs.erase(pair.first);  // 剔除他
      }
    }
    if (island.merged_ids.empty()) {  // nothing overlaps with it, insert the new one
      double area_thre = 0.1;
      if ((!refine || getArea(new_island[0], new_island[1]) > area_thre)) {
        island.box = new_island;
        island.state = NEW;
        island.area = getArea(new_island[0], new_island[1]);
        island.last_area = -1;
        island.center = 0.5 * (new_island[0] + new_island[1]);  // 用来唯一标识可疑区域
        this->island_boxs[drone_id_ - 1][getNewIslandID(island.center)] = island;
      }
    } else {
      for (const int& id : island.merged_ids) {
        this->island_boxs[drone_id_ - 1].erase(id);
      }
      island.box.push_back(Eigen::Vector3d(min_x, min_y, min_z));
      island.box.push_back(Eigen::Vector3d(max_x, max_y, max_z));
      // if(isknown(island.box)) continue;  // 已经被认知过的区域不再处理
      island.state = MERGED;

      island.last_area = island.area;  // 用于判断是否发生变化
      island.area = getArea(island.box[0], island.box[1]);
      island.last_center = island.center;  // 上一次的中心位置
      island.center = 0.5 * (new_island[0] + new_island[1]);
      this->island_boxs[drone_id_ - 1][min_id] = island;
    }
  }
  return this->island_boxs[drone_id_ - 1].size();
}

// /// @brief 把新发现的island和旧的融合，存储在类的island_boxs
// /// @param island_buffer 新的island
// /// @param refine 如果需要进行筛选，置1
// /// @return 融合之后的island_boxs大小
// int IslandFinder::mergeNewIslands(
//     const vector<vector<Eigen::Vector3d>>& island_buffer, bool refine = true) {
//   //auto& state_list = this->island_state_list;
//   // auto have_2dimoverlap = [](const Eigen::Vector3d& min1, const Eigen::Vector3d& max1,
//   //                             const Eigen::Vector3d& min2, const Eigen::Vector3d& max2) {
//   //   for (int m = 0; m < 2; ++m) {
//   //     double bmin = max(min1[m], min2[m]);
//   //     double bmax = min(max1[m], max2[m]);
//   //     if (bmin > bmax + 1e-3) return false;
//   //   }
//   //   return true;
//   // };

//   for (const auto& new_island : island_buffer) {
//     vector<int> list;
//     for (int j = 0; j < this->island_boxs.size(); j++) {
//       if(this->island_boxs[j].state == COMPLETED) continue;
//       double overlap_area = getOverlapArea(
//           new_island[0], new_island[1], this->island_boxs[j].box[0],
//           this->island_boxs[j].box[1]);
//       double S1 = getArea(new_island[0], new_island[1]);
//       double S2 = getArea(this->island_boxs[j].box[0], this->island_boxs[j].box[1]);
//       double S = min(S1, S2);
//       if (overlap_area >= 0.5 * S) {  // 所占面积大于一个比例，才有必要融合
//         list.push_back(j);
//       }
//     }
//     if (!list.empty()) {  // merge
//       double max_x = new_island[1](0), max_y = new_island[1](1), max_z = new_island[1](2),
//              min_x = new_island[0](0), min_y = new_island[0](1), min_z = new_island[0](2);
//       for (int k : list) {  // find the Bbox of them all
//         auto& island = this->island_boxs[k];
//         max_x = max(max_x, island.box[1](0));
//         max_y = max(max_y, island.box[1](1));
//         max_z = max(max_z, island.box[1](2));
//         min_x = min(min_x, island.box[0](0));
//         min_y = min(min_y, island.box[0](1));
//         min_z = min(min_z, island.box[0](2));
//       }

//       Island island;
//       for (int i = list.size() - 1; i >= 0; i--) {  // erase old one
//         // this->island_boxs.erase(this->island_boxs.begin() + list[i]);
//         // state_list.erase(state_list.begin() + list[i]);
//         this->island_boxs[list[i]].state = COMPLETED;
//         island.merged_ids.push_back(list[i]);
//       }
//       island.box.push_back(Eigen::Vector3d(min_x, min_y, min_z));
//       island.box.push_back(Eigen::Vector3d(max_x, max_y, max_z));
//       island.state = MERGED;
//       // if (!refine || !isknown(b))
//       {
//         this->island_boxs.push_back(island);
//         // state_list.push_back(MERGED);
//       }
//     } else {  // nothing overlaps with it, insert the new one
//       double area_thre = 0.1;
//       if (!refine ||
//           (/*!isknown(new_island) && */ getArea(new_island[0], new_island[1]) > area_thre)) {
//         Island new_one;
//         new_one.box = new_island;
//         new_one.state = NEW;
//         this->island_boxs.push_back(new_one);
//         // state_list.push_back(NEW);
//       }
//     }
//   }
//   return this->island_boxs.size();
// }

/**
 * @brief 判断两个由最小点和最大点定义的三维区域在XY平面上的投影是否相交，但非完全包含。
 * * @param region1 第一个区域，由两个Eigen::Vector3d表示（最小点和最大点）。
 * @param region2 第二个区域，由两个Eigen::Vector3d表示（最小点和最大点）。
 * @return bool 如果两个区域在XY平面上相交但互不完全包含，则返回true，否则返回false。
 */
bool IslandFinder::areBoxsIntersect(
    const std::vector<Eigen::Vector3d>& region1, const std::vector<Eigen::Vector3d>& region2) {

  // 参数校验
  if (region1.size() != 2 || region2.size() != 2) {
    // 或者可以抛出异常
    std::cerr << "错误：每个区域必须包含恰好两个点（最小点和最大点）。" << std::endl;
    return false;
  }

  // 从三维向量中提取二维区域的边界
  // 为了代码清晰，我们定义了最小和最大点
  const Eigen::Vector2d min1(region1[0].x(), region1[0].y());
  const Eigen::Vector2d max1(region1[1].x(), region1[1].y());
  const Eigen::Vector2d min2(region2[0].x(), region2[0].y());
  const Eigen::Vector2d max2(region2[1].x(), region2[1].y());

  // 1. 判断两个矩形是否相交
  bool intersects =
      (max1.x() > min2.x() && min1.x() < max2.x() && max1.y() > min2.y() && min1.y() < max2.y());

  // 如果不相交，直接返回 false
  if (!intersects) {
    return false;
  }

  // 2. 判断其中一方是否完全包围另一方
  // 检查 region1 是否包含 region2
  bool region1ContainsRegion2 = (min1.x() <= min2.x() && min1.y() <= min2.y() &&
                                 max1.x() >= max2.x() && max1.y() >= max2.y());

  // 检查 region2 是否包含 region1
  bool region2ContainsRegion1 = (min2.x() <= min1.x() && min2.y() <= min1.y() &&
                                 max2.x() >= max1.x() && max2.y() >= max1.y());

  // 只有在相交且不互相包含的情况下，才返回 true
  return intersects && !region1ContainsRegion2 && !region2ContainsRegion1;
}

/// @brief 判断了解程度
/// @param island_box
/// @return
bool IslandFinder::isknown(Island& island, const vector<vector<Vector3d>>& clusters) {
  if (island.box.empty()) {
    ROS_ERROR("EMPTY BOX");
    return true;
  }
  if (island.box.size() != 2) {
    ROS_ERROR("WRONG BOX");
    return true;
  }
  // const auto& frontier_finder = this->frontier_finder_;
  vector<pair<Eigen::Vector3d, Eigen::Vector3d>> frontier_boxes;
  frontier_finder_->getFrontierBoxes(frontier_boxes);
  int known_num = 0;
  bool have_frontier_inside = false;
  int frontier_thre = 0;
  for (size_t i = 0; i < frontier_boxes.size(); i++) {
    vector<Eigen::Vector3d> box = { frontier_boxes[i].first, frontier_boxes[i].second };
    if (checkAABBIntersection(island.box, box)) {  // 如果边界和可疑区域有交集
      have_frontier_inside = true;
    }
  }
  // island.frontier_pt_num = known_num;
  if (!have_frontier_inside) return true;

  Eigen::Vector3i max_id, min_id;
  this->sdf_map_->posToIndex(island.box[0], min_id);
  this->sdf_map_->posToIndex(island.box[1], max_id);

  // int boundary_num = 0;
  // int occ_num = 0;
  // int sum = (max_id(0) - min_id(0)) * (max_id(1) - min_id(1)) * (max_id(2) - min_id(2));

  // for (int x = min_id(0); x <= max_id(0); x++) {
  //   for (int y = min_id(1); y <= max_id(1); y++) {
  //     for (int z = min_id(2); z <= max_id(2); z++) {

  //       Eigen::Vector3i pos = Eigen::Vector3i(x, y, z);

  //       if (this->sdf_map_->getOccupancy(pos) == fast_planner::SDFMap::UNKNOWN) {
  //         const vector<Eigen::Vector3i> delta = { Eigen::Vector3i(1, 0, 0),
  //           Eigen::Vector3i(-1, 0, 0), Eigen::Vector3i(0, 1, 0), Eigen::Vector3i(0, -1, 0),
  //           Eigen::Vector3i(0, 0, 1), Eigen::Vector3i(0, 0, -1) };
  //         for (int i = 0; i < 6; i++) {
  //           Eigen::Vector3i neighbor = pos + delta[i];
  //           if (this->sdf_map_->isInMap(neighbor)) {
  //             if (this->sdf_map_->getOccupancy(neighbor) != fast_planner::SDFMap::UNKNOWN) {
  //               boundary_num++;
  //               break;
  //             }
  //           }
  //         }
  //       } else if (this->sdf_map_->getOccupancy(pos) == fast_planner::SDFMap::OCCUPIED) {
  //         occ_num++;
  //       }
  //     }
  //   }
  // }

  // // 在这里添加验证
  // if (boundary_num == 0) {
  //   ROS_ERROR("IslandFinder::isknown: Division by zero avoided! boundary_num is 0.");
  //   return false;  // 或者其他适合你业务逻辑的返回值
  // }
  // // double rate = boundary_num / ((double)sum);  // unknown_voxels / all_voxels
  // double rate = (double)occ_num / boundary_num;
  // std::cout << "boundary_num" << boundary_num << std::endl;
  // std::cout << "occ_num" << occ_num << std::endl;
  // if (rate >= 0.9) {
  //   return true;
  // }
  // return false;
  return false;
}

// /// @brief 判断了解程度
// /// @param island_box
// /// @return
// bool IslandFinder::isknown(Island& island, const vector<vector<Vector3d>>& clusters) {
//   if (island.box.empty()) {
//     ROS_ERROR("EMPTY BOX");
//     return true;
//   }
//   if (island.box.size() != 2) {
//     ROS_ERROR("WRONG BOX");
//     return true;
//   }
//   //const auto& frontier_finder = this->frontier_finder_;
//   vector<pair<Eigen::Vector3d,Eigen::Vector3d>> frontier_boxes;
//   frontier_finder_->getFrontierBoxes(frontier_boxes);
//   int known_num = 0;
//   int frontier_thre = 0;
//   for(size_t i = 0; i < frontier_boxes.size(); i++) {
//     vector<Eigen::Vector3d> box = {frontier_boxes[i].first, frontier_boxes[i].second};
//     if(checkAABBIntersection(island.box, box)) {  //如果边界和可疑区域有交集
//       const auto& points = clusters[i]; //找到这个边界的簇
//       for(const auto& point : points) {
//         if(point(0) >= island.box[0](0) && point(0) <= island.box[1](0) &&
//            point(1) >= island.box[0](1) && point(1) <= island.box[1](1) &&
//            point(2) >= island.box[0](2) && point(2) <= island.box[1](2)) {
//           known_num++;  //记录可以区域之中的边界点总数
//         }
//       }
//     }
//   }
//   island.frontier_pt_num = known_num;
//   if(known_num <= frontier_thre) return true;

//   Eigen::Vector3i max_id, min_id;
//   this->sdf_map_->posToIndex(island.box[0], min_id);
//   this->sdf_map_->posToIndex(island.box[1], max_id);

//   // int boundary_num = 0;
//   // int occ_num = 0;
//   // int sum = (max_id(0) - min_id(0)) * (max_id(1) - min_id(1)) * (max_id(2) - min_id(2));

//   // for (int x = min_id(0); x <= max_id(0); x++) {
//   //   for (int y = min_id(1); y <= max_id(1); y++) {
//   //     for (int z = min_id(2); z <= max_id(2); z++) {

//   //       Eigen::Vector3i pos = Eigen::Vector3i(x, y, z);

//   //       if (this->sdf_map_->getOccupancy(pos) == fast_planner::SDFMap::UNKNOWN) {
//   //         const vector<Eigen::Vector3i> delta = { Eigen::Vector3i(1, 0, 0),
//   //           Eigen::Vector3i(-1, 0, 0), Eigen::Vector3i(0, 1, 0), Eigen::Vector3i(0, -1, 0),
//   //           Eigen::Vector3i(0, 0, 1), Eigen::Vector3i(0, 0, -1) };
//   //         for (int i = 0; i < 6; i++) {
//   //           Eigen::Vector3i neighbor = pos + delta[i];
//   //           if (this->sdf_map_->isInMap(neighbor)) {
//   //             if (this->sdf_map_->getOccupancy(neighbor) != fast_planner::SDFMap::UNKNOWN) {
//   //               boundary_num++;
//   //               break;
//   //             }
//   //           }
//   //         }
//   //       } else if (this->sdf_map_->getOccupancy(pos) == fast_planner::SDFMap::OCCUPIED) {
//   //         occ_num++;
//   //       }
//   //     }
//   //   }
//   // }

//   // // 在这里添加验证
//   // if (boundary_num == 0) {
//   //   ROS_ERROR("IslandFinder::isknown: Division by zero avoided! boundary_num is 0.");
//   //   return false;  // 或者其他适合你业务逻辑的返回值
//   // }
//   // // double rate = boundary_num / ((double)sum);  // unknown_voxels / all_voxels
//   // double rate = (double)occ_num / boundary_num;
//   // std::cout << "boundary_num" << boundary_num << std::endl;
//   // std::cout << "occ_num" << occ_num << std::endl;
//   // if (rate >= 0.9) {
//   //   return true;
//   // }
//   // return false;
//   return false;
// }

/// @brief 计算两个矩形区域的相交面积
/// @param min1
/// @param max1
/// @param min2
/// @param max2
/// @return double 如果相交，返回相交部分的面积；如果不相交，返回 -1.0
double IslandFinder::getOverlapArea(const Eigen::Vector3d& min1, const Eigen::Vector3d& max1,
    const Eigen::Vector3d& min2, const Eigen::Vector3d& max2) {
  // 为清晰起见，提取各个坐标值
  // 矩形1的坐标
  double r1_x_min = min1(0);
  double r1_y_min = min1(1);
  double r1_x_max = max1(0);
  double r1_y_max = max1(1);

  // 矩形2的坐标
  double r2_x_min = min2(0);
  double r2_y_min = min2(1);
  double r2_x_max = max2(0);
  double r2_y_max = max2(1);

  // 计算相交区域的坐标
  // 相交区域的左下角 x 坐标是两个矩形左下角 x 坐标的最大值
  double intersect_x_min = std::max(r1_x_min, r2_x_min);
  // 相交区域的左下角 y 坐标是两个矩形左下角 y 坐标的最大值
  double intersect_y_min = std::max(r1_y_min, r2_y_min);

  // 相交区域的右上角 x 坐标是两个矩形右上角 x 坐标的最小值
  double intersect_x_max = std::min(r1_x_max, r2_x_max);
  // 相交区域的右上角 y 坐标是两个矩形右上角 y 坐标的最小值
  double intersect_y_max = std::min(r1_y_max, r2_y_max);

  // // 计算相交区域的宽度和高度
  // double width = intersect_x_max - intersect_x_min;
  // double height = intersect_y_max - intersect_y_min;
  return getArea(Eigen::Vector3d(intersect_x_min, intersect_y_min, 0),
      Eigen::Vector3d(intersect_x_max, intersect_y_max, 0));
}

bool IslandFinder::checkAABBIntersection(
    const std::vector<Eigen::Vector3d>& box1, const std::vector<Eigen::Vector3d>& box2) {
  // 从输入中获取每个立方体的最小和最大顶点
  const Eigen::Vector3d& minA = box1[0];
  const Eigen::Vector3d& maxA = box1[1];
  const Eigen::Vector3d& minB = box2[0];
  const Eigen::Vector3d& maxB = box2[1];

  // 检查每个轴上的投影是否重叠
  // 两个一维区间 [min1, max1] 和 [min2, max2] 重叠的条件是 (min1 <= max2) && (max1 >= min2)
  bool overlapX = (minA.x() <= maxB.x()) && (maxA.x() >= minB.x());
  bool overlapY = (minA.y() <= maxB.y()) && (maxA.y() >= minB.y());
  bool overlapZ = (minA.z() <= maxB.z()) && (maxA.z() >= minB.z());

  // 只有当所有轴的投影都重叠时，立方体才相交
  return overlapX && overlapY && overlapZ;
}

/// @brief 计算一个AABB的面积
/// @param min1 最小点，
/// @param max1 最大点
/// @return 计算出来的面积，如果矩形非法，返回-1
double IslandFinder::getArea(const Eigen::Vector3d& min1, const Eigen::Vector3d& max1) {
  // 计算相交区域的宽度和高度
  double width = max1(0) - min1(0);
  double height = max1(1) - min1(1);

  if (width >= 0 && height >= 0)
    return height * width;
  else {
    return -1;
  }
}

void IslandFinder::refineLocalIslands(vector<int>& dropped_ids) {
  dropped_ids.clear();
  vector<vector<Eigen::Vector3d>> clusters;
  frontier_finder_->getFrontiers(clusters);  // 缓存
  for (auto& pair : this->island_boxs[drone_id_ - 1]) {
    if (isknown(pair.second, clusters)) {
      dropped_ids.push_back(pair.first);
    }
  }
  for (const auto& id : dropped_ids) {
    this->island_boxs[drone_id_ - 1].erase(id);
  }
}

// void IslandFinder::refineLocalIslands(vector<int>& dropped_ids) {
//   dropped_ids.clear();
//   for (auto& pair : this->island_boxs) {
//     if (this->island_boxs[i].state == COMPLETED) continue;
//     if (isknown(this->island_boxs[i].box)) {
//       this->island_boxs[i].state = COMPLETED;
//       dropped_ids.push_back(i);
//     }
//   }
// }

// double IslandFinder::getGroundHeight(const Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic>&
// img, const Eigen::Vector3i& base,
//     const vector<vector<Eigen::Vector3d>>& island_box) {
//   double sum = 0;
//   int num = 0;
//   for (int x = 0; x < img.rows; x++) {
//     for (int y = 0; y < img.col; y++) {
//       bool notinbox = true;
//       Eigen::Vector3i index(x + base(0), y + base(1), img(x, y));
//       Eigen::Vector3d position;
//       this->sdf_map_->indexToPos(index, position);
//       for (const auto& island : island_box) {
//         if (position(0) >= island[0](0) && position(1) >= island[0](1) && position(0) <=
//         island[1](0) && position(1) <= island[1](1))
//           continue;
//         notinbox = false;
//         break;
//       }
//       if (notinbox) {
//         sum += position(2);
//         num++;
//       }
//     }
//   }
//   if (num != 0) return sum / num;
//   return -1;
// }

/// @brief 得到状态不为COMPLETED的island,然后把这些island的状态变为COMPLETED
/// @param out
void IslandFinder::getIslandToPub(map<int, Island>& out) {
  out.clear();
  for (auto& pair : this->island_boxs[drone_id_ - 1]) {
    auto& island = pair.second;
    if (island.state != COMPLETED) {
      out[pair.first] = island;
      island.state = COMPLETED;
    }
  }
}

void IslandFinder::getAllIslandBoxs(map<int, Island>& out, int id) {
  if (id == -1) id = drone_id_;
  out.clear();
  out = this->island_boxs[id - 1];
}

void IslandFinder::updateGlobalIslandMaps(const map<int, Island>& input, const int id) {
  if (id <= 0 || id > drone_num_) return;
  this->island_boxs[id - 1].clear();
  this->island_boxs[id - 1] = input;
}


