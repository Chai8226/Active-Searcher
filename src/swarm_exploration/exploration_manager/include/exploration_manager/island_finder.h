#ifndef _ISLAND_FINDER_H
#define _ISLAND_FINDER_H

#include <exploration_manager/occmap_processing.h>
#include <plan_env/sdf_map.h>
// #include <unordered_map>
#include <vector>
#include <map>
#include <math.h>
#include <active_perception/frontier_finder.h>

using namespace std;

enum ISLAND_STATE { NEW, MERGED, COMPLETED, STABLE };

typedef struct {
  vector<Eigen::Vector3d> box;
  ISLAND_STATE state;
  vector<int> merged_ids;
  double area;
  double last_area;  // 用于判断是否发生变化
  Eigen::Vector3d center;  // 用来唯一标识可疑区域
  Eigen::Vector3d last_center;  // 上一次的中心位置
  double frontier_pt_num;  // 边界点数目
} Island;

class IslandFinder {
public:
  IslandFinder() {
  }
  ~IslandFinder() {
  }
  void init(shared_ptr<fast_planner::SDFMap> map, shared_ptr<fast_planner::FrontierFinder> frontier_finder, int drone_id, int drone_num);
  int searchMSERUpdatedIslands();
  int searchCannyUpdatedIslands();
  int searchSVDUpdatedIslands();
  int mergeNewIslands(const vector<vector<Eigen::Vector3d>>& island_buffer, bool refine);
  void refineLocalIslands(vector<int>& dropped_ids);
  // void dropRelatedIslands(vector<vector<Eigen::Vector3d>>& deleted_island_box);

  void getIslandToPub(map<int, Island>& out);
  void getAllIslandBoxs(map<int, Island>& out, int id = -1);
  void updateGlobalIslandMaps(const map<int, Island>& input, const int id);
  inline void setCompleted(const int id) {
    this->island_boxs[drone_id_ - 1][id].state = COMPLETED;
  }
  bool isIslandStable(const Island& island) {
    if(island.last_area < 0) return false;  // 初始状态
    if (fabs(island.area - island.last_area) < 1e-3) {
      return true;  // 没有变化，认为是稳定的
    }
    return false;
  }
  
  bool isIslandStable(int island_id) {
    return isIslandStable(this->island_boxs[drone_id_ - 1][island_id]);
  }

  inline Eigen::Vector3d getGrowthVector(const Island& island, const Eigen::Vector3d& pos) {
    if (island.last_area < 0) return Eigen::Vector3d::Zero();  // NEW初始状态
    Eigen::Vector3d growth_vector = Eigen::Vector3d::Zero();
    growth_vector = (island.center - island.last_center).normalized() * (island.area - island.last_area);
    auto dir = island.center - pos;
    growth_vector = growth_vector.dot(dir.normalized()) * dir.normalized();
    return growth_vector;
  }

  inline Eigen::Vector3d getGrowthVector(const int island_id, const Eigen::Vector3d& pos) {
    return getGrowthVector(this->island_boxs[drone_id_ - 1][island_id], pos);
  }
  /// @brief 获取这个id的island_state
  // inline ISLAND_STATE getIslandState(const int id) {
  //   return island_boxs[id].state;
  // }
  // inline void getDroppedBoxs(vector<vector<Eigen::Vector3d>>& out) {
  //   out.clear();
  //   out = this->dropped_boxs;
  // }
  // inline void clearDroppedBoxs() {
  //   this->dropped_boxs.clear();
  // }
private:
  double getArea(const Eigen::Vector3d& min1, const Eigen::Vector3d& max1);
  double getOverlapArea(const Eigen::Vector3d& min1, const Eigen::Vector3d& max1,
      const Eigen::Vector3d& min2, const Eigen::Vector3d& max2);
  bool isknown(Island& island, const vector<vector<Vector3d>>& clusters);
  double getGroundHeight(const Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic>& img,
      const Eigen::Vector3i& base, const vector<vector<Eigen::Vector3d>>& island_box);

  /// @brief double 按照thre阈值来舍入
  inline int F2I(const double& num, double thre) {
    if (num - floor(num) > thre) return ceil(num);
    return floor(num);
  }

  /// @brief 根据可疑区域的中心位置的address生成一个唯一的标识id
  /// @param center_pos 中心
  /// @return id值
  inline int getNewIslandID(const Eigen::Vector3d& center_pos) {
    Eigen::Vector3i index; 
    this->sdf_map_->posToIndex(center_pos, index);
    return this->sdf_map_->toAddress(index);
  }

  bool checkAABBIntersection(const std::vector<Eigen::Vector3d>& box1, const std::vector<Eigen::Vector3d>& box2);

  shared_ptr<OccmapProcessing> occ_process;
  shared_ptr<fast_planner::SDFMap> sdf_map_;
  shared_ptr<fast_planner::FrontierFinder> frontier_finder_;
  // vector<vector<Eigen::Vector3d>> island_boxs;  // xy平面上的AABBs
  // vector<Island> island_boxs;
  vector<map<int, Island>> island_boxs;
  // vector<ISLAND_STATE> island_state_list;       // 大小应和island_boxs时刻一致
  int drone_id_;
  int drone_num_;
  double avrg_h;  // 用来设置无人机的飞行高度（avrg_h + ep->height）
  unsigned long island_counter;
  
};

#endif