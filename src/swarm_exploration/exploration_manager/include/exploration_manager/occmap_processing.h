#ifndef _OCCMAP_PROCESSING_H
#define _OCCMAP_PROCESSING_H

#include <Eigen/Dense>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <vector>
#include <opencv2/imgproc/types_c.h>
#include <plan_env/sdf_map.h>
#include <math.h>
#include <stdexcept>
#include <limits>
#include <algorithm>

#define PI 3.1415926535

using namespace std;

struct Plane {
public:
  Eigen::Vector3d init_coord;  // 2D
  std::vector<Eigen::Vector3d> plane_pts;
  Eigen::Vector3d normal_vector;
  float plane_height;
  float plane_angle;
};

class OccmapProcessing {
public:
  OccmapProcessing() {
  }
  ~OccmapProcessing() {
  }

  void init(shared_ptr<fast_planner::SDFMap> map, double low, double high, int apsize,
      int cl_kernel, int min_a_thr);
  int MSERCelculateBoxs(
      const Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>& img_matrix,
      vector<vector<Eigen::Vector2i>>& boxs);
  int cannyCelculateBoxs(
      const Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>& img_matrix,
      vector<vector<Eigen::Vector2i>>& boxs);
  int getAngleMatrix(const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& raw_matrix,
      Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>& angle_matrix,
      const Eigen::Vector3i& bmin);
  double calculateProperHeight(
      const Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic>& height_matrix,
      const double& percentile_fraction);
  int cutInImg(const Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic>& img,
      Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>& output,
      Eigen::Vector2i& basedelta);
  int cutnfillAABBImg(const Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic>& img,
      Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>& output,
      Eigen::Vector2i& basedelta);

private:
  double lowThreshold;
  double highThreshold;
  int apertureSize;
  int closing_kernel;
  double min_area_thresh;
  shared_ptr<fast_planner::SDFMap> sdf_map_;

  double getFitPlaneAngle(Eigen::Vector3d& p_surface, const double& radius);
  double getAngle(Eigen::Vector3d& plane_vector);
  bool fillUnknown(Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic>& angle_matrix);
  bool expandEdge(const cv::Mat& img, int edge[], const int edgeID);
  cv::Rect InSquare(const cv::Mat& img, const cv::Point center);
  /// @brief 根据rect裁剪cv::mat
  /// @param original_mat 素材
  /// @param rect 裁剪区域
  /// @return rect或者mat非法，返回一个空mat， 否则返回裁剪后的mat
  inline cv::Mat cutMat(cv::Mat& original_mat, const cv::Rect& rect) {
    if (rect.x + rect.width > original_mat.cols || rect.y + rect.height > original_mat.rows ||
        rect.width <= 0 || rect.height <= 0) {
      return cv::Mat();
    }
    return original_mat(rect).clone();
  }
};

#endif