#include <exploration_manager/occmap_processing.h>

void OccmapProcessing::init(shared_ptr<fast_planner::SDFMap> map, double low, double high,
    int apsize, int cl_kernel, int min_a_thr) {
  this->sdf_map_ = map;
  lowThreshold = low;
  highThreshold = high;
  apertureSize = apsize;
  closing_kernel = cl_kernel;
  min_area_thresh = min_a_thr;
}

int OccmapProcessing::MSERCelculateBoxs(
    const Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>& img_matrix,
    vector<vector<Eigen::Vector2i>>& boxs) {
  cv::Mat image = cv::Mat::zeros(img_matrix.rows(), img_matrix.cols(), CV_8UC1);
  cv::eigen2cv(img_matrix, image);

  cv::Ptr<cv::MSER> mesr1 =
      cv::MSER::create(2, 5, img_matrix.rows() * img_matrix.cols() / 2, 0.2, 0.3);
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Rect> bboxes;
  mesr1->detectRegions(image, contours, bboxes);  // gray为处理的图像，为单通道灰度图

  std::cout << "contours size:" << contours.size() << std::endl;

  vector<vector<cv::Point>> ploys;  // 存储多边形
  for (int i = 0; i < contours.size(); i++) {
    const auto& contour = contours[i];

    // std::cout << i << " contour" << contour << std::endl;
    // std::cout << "----------------------------------" << std::endl;

    // if (hierarchy[i][3] == -1){
    //   std::cout << "without parent contours" << std::endl;
    //   continue;
    // }   // Skip contours without parent contours

    if (cv::contourArea(contour) < min_area_thresh) {
      // std::cout << "ploy min_area_thresh, size: " << contourArea(contour) << std::endl;
      continue;
    }

    ploys.push_back(contour);
  }

  if (ploys.size() == 0) {
    std::cout << "no ploys" << std::endl;
    return 0;
  }
  std::cout << "we got ploys, num" << ploys.size() << std::endl;
  for (int i = 0; i < ploys.size(); i++) {
    vector<Eigen::Vector2i> box;
    cv::Rect rect = cv::boundingRect(ploys[i]);
    Eigen::Vector2i b_min(
        rect.y, rect.x);  // 注意：rect.width/height 表示宽度/高度，最大坐标需要计算
    Eigen::Vector2i b_max(rect.y + rect.height - 1, rect.x + rect.width - 1);
    box.push_back(b_min);
    box.push_back(b_max);
    boxs.push_back(box);
    // std::cout << "ploy " << i + 1 << "bmin(" << b_min(0) << "," << b_min(1) << ") bmax(" <<
    // b_max(0) << ","
    //           << b_max(1) << ")" << std::endl;
  }

  return 1;
}

/// @brief 基于边界检测算法划出box
/// @param img_matrix 输入一个Eigen::Matrix，一个bov按照xy坐标存储z轴高度
/// @param boxs 存储各个box， 底层vector(bmax, bmin)
int OccmapProcessing::cannyCelculateBoxs(
    const Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>& img_matrix,
    vector<vector<Eigen::Vector2i>>& boxs) {

  cv::Mat image = cv::Mat::zeros(img_matrix.rows(), img_matrix.cols(), CV_8UC1);
  cv::eigen2cv(img_matrix, image);

  cv::Mat canny_img = cv::Mat::zeros(image.rows, image.cols, CV_8UC1);
  std::cout << "canny" << std::endl;
  cv::Canny(image, canny_img, lowThreshold, highThreshold, apertureSize, false);

  std::cout << "canny_img" << std::endl
            << format(canny_img, cv::Formatter::FMT_NUMPY) << std::endl
            << std::endl;
  std::cout << "start morphologyEx" << std::endl;
  cv::Mat kernel = cv::Mat::ones(this->closing_kernel, this->closing_kernel, CV_8UC1);
  cv::Mat edges;
  cv::morphologyEx(canny_img, edges, cv::MORPH_CLOSE, kernel);
  std::cout << "morphologyEx" << std::endl
            << format(edges, cv::Formatter::FMT_NUMPY) << std::endl
            << std::endl;
  // std::cout << "start findContours" << std::endl;
  vector<vector<cv::Point>> contours;
  vector<cv::Vec4i> hierarchy;
  cv::findContours(edges, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
  // std::cout << "make box" << std::endl;
  vector<vector<cv::Point>> ploys;  // 存储多边形
  for (int i = 0; i < contours.size(); i++) {
    const auto& contour = contours[i];
    // if (hierarchy[i][3] == -1){
    //   std::cout << "without parent contours" << std::endl;
    //   continue;
    // }   // Skip contours without parent contours

    if (cv::contourArea(contour) < min_area_thresh) {
      // std::cout << "min_area_thresh" << std::endl;
      continue;
    }

    auto epsilon = 0.02 * cv::arcLength(contour, true);
    vector<cv::Point> ploy;
    cv::approxPolyDP(contour, ploy, epsilon, true);
    if (cv::contourArea(ploy) < min_area_thresh) {
      // std::cout << "ploy min_area_thresh" << std::endl;
      continue;
    }

    ploys.push_back(ploy);
  }

  if (ploys.size() == 0) return 0;
  // std::cout << "we got ploys, num" << ploys.size() << std::endl;
  for (int i = 0; i < ploys.size(); i++) {
    vector<Eigen::Vector2i> box;
    cv::Rect rect = cv::boundingRect(ploys[i]);
    Eigen::Vector2i b_min(
        rect.y, rect.x);  // 注意：rect.width/height 表示宽度/高度，最大坐标需要计算
    Eigen::Vector2i b_max(rect.y + rect.height - 1, rect.x + rect.width - 1);
    box.push_back(b_min);
    box.push_back(b_max);
    boxs.push_back(box);
    // std::cout << "ploy " << i + 1 << "bmin(" << b_min(0) << "," << b_min(1) << ") bmax(" <<
    // b_max(0) << ","
    //           << b_max(1) << ")" << std::endl;
  }

  return 1;
}

/// @brief 接受一个高度矩阵，输出一个角度矩阵，矩阵中存储每个点对应的坡度角
/// @param raw_matrix 接收一个高度矩阵
/// @param angle_matrix 输出一个角度矩阵
/// @param bmin 接受高度矩阵对应的bmin
/// @return 如果成功，返回1
int OccmapProcessing::getAngleMatrix(
    const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& raw_matrix,
    Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>& o_angle_matrix,
    const Eigen::Vector3i& bmin) {

  Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> angle_matrix(
      raw_matrix.rows(), raw_matrix.cols());

  int fit_num = static_cast<int>(0.3 / sdf_map_->getResolution());
  std::cout << "fit num: " << fit_num << std::endl;
  for (int x = 0; x < raw_matrix.rows(); x++) {
    for (int y = 0; y < raw_matrix.cols(); y++) {
      // if(raw_matrix(x, y) < 0) {
      //   angle_matrix(x, y) = -1;
      //   continue;
      // }
      Eigen::Vector3d p_sur;
      sdf_map_->indexToPos(Eigen::Vector3i(x + bmin(0), y + bmin(1), 0), p_sur);
      p_sur(2) = raw_matrix(x, y);
      unsigned char tmp = getFitPlaneAngle(p_sur, 0.3);
      if (tmp == -1) continue;
      angle_matrix(x, y) =
          static_cast<unsigned char>(1.3 * tmp);  // 1.3用于把范围和unsigned char贴近
    }  // 0.5
  }
  // cout << "fillUnknown" << endl;
  // while (fillUnknown(angle_matrix));  // 填充矩阵中未知的angle

  Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
  std::cout << "show angle matrix" << std::endl;
  std::cout << angle_matrix.format(CleanFmt) << std::endl;

  o_angle_matrix = angle_matrix.cast<unsigned char>();

  return 1;
}

/// @brief 参考3x3范围内的已知值来填充未知值
/// @param angle_matrix 输入矩阵
/// @return 如果没有检查到未知角度，返回false，否则返回true
bool OccmapProcessing::fillUnknown(
    Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic>& angle_matrix) {
  bool flag = false;
  double unknown_i = 0.2;  // 一个惯性参数
  for (int x = 0; x < angle_matrix.rows(); x++) {
    for (int y = 0; y < angle_matrix.cols(); y++) {

      if (angle_matrix(x, y) >= 0) continue;  // 用周围几个单元的均值代替

      if (!flag) flag = true;
      double sum = 0;
      int n = 0;
      if (x < angle_matrix.rows() - 1 && angle_matrix(x + 1, y) >= 0) {
        n++;
        sum += angle_matrix(x + 1, y);
      }
      if (x < angle_matrix.rows() - 1 && y < angle_matrix.cols() - 1 &&
          angle_matrix(x + 1, y + 1) >= 0) {
        n++;
        sum += angle_matrix(x + 1, y + 1);
      }
      if (x < angle_matrix.rows() - 1 && y > 0 && angle_matrix(x + 1, y - 1) >= 0) {
        n++;
        sum += angle_matrix(x + 1, y - 1);
      }
      if (x > 0 && angle_matrix(x - 1, y) >= 0) {
        n++;
        sum += angle_matrix(x - 1, y);
      }
      if (x > 0 && y > 0 && angle_matrix(x - 1, y - 1) >= 0) {
        n++;
        sum += angle_matrix(x - 1, y - 1);
      }
      if (x > 0 && y < angle_matrix.cols() - 1 && angle_matrix(x - 1, y + 1) >= 0) {
        n++;
        sum += angle_matrix(x - 1, y + 1);
      }
      if (y > 0 && angle_matrix(x, y - 1) >= 0) {
        n++;
        sum += angle_matrix(x, y - 1);
      }
      if (y < angle_matrix.cols() - 1 && angle_matrix(x, y + 1) >= 0) {
        n++;
        sum += angle_matrix(x, y + 1);
      }
      if (n > 0) angle_matrix(x, y) = static_cast<int>(unknown_i * sum / n);
    }
  }
  return flag;
}

/// @brief 获取一个pos点小区域拟合平面的斜度
/// @param p_surface 拟合的中心点
/// @param radius world坐标下的拟合半径
/// @return 返回计算出来的角度
double OccmapProcessing::getFitPlaneAngle(Eigen::Vector3d& p_surface, const double& radius) {
  Plane tmp_Plane;
  tmp_Plane.init_coord = Eigen::Vector3d(p_surface(0), p_surface(1), p_surface(2));
  Eigen::Vector3i tmp_id;
  sdf_map_->posToIndex(p_surface, tmp_id);
  Eigen::Vector3d ball_center;
  sdf_map_->indexToPos(tmp_id, ball_center);
  double resolution = sdf_map_->getResolution();
  int fit_num = static_cast<int>(radius / resolution);
  // Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> vac(2 * fit_num + 1, 2 * fit_num + 1);
  // int vac_cout_init = (2 * fit_num + 1) * (2 * fit_num + 1);
  for (int i = -fit_num; i <= fit_num; i++)  // 遍历小区域，找到所在位置垂直上有occ的二维点坐标
  {
    for (int j = -fit_num; j <= fit_num; j++) {
      // vac(i + fit_num, j + fit_num) = false;
      for (int k = -10; k <= 10; k++) {
        Eigen::Vector3d point = ball_center + resolution * Eigen::Vector3d(i, j, k);

        if (sdf_map_->isInMap(point) &&  // 检查sdf map之中这个小范围内的所有occ点
            (sdf_map_->getOccupancy(point) == fast_planner::SDFMap::OCCUPIED)) {
          tmp_Plane.plane_pts.push_back(point);
          // if (!vac(i + fit_num, j + fit_num)) {
          //   vac(i + fit_num, j + fit_num) = true;
          //   vac_cout_init--;
          //}
        }
      }
    }
  }

  size_t pt_num = tmp_Plane.plane_pts.size();

  if (pt_num < 10) return 0;  // 点的数量太少,不进行平面拟合

  Eigen::Vector3d center;
  for (const auto& pt : tmp_Plane.plane_pts) center += pt;
  center /= pt_num;              // 找到所有点的平均
  Eigen::MatrixXd A(pt_num, 3);  // N×3
  for (size_t i = 0; i < pt_num; i++)
    A.row(i) = tmp_Plane.plane_pts[i] - center;  // 高度标准化，每一行填一个点

  // 进行奇异值分解操作
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullV);
  tmp_Plane.normal_vector = svd.matrixV().col(
      2);  // 矩阵 V，它的列向量是输入矩阵列空间的右奇异向量，按对应的奇异值大小排序（通常是降序）
  double angle_ = getAngle(tmp_Plane.normal_vector);  // 把奇异值最小的方向作为平面方向

  return angle_;
}

double OccmapProcessing::getAngle(Eigen::Vector3d& plane_vector) {
  Eigen::Vector3d n1(0, 0, 1);  // 在z方向上
  double cos_ = abs(n1(0) * plane_vector(0) + n1(1) * plane_vector(1) + n1(2) * plane_vector(2)) /
                (sqrt(n1(0) * n1(0) + n1(1) * n1(1) + n1(2) * n1(2)) *
                    sqrt(plane_vector(0) * plane_vector(0) + plane_vector(1) * plane_vector(1) +
                         plane_vector(2) * plane_vector(2)));
  double angle = std::acos(cos_);
  angle = angle * 180.0 / PI;
  if (angle > 90) angle = 180 - angle;
  return angle;
}

/**
 * @brief Calculates the specified percentile height from an Eigen matrix of integer values.
 *
 * The function expects the percentile as a fraction (e.g., 0.8 for the 80th percentile,
 * 0.95 for the 95th percentile).
 * It uses linear interpolation between the closest ranks for non-integer indices,
 * corresponding to Hyndman and Fan's R-7 method (similar to Excel's PERCENTILE.INC
 * or Python NumPy's default percentile).
 *
 * @param height_matrix The input Eigen matrix containing integer height values.
 * @param percentile_fraction The desired percentile as a fraction (must be between 0.0 and 1.0,
 * inclusive). For instance, 0.8 means the 80th percentile.
 * @return The calculated percentile height as a double.
 * @throws std::out_of_range if the matrix is empty.
 * @throws std::invalid_argument if the percentile_fraction is outside the [0.0, 1.0] range.
 */
double OccmapProcessing::calculateProperHeight(
    const Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic>& height_matrix,
    const double& percentile_fraction) {

  if (percentile_fraction < 0.0 || percentile_fraction > 1.0) {
    throw std::invalid_argument("Percentile fraction must be between 0.0 and 1.0 (inclusive).");
  }

  long long total_elements = height_matrix.size();
  if (total_elements == 0) {
    // Or return std::numeric_limits<double>::quiet_NaN(); if exceptions are not desired for this
    // case
    throw std::out_of_range("Input matrix is empty.");
  }

  // 1. Flatten the matrix into a std::vector<int>
  // Eigen matrices are column-major by default. Using .data() provides direct access.
  const int* data_ptr = height_matrix.data();
  std::vector<int> values(data_ptr, data_ptr + total_elements);

  // 2. Sort the vector
  std::sort(values.begin(), values.end());

  // Handle cases where N=1 (single element matrix)
  if (total_elements == 1) {
    return static_cast<double>(values[0]);
  }

  // 3. Calculate the 0-based fractional index
  // For percentile p_frac and N elements, the index i = p_frac * (N-1)
  double index_double = percentile_fraction * (static_cast<double>(total_elements) - 1.0);

  // 4. Interpolate if necessary
  // Check if index_double is effectively an integer
  if (std::abs(index_double - std::round(index_double)) <
          1e-9 * (static_cast<double>(total_elements) - 1.0) ||  // Relative tolerance
      std::abs(index_double - std::round(index_double)) <
          std::numeric_limits<double>::epsilon()) {  // Absolute tolerance for small indices
    size_t int_idx = static_cast<size_t>(std::round(index_double));
    // Ensure index is within bounds after rounding (can happen with p=0 or p=1 due to precision)
    if (int_idx >= total_elements) int_idx = total_elements - 1;
    return static_cast<double>(values[int_idx]);
  } else {
    size_t lower_idx = static_cast<size_t>(std::floor(index_double));
    size_t upper_idx = static_cast<size_t>(std::ceil(index_double));

    // Ensure upper_idx is not out of bounds, especially if index_double is very close to (N-1)
    if (upper_idx >= total_elements) {
      upper_idx = total_elements - 1;
    }
    // Ensure lower_idx is valid, especially for very small percentiles
    // if index_double is very close to 0, lower_idx can be 0, upper_idx can be 0 if N=1 or 1.
    if (lower_idx >= total_elements) {  // Should ideally not happen if upper_idx is capped and
                                        // total_elements > 0
      lower_idx = total_elements - 1;
    }
    // If after capping, lower becomes greater than upper (can happen if index_double was
    // extremely close to N-1 and rounded up)
    if (lower_idx > upper_idx && upper_idx == total_elements - 1) {
      lower_idx = upper_idx;
    }

    double value_lower = static_cast<double>(values[lower_idx]);
    double value_upper = static_cast<double>(values[upper_idx]);

    if (lower_idx == upper_idx) {  // This can happen if index_double was an integer or due to
                                   // capping
      return value_lower;
    }

    double fraction = index_double - static_cast<double>(lower_idx);
    return value_lower + fraction * (value_upper - value_lower);
  }
}

bool OccmapProcessing::expandEdge(const cv::Mat& img, int edge[], const int edgeID) {
  //[1] --初始化参数
  int nc = img.cols;
  int nr = img.rows;
  switch (edgeID) {
    case 0:
      if (edge[0] > nr) return false;
      for (int i = edge[3]; i <= edge[1]; ++i) {
        if (img.at<int>(edge[0], i) == -1)  // 遇见255像素表明碰到边缘线
          return false;
      }
      edge[0]++;
      return true;
      break;
    case 1:
      if (edge[1] > nc) return false;
      for (int i = edge[2]; i <= edge[0]; ++i) {
        if (img.at<int>(i, edge[1]) == -1)  // 遇见255像素表明碰到边缘线
          return false;
      }
      edge[1]++;
      return true;
      break;
    case 2:
      if (edge[2] < 0) return false;
      for (int i = edge[3]; i <= edge[1]; ++i) {
        if (img.at<int>(edge[2], i) == -1)  // 遇见255像素表明碰到边缘线
          return false;
      }
      edge[2]--;
      return true;
      break;
    case 3:
      if (edge[3] < 0) return false;
      for (int i = edge[2]; i <= edge[0]; ++i) {
        if (img.at<int>(i, edge[3]) == -1)  // 遇见255像素表明碰到边缘线
          return false;
      }
      edge[3]--;
      return true;
      break;
    default:
      return false;
      break;
  }
}

/// @brief 裁剪出img已知区域的最大外包围盒，然后填充盒中剩余的未知区域
/// @param img
/// @param output
/// @param basedelta 对原矩阵基点的修正值
/// @return 矩阵无效返回-1, 否则返回0
int OccmapProcessing::cutnfillAABBImg(const Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic>& img,
    Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>& output,
    Eigen::Vector2i& basedelta) {
  // std::cout << "hi" << std::endl;

  if (img.cols() == 0 || img.rows() == 0) return -1;
  Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
  // std::cout << img.format(CleanFmt) << std::endl;
  Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> tmp = img;
  for (int x = 0; x < tmp.rows(); x++) {
    for (int y = 0; y < tmp.cols(); y++) {
      if (tmp(x, y) == -1) {
        tmp(x, y) = 0;
      } else {
        tmp(x, y) = 255;
      }
    }
  }
  Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> selected_img =
      tmp.cast<unsigned char>();

  //std::cout << "cutnfillAABBImg: selected_img: " << std::endl;

  //std::cout << selected_img.cast<int>().format(CleanFmt) << std::endl;

  cv::Mat cv_img;
  cv::eigen2cv(selected_img, cv_img);

  // 查找轮廓，对应连通域
  std::vector<std::vector<cv::Point>> contourVecs;
  cv::findContours(cv_img, contourVecs, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

  cv::Rect maxRect;
  if (contourVecs.size() > 0) {  // 存在多个连通域，寻找最大连通域
    double maxArea = 0;
    std::vector<cv::Point> maxContour;
    for (size_t i = 0; i < contourVecs.size(); i++) {
      double area = cv::contourArea(contourVecs[i]);
      if (area > maxArea) {
        maxArea = area;
        maxContour = contourVecs[i];
      }
    }

    // 将轮廓转为矩形框
    maxRect = cv::boundingRect(maxContour);
  } else
    return -1;  // 找不到

  cv::Mat cv_raw_img;  // int
  eigen2cv(img, cv_raw_img);
  cv::Mat cutted_raw =
      cutMat(cv_raw_img, maxRect);  // 裁剪的对象是还没有二值化的img，后期还要处理剩下的-1

  if (cutted_raw.empty()) return -1;
  basedelta(0) = maxRect.y;
  basedelta(1) = maxRect.x;

  // 填充区域之中-1
  Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> ei_cutted_raw;
  cv2eigen(cutted_raw, ei_cutted_raw);
  while (fillUnknown(ei_cutted_raw));
  output = ei_cutted_raw.cast<unsigned char>();

  return 0;
}
