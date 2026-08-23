#ifndef DETECTED_PATH_LAYER__DETECTED_PATH_LAYER_HPP_
#define DETECTED_PATH_LAYER__DETECTED_PATH_LAYER_HPP_

#include <cmath>
#include <mutex>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <nav2_costmap_2d/layer.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>
#include <path_detection/msg/detected_path.hpp>

namespace detected_path_layer
{

// ROS2 / Nav2 port of the camera walkable-path costmap layer. Marks LETHAL
// cells where the walkable path was NOT detected; the detected path stays free.
class DetectedPathLayer : public nav2_costmap_2d::Layer
{
public:
  DetectedPathLayer() = default;

  void onInitialize() override;
  void updateBounds(double origin_x, double origin_y, double origin_yaw,
                    double* min_x, double* min_y, double* max_x, double* max_y) override;
  void updateCosts(nav2_costmap_2d::Costmap2D& master_grid,
                   int min_i, int min_j, int max_i, int max_j) override;
  void reset() override {}
  bool isClearable() override { return false; }

private:
  void pathDetectionCallback(const path_detection::msg::DetectedPath::ConstSharedPtr msg);
  void frameToCostmap(double robot_x, double robot_y, double robot_yaw);

  rclcpp::Subscription<path_detection::msg::DetectedPath>::SharedPtr sub_;

  std::mutex mutex_;
  unsigned int frame_height_{0};
  unsigned int frame_width_{0};
  std::vector<unsigned char> frame_;
  bool has_frame_{false};

  std::vector<double> mark_x_;
  std::vector<double> mark_y_;
  double bmin_x_{0.0}, bmin_y_{0.0}, bmax_x_{0.0}, bmax_y_{0.0};

  static constexpr double PI = 3.1415926536;

  // Camera parameters (from the ROS1 original).
  const double CAMERA_FOCAL_LENGTH = 474.4268835;
  const double CAMERA_X = 0.1946454825;
  const double CAMERA_HEIGHT = 0.5706493713;
  const double CAMERA_PITCH = PI / 8;
  const double PHI = PI - CAMERA_PITCH;
  const double COS_CAMERA_PITCH = std::cos(CAMERA_PITCH);
  const double SIN_CAMERA_PITCH = std::sin(CAMERA_PITCH);
  const unsigned int FRAME_STARTING_IDX = 280;
  const unsigned int FRAME_ENDING_IDX = 550;
};

}  // namespace detected_path_layer

#endif  // DETECTED_PATH_LAYER__DETECTED_PATH_LAYER_HPP_
