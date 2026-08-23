#include "detected_path_layer/detected_path_layer.hpp"

#include <nav2_costmap_2d/cost_values.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <algorithm>
#include <memory>

namespace detected_path_layer
{

void DetectedPathLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node)
    throw std::runtime_error{"DetectedPathLayer: failed to lock node"};

  current_ = true;
  declareParameter("enabled", rclcpp::ParameterValue(false));
  node->get_parameter(name_ + "." + "enabled", enabled_);

  sub_ = node->create_subscription<path_detection::msg::DetectedPath>(
      "/shoddy/detected_path", 10,
      std::bind(&DetectedPathLayer::pathDetectionCallback, this, std::placeholders::_1));

  RCLCPP_INFO(node->get_logger(), "DetectedPathLayer initialised (enabled=%d)", enabled_);
}

void DetectedPathLayer::pathDetectionCallback(
    const path_detection::msg::DetectedPath::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  frame_height_ = msg->height;
  frame_width_ = msg->width;
  frame_ = msg->frame;
  has_frame_ = true;
}

void DetectedPathLayer::updateBounds(double origin_x, double origin_y, double origin_yaw,
                                     double* min_x, double* min_y,
                                     double* max_x, double* max_y)
{
  if (!enabled_)
    return;

  frameToCostmap(origin_x, origin_y, origin_yaw);

  std::lock_guard<std::mutex> lock(mutex_);
  if (!mark_x_.empty())
  {
    *min_x = std::min(*min_x, bmin_x_);
    *min_y = std::min(*min_y, bmin_y_);
    *max_x = std::max(*max_x, bmax_x_);
    *max_y = std::max(*max_y, bmax_y_);
  }
}

void DetectedPathLayer::updateCosts(nav2_costmap_2d::Costmap2D& master_grid,
                                    int /*min_i*/, int /*min_j*/, int /*max_i*/, int /*max_j*/)
{
  if (!enabled_)
    return;

  std::lock_guard<std::mutex> lock(mutex_);
  unsigned int x = 0, y = 0;
  for (std::size_t i = 0; i < mark_x_.size(); ++i)
  {
    if (master_grid.worldToMap(mark_x_[i], mark_y_[i], x, y))
      master_grid.setCost(x, y, nav2_costmap_2d::LETHAL_OBSTACLE);
  }
}

void DetectedPathLayer::frameToCostmap(double robot_x, double robot_y, double robot_yaw)
{
  std::lock_guard<std::mutex> lock(mutex_);
  mark_x_.clear();
  mark_y_.clear();

  if (!has_frame_ || frame_width_ == 0 || frame_.empty())
    return;

  double phi = 0.0, r = 0.0, x = 0.0, y = 0.0;

  for (unsigned int h = FRAME_STARTING_IDX; h < FRAME_ENDING_IDX && h < frame_height_; ++h)
  {
    for (unsigned int w = 0; w < frame_width_; ++w)
    {
      const std::size_t idx = static_cast<std::size_t>(h) * frame_width_ + w;
      if (idx >= frame_.size())
        continue;
      if (!frame_[idx])
      {
        phi = std::atan2(h - frame_height_ / 2.0, CAMERA_FOCAL_LENGTH);

        r = -CAMERA_HEIGHT * (std::cos(PHI - phi) / std::sin(PHI - phi));
        x = CAMERA_X + r;

        r = r * COS_CAMERA_PITCH + CAMERA_HEIGHT * SIN_CAMERA_PITCH;
        y = -r / CAMERA_FOCAL_LENGTH * (w - frame_width_ / 2.0);

        mark_x_.push_back(robot_x + x * std::cos(robot_yaw) - y * std::sin(robot_yaw));
        mark_y_.push_back(robot_y + x * std::sin(robot_yaw) + y * std::cos(robot_yaw));
      }
    }
  }

  if (!mark_x_.empty())
  {
    bmin_x_ = *std::min_element(mark_x_.begin(), mark_x_.end());
    bmin_y_ = *std::min_element(mark_y_.begin(), mark_y_.end());
    bmax_x_ = *std::max_element(mark_x_.begin(), mark_x_.end());
    bmax_y_ = *std::max_element(mark_y_.begin(), mark_y_.end());
  }
}

}  // namespace detected_path_layer

PLUGINLIB_EXPORT_CLASS(detected_path_layer::DetectedPathLayer, nav2_costmap_2d::Layer)
