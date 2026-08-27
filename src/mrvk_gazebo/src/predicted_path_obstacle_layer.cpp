#include "mrvk_gazebo/predicted_path_obstacle_layer.hpp"

#include <nav2_costmap_2d/cost_values.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>

namespace mrvk_gazebo
{

PredictedPathObstacleLayer::PredictedPathObstacleLayer()
  : robot_x_(0.0),
    robot_y_(0.0),
    has_robot_pose_(false),
    obstacle_radius_(0.25),
    max_path_length_(3.0),
    robot_exclusion_radius_(0.55),
    path_staleness_tolerance_(0.5),
    cost_value_(static_cast<int>(nav2_costmap_2d::LETHAL_OBSTACLE))
{
}

void PredictedPathObstacleLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node)
    throw std::runtime_error{"PredictedPathObstacleLayer: failed to lock node"};

  current_ = true;

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("path_topic", rclcpp::ParameterValue(std::string("/predicted_obstacle_markers")));
  declareParameter("obstacle_radius", rclcpp::ParameterValue(0.25));
  declareParameter("max_path_length", rclcpp::ParameterValue(3.0));
  declareParameter("robot_exclusion_radius", rclcpp::ParameterValue(0.55));
  declareParameter("path_staleness_tolerance", rclcpp::ParameterValue(0.5));
  declareParameter("cost_value", rclcpp::ParameterValue(static_cast<int>(nav2_costmap_2d::LETHAL_OBSTACLE)));

  node->get_parameter(name_ + "." + "enabled", enabled_);
  node->get_parameter(name_ + "." + "path_topic", path_topic_);
  node->get_parameter(name_ + "." + "obstacle_radius", obstacle_radius_);
  node->get_parameter(name_ + "." + "max_path_length", max_path_length_);
  node->get_parameter(name_ + "." + "robot_exclusion_radius", robot_exclusion_radius_);
  node->get_parameter(name_ + "." + "path_staleness_tolerance", path_staleness_tolerance_);
  node->get_parameter(name_ + "." + "cost_value", cost_value_);

  cost_value_ = std::min(std::max(cost_value_, 1),
                         static_cast<int>(nav2_costmap_2d::LETHAL_OBSTACLE));

  rclcpp::QoS qos(1);
  marker_sub_ = node->create_subscription<visualization_msgs::msg::MarkerArray>(
      path_topic_, qos,
      std::bind(&PredictedPathObstacleLayer::markerArrayCallback, this, std::placeholders::_1));

  RCLCPP_INFO(node->get_logger(),
              "PredictedPathObstacleLayer initialised (topic=%s radius=%.2f max_len=%.2f excl=%.2f)",
              path_topic_.c_str(), obstacle_radius_,
              max_path_length_, robot_exclusion_radius_);
}

void PredictedPathObstacleLayer::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  tracked_paths_.clear();
}

void PredictedPathObstacleLayer::markerArrayCallback(
    const visualization_msgs::msg::MarkerArray::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto& marker : msg->markers)
  {
    if (marker.action == visualization_msgs::msg::Marker::DELETEALL)
    {
      tracked_paths_.clear();
      continue;
    }
    if (marker.action == visualization_msgs::msg::Marker::DELETE)
    {
      tracked_paths_.erase(marker.id);
      continue;
    }

    if (marker.points.empty())
    {
      tracked_paths_.erase(marker.id);
      continue;
    }

    TrackedPath tp;
    tp.points.reserve(marker.points.size());
    for (const auto& pt : marker.points)
      tp.points.push_back(Point2D{pt.x, pt.y});
    tp.stamp = rclcpp::Time(marker.header.stamp);

    tracked_paths_[marker.id] = std::move(tp);
  }
}

std::vector<PredictedPathObstacleLayer::Point2D>
PredictedPathObstacleLayer::truncatePath(const std::vector<Point2D>& raw) const
{
  std::vector<Point2D> pts;
  if (raw.empty()) return pts;

  pts.reserve(raw.size());
  double accumulated = 0.0;
  bool has_prev = false;
  Point2D prev{0.0, 0.0};

  for (const auto& cur : raw)
  {
    if (!has_prev)
    {
      pts.push_back(cur);
      prev = cur;
      has_prev = true;
      continue;
    }

    const double dx = cur.x - prev.x;
    const double dy = cur.y - prev.y;
    const double seg = std::hypot(dx, dy);
    if (seg < 1e-9) { prev = cur; continue; }

    if (accumulated + seg > max_path_length_)
    {
      const double remain = std::max(0.0, max_path_length_ - accumulated);
      const double ratio = remain / seg;
      Point2D truncated{prev.x + ratio * dx, prev.y + ratio * dy};
      pts.push_back(truncated);
      accumulated += remain;
      break;
    }

    pts.push_back(cur);
    accumulated += seg;
    prev = cur;
  }

  // Static / near-static obstacle: nearly all poses at the same spot. Skip so
  // the inflation layer does not blow a tiny disc into a big blob (the scan
  // marking in the ObstacleLayer already handles real static obstacles).
  if (accumulated < 0.20) pts.clear();

  return pts;
}

std::vector<std::vector<PredictedPathObstacleLayer::Point2D>>
PredictedPathObstacleLayer::buildTruncatedPaths(const rclcpp::Time& now) const
{
  std::vector<std::vector<Point2D>> paths;
  paths.reserve(tracked_paths_.size());

  for (const auto& [id, tp] : tracked_paths_)
  {
    if (tp.stamp.nanoseconds() == 0) continue;
    const double age = (now - tp.stamp).seconds();
    if (age < 0.0 || age > path_staleness_tolerance_) continue;

    auto truncated = truncatePath(tp.points);
    if (truncated.size() >= 2) paths.push_back(std::move(truncated));
  }

  return paths;
}

void PredictedPathObstacleLayer::updateBounds(double robot_x, double robot_y,
                                              double /*robot_yaw*/,
                                              double* min_x, double* min_y,
                                              double* max_x, double* max_y)
{
  std::lock_guard<std::mutex> lock(mutex_);
  robot_x_ = robot_x;
  robot_y_ = robot_y;
  has_robot_pose_ = true;

  if (!enabled_) return;

  auto node = node_.lock();
  if (!node) return;

  const auto paths = buildTruncatedPaths(node->now());
  if (paths.empty()) return;

  const double pad = obstacle_radius_ + 0.1;
  for (const auto& path : paths)
    for (const auto& p : path)
      expandBounds(min_x, min_y, max_x, max_y, p.x, p.y, pad);
}

void PredictedPathObstacleLayer::updateCosts(nav2_costmap_2d::Costmap2D& master_grid,
                                             int min_i, int min_j,
                                             int max_i, int max_j)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!enabled_) return;

  auto node = node_.lock();
  if (!node) return;

  const auto paths = buildTruncatedPaths(node->now());
  if (paths.empty()) return;

  const double rad2 = obstacle_radius_ * obstacle_radius_;
  const double excl2 = robot_exclusion_radius_ * robot_exclusion_radius_;
  const unsigned char cost = static_cast<unsigned char>(cost_value_);

  for (int j = min_j; j < max_j; ++j)
  {
    for (int i = min_i; i < max_i; ++i)
    {
      double wx = 0.0, wy = 0.0;
      master_grid.mapToWorld(static_cast<unsigned int>(i),
                             static_cast<unsigned int>(j), wx, wy);

      if (has_robot_pose_)
      {
        const double dxr = wx - robot_x_;
        const double dyr = wy - robot_y_;
        if (dxr * dxr + dyr * dyr < excl2)
          continue;
      }

      double best = std::numeric_limits<double>::infinity();
      for (const auto& path : paths)
      {
        for (std::size_t k = 1; k < path.size(); ++k)
        {
          const double d2 = pointToSegmentDistSq(wx, wy,
                                                 path[k - 1].x, path[k - 1].y,
                                                 path[k].x, path[k].y);
          if (d2 < best) best = d2;
          if (best < 1e-9) break;
        }
        if (best < 1e-9) break;
      }

      if (best > rad2) continue;

      const unsigned char old = master_grid.getCost(i, j);
      master_grid.setCost(i, j,
                          (old == nav2_costmap_2d::NO_INFORMATION)
                              ? cost
                              : std::max(old, cost));
    }
  }
}

double PredictedPathObstacleLayer::pointToSegmentDistSq(double px, double py,
                                                        double x0, double y0,
                                                        double x1, double y1) const
{
  const double dx = x1 - x0;
  const double dy = y1 - y0;
  const double len2 = dx * dx + dy * dy;
  if (len2 < 1e-12)
  {
    const double ddx = px - x0;
    const double ddy = py - y0;
    return ddx * ddx + ddy * ddy;
  }
  double t = ((px - x0) * dx + (py - y0) * dy) / len2;
  t = std::max(0.0, std::min(1.0, t));
  const double cx = x0 + t * dx;
  const double cy = y0 + t * dy;
  const double ex = px - cx;
  const double ey = py - cy;
  return ex * ex + ey * ey;
}

void PredictedPathObstacleLayer::expandBounds(double* min_x, double* min_y,
                                              double* max_x, double* max_y,
                                              double px, double py, double pad) const
{
  *min_x = std::min(*min_x, px - pad);
  *min_y = std::min(*min_y, py - pad);
  *max_x = std::max(*max_x, px + pad);
  *max_y = std::max(*max_y, py + pad);
}

}  // namespace mrvk_gazebo

PLUGINLIB_EXPORT_CLASS(mrvk_gazebo::PredictedPathObstacleLayer, nav2_costmap_2d::Layer)
