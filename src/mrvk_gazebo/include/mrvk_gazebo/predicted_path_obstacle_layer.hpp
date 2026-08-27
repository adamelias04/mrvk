#ifndef MRVK_GAZEBO__PREDICTED_PATH_OBSTACLE_LAYER_HPP_
#define MRVK_GAZEBO__PREDICTED_PATH_OBSTACLE_LAYER_HPP_

#include <map>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <nav2_costmap_2d/layer.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace mrvk_gazebo
{

// ROS2 / Nav2 port. Stamps LETHAL cells along every currently active
// predicted-obstacle track (read from /predicted_obstacle_markers, a
// MarkerArray with one LINE_STRIP per tracked object, keyed by marker id)
// directly into the master grid in updateCosts. No persistence: every
// costmap update cycle recomputes from the current marker set, and DELETE
// actions remove a track's path immediately, so stale predictions disappear
// once the publisher stops including that id (or sends a DELETE).
class PredictedPathObstacleLayer : public nav2_costmap_2d::Layer
{
public:
  PredictedPathObstacleLayer();

  void onInitialize() override;
  void updateBounds(double robot_x, double robot_y, double robot_yaw,
                    double* min_x, double* min_y,
                    double* max_x, double* max_y) override;
  void updateCosts(nav2_costmap_2d::Costmap2D& master_grid,
                   int min_i, int min_j, int max_i, int max_j) override;
  void reset() override;
  bool isClearable() override { return false; }

private:
  struct Point2D
  {
    double x;
    double y;
  };

  struct TrackedPath
  {
    std::vector<Point2D> points;
    rclcpp::Time stamp;
  };

  void markerArrayCallback(const visualization_msgs::msg::MarkerArray::ConstSharedPtr msg);

  std::vector<std::vector<Point2D>> buildTruncatedPaths(const rclcpp::Time& now) const;
  std::vector<Point2D> truncatePath(const std::vector<Point2D>& raw) const;

  double pointToSegmentDistSq(double px, double py,
                              double x0, double y0,
                              double x1, double y1) const;

  void expandBounds(double* min_x, double* min_y,
                    double* max_x, double* max_y,
                    double px, double py, double pad) const;

  std::mutex mutex_;

  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr marker_sub_;

  std::map<int, TrackedPath> tracked_paths_;

  double robot_x_;
  double robot_y_;
  bool has_robot_pose_;

  std::string path_topic_;
  double obstacle_radius_;
  double max_path_length_;
  double robot_exclusion_radius_;
  double path_staleness_tolerance_;
  int cost_value_;
};

}  // namespace mrvk_gazebo

#endif  // MRVK_GAZEBO__PREDICTED_PATH_OBSTACLE_LAYER_HPP_
