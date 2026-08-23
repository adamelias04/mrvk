// ROS2 port of the Gazebo Classic point-and-click "2D Nav Goal" GUI tool.
// Toggle the button, then click (or click-drag for orientation) on the ground
// plane in the Gazebo 3D view; the world point is published as a PoseStamped
// on /goal_pose for Nav2 (ROS1 original published to /move_base_simple/goal).
#pragma once

#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <gazebo/common/MouseEvent.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/gui/GuiPlugin.hh>
#include <gazebo/rendering/rendering.hh>

#include <QPushButton>

namespace gazebo
{
class GazeboGoalTool : public GUIPlugin
{
public:
  GazeboGoalTool();
  virtual ~GazeboGoalTool();

  void Load(sdf::ElementPtr _sdf) override;

private:
  void OnToggle(bool _checked);
  void OnPreRender();
  bool OnMousePress(const common::MouseEvent &_event);
  bool OnMouseRelease(const common::MouseEvent &_event);

  bool ScreenToGround(
      const common::MouseEvent &_event,
      ignition::math::Vector3d &_worldPoint) const;

  void PublishGoal(
      const ignition::math::Vector3d &_start,
      const ignition::math::Vector3d &_end);

private:
  QPushButton *toggleButton_{nullptr};

  bool toolEnabled_{false};
  bool dragActive_{false};
  bool cameraReady_{false};

  std::string mouseFilterName_{"gazebo_goal_tool_filter"};
  std::string frameId_{"odom"};
  std::string topic_{"/goal_pose"};
  int posX_{170};
  int posY_{455};
  double groundZ_{0.0};

  ignition::math::Vector3d dragStart_;

  rendering::UserCameraPtr userCamera_;
  event::ConnectionPtr preRenderConn_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goalPub_;
  // Spun in a background thread so the node's clock follows /clock
  // (use_sim_time) and goal stamps match Nav2's simulated time.
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread spinThread_;
};
}  // namespace gazebo
