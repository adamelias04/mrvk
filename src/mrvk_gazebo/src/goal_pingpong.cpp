// ROS2 (rclcpp_action) port. Pingpong experiment: the robot alternates
// between two goals A <-> B N times via the Nav2 NavigateToPose action.
// Publishes /nav_tour/active (latched Bool) for nav_metrics:
//   true  -- before the first goal (start measuring)
//   false -- after the last goal   (finish and write results)
//
// Params (private namespace equivalents; defaults match (0,-6) <-> (0,6), 8x):
//   pose_a_x, pose_a_y / pose_b_x, pose_b_y  [m]
//   repeats        [-]  number of goals total (default 8)
//   goal_timeout   [s]  max time per goal (default 60)
//   pause_min, pause_max [s] random pause between goals (default 0..10)
//   seed           [-]  RNG seed (fixed default 42 -> identical pause sequence)
//   frame_id       [-]  goal frame (default "map")

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <chrono>
#include <cmath>
#include <memory>
#include <random>
#include <string>

using NavigateToPose = nav2_msgs::action::NavigateToPose;

static geometry_msgs::msg::PoseStamped makePose(const rclcpp::Node::SharedPtr& node,
                                                const std::string& frame,
                                                double x, double y, double yaw)
{
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = frame;
  p.header.stamp = node->now();
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.pose.position.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  p.pose.orientation = tf2::toMsg(q);
  return p;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("goal_pingpong");

  const double ax = node->declare_parameter<double>("pose_a_x", 0.0);
  const double ay = node->declare_parameter<double>("pose_a_y", -6.0);
  const double bx = node->declare_parameter<double>("pose_b_x", 0.0);
  const double by = node->declare_parameter<double>("pose_b_y", 6.0);
  const int repeats = node->declare_parameter<int>("repeats", 8);
  const double goal_timeout = node->declare_parameter<double>("goal_timeout", 60.0);
  double pause_min = node->declare_parameter<double>("pause_min", 0.0);
  double pause_max = node->declare_parameter<double>("pause_max", 10.0);
  const int seed = node->declare_parameter<int>("seed", 42);
  const std::string frame_id = node->declare_parameter<std::string>("frame_id", "map");

  if (pause_max < pause_min) std::swap(pause_min, pause_max);

  std::mt19937 rng(static_cast<uint32_t>(seed));
  std::uniform_real_distribution<double> pause_dist(pause_min, pause_max);

  if (repeats < 1)
  {
    RCLCPP_ERROR(node->get_logger(), "goal_pingpong: 'repeats' must be >= 1 (got %d)", repeats);
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::QoS latched(1);
  latched.transient_local();
  auto active_pub = node->create_publisher<std_msgs::msg::Bool>("/nav_tour/active", latched);

  RCLCPP_INFO(node->get_logger(), "goal_pingpong: A=(%.2f, %.2f)  B=(%.2f, %.2f)  repeats=%d",
              ax, ay, bx, by, repeats);

  auto client = rclcpp_action::create_client<NavigateToPose>(node, "navigate_to_pose");
  RCLCPP_INFO(node->get_logger(), "goal_pingpong: waiting for navigate_to_pose action server...");
  client->wait_for_action_server();
  RCLCPP_INFO(node->get_logger(), "goal_pingpong: nav2 ready.");

  {
    std_msgs::msg::Bool m;
    m.data = true;
    active_pub->publish(m);
  }
  rclcpp::sleep_for(std::chrono::milliseconds(500));

  int successes = 0;
  for (int i = 0; i < repeats && rclcpp::ok(); ++i)
  {
    const bool use_a = (i % 2 == 0);
    const double gx = use_a ? ax : bx;
    const double gy = use_a ? ay : by;
    const double yaw = std::atan2((use_a ? by : ay) - gy,
                                  (use_a ? bx : ax) - gx);

    NavigateToPose::Goal goal;
    goal.pose = makePose(node, frame_id, gx, gy, yaw);
    RCLCPP_INFO(node->get_logger(), "goal_pingpong [%d/%d]: -> %s (%.2f, %.2f)",
                i + 1, repeats, use_a ? "A" : "B", gx, gy);

    auto gh_future = client->async_send_goal(goal);
    if (rclcpp::spin_until_future_complete(node, gh_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_WARN(node->get_logger(), "goal_pingpong: send_goal %d failed.", i + 1);
    }
    else
    {
      auto goal_handle = gh_future.get();
      if (!goal_handle)
      {
        RCLCPP_WARN(node->get_logger(), "goal_pingpong: goal %d rejected.", i + 1);
      }
      else
      {
        auto result_future = client->async_get_result(goal_handle);
        const auto rc = rclcpp::spin_until_future_complete(
            node, result_future, std::chrono::duration<double>(goal_timeout));
        if (rc != rclcpp::FutureReturnCode::SUCCESS)
        {
          RCLCPP_WARN(node->get_logger(), "goal_pingpong: goal %d timed out, cancelling.", i + 1);
          client->async_cancel_goal(goal_handle);
        }
        else
        {
          const auto result = result_future.get();
          if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
          {
            RCLCPP_INFO(node->get_logger(), "goal_pingpong: goal %d SUCCEEDED.", i + 1);
            ++successes;
          }
          else
          {
            RCLCPP_WARN(node->get_logger(), "goal_pingpong: goal %d ended code=%d.",
                        i + 1, static_cast<int>(result.code));
          }
        }
      }
    }

    if (i + 1 < repeats && rclcpp::ok())
    {
      const double pause = pause_dist(rng);
      RCLCPP_INFO(node->get_logger(),
                  "goal_pingpong: pausing %.2f s before next goal (seed=%d, range=[%.1f, %.1f])",
                  pause, seed, pause_min, pause_max);
      if (pause > 0.0)
        rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(pause)));
    }
  }

  {
    std_msgs::msg::Bool m;
    m.data = false;
    active_pub->publish(m);
  }
  RCLCPP_INFO(node->get_logger(), "goal_pingpong: done. %d/%d goals SUCCEEDED.",
              successes, repeats);
  rclcpp::sleep_for(std::chrono::milliseconds(500));
  rclcpp::shutdown();
  return 0;
}
