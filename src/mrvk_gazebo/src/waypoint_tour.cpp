// ROS2 (rclcpp_action) port. Sends the robot through a sequence of waypoints
// via the Nav2 NavigateToPose action. Publishes /nav_tour/active=true before
// the first goal and =false after the last so nav_metrics can measure the
// whole tour as one run.
//
// Default waypoints: (-6,-6) -> (-3,5) -> (3,0)
//
// Params (declared on node):
//   wpN_x / wpN_y  (N=1..8) waypoint coordinates (wp4..wp8 optional)
//   frame_id       (str, "map")
//   goal_timeout   (double, 120) max time per waypoint [s]
//   repeats        (int, 1) how many times to repeat the tour
//   pause_between  (double, 1.0) pause between tours [s]

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cmath>
#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <utility>

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

static geometry_msgs::msg::PoseStamped makePose(const rclcpp::Node::SharedPtr& node,
                                                const std::string& frame,
                                                double x, double y, double yaw)
{
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = frame;
  p.header.stamp = node->now();
  p.pose.position.x = x;
  p.pose.position.y = y;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  p.pose.orientation = tf2::toMsg(q);
  return p;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("waypoint_tour");

  const std::string frame = node->declare_parameter<std::string>("frame_id", "map");
  const double goal_timeout = node->declare_parameter<double>("goal_timeout", 120.0);
  const int repeats = node->declare_parameter<int>("repeats", 1);
  const double pause_between = node->declare_parameter<double>("pause_between", 1.0);

  std::vector<std::pair<double, double>> wps = {
    {node->declare_parameter<double>("wp1_x", -6.0), node->declare_parameter<double>("wp1_y", -6.0)},
    {node->declare_parameter<double>("wp2_x", -3.0), node->declare_parameter<double>("wp2_y",  5.0)},
    {node->declare_parameter<double>("wp3_x",  3.0), node->declare_parameter<double>("wp3_y",  0.0)},
  };
  for (int n = 4; n <= 8; ++n)
  {
    const std::string kx = "wp" + std::to_string(n) + "_x";
    const std::string ky = "wp" + std::to_string(n) + "_y";
    const double x = node->declare_parameter<double>(kx, std::numeric_limits<double>::quiet_NaN());
    const double y = node->declare_parameter<double>(ky, std::numeric_limits<double>::quiet_NaN());
    if (std::isfinite(x) && std::isfinite(y))
      wps.emplace_back(x, y);
  }

  rclcpp::QoS latched(1);
  latched.transient_local();
  auto tour_pub = node->create_publisher<std_msgs::msg::Bool>("/nav_tour/active", latched);

  auto client = rclcpp_action::create_client<NavigateToPose>(node, "navigate_to_pose");
  RCLCPP_INFO(node->get_logger(), "[waypoint_tour] waiting for navigate_to_pose action server...");
  if (!client->wait_for_action_server(std::chrono::seconds(30)))
  {
    RCLCPP_ERROR(node->get_logger(), "[waypoint_tour] navigate_to_pose unavailable.");
    rclcpp::shutdown();
    return 1;
  }
  RCLCPP_INFO(node->get_logger(), "[waypoint_tour] connected. %zu waypoints, repeats=%d",
              wps.size(), repeats);

  for (int rep = 0; rep < repeats && rclcpp::ok(); ++rep)
  {
    RCLCPP_INFO(node->get_logger(), "[waypoint_tour] ===== TOUR %d/%d START =====", rep + 1, repeats);

    std_msgs::msg::Bool act;
    act.data = true;
    tour_pub->publish(act);
    rclcpp::sleep_for(std::chrono::milliseconds(300));

    for (std::size_t i = 0; i < wps.size() && rclcpp::ok(); ++i)
    {
      const auto& cur = wps[i];
      double yaw = 0.0;
      if (i + 1 < wps.size())
        yaw = std::atan2(wps[i + 1].second - cur.second,
                         wps[i + 1].first  - cur.first);

      RCLCPP_INFO(node->get_logger(), "[waypoint_tour] WP %zu/%zu -> (%.2f, %.2f)",
                  i + 1, wps.size(), cur.first, cur.second);

      NavigateToPose::Goal goal;
      goal.pose = makePose(node, frame, cur.first, cur.second, yaw);

      auto gh_future = client->async_send_goal(goal);
      if (rclcpp::spin_until_future_complete(node, gh_future) !=
          rclcpp::FutureReturnCode::SUCCESS)
      {
        RCLCPP_WARN(node->get_logger(), "[waypoint_tour] send_goal failed, skipping WP %zu.", i + 1);
        continue;
      }
      auto goal_handle = gh_future.get();
      if (!goal_handle)
      {
        RCLCPP_WARN(node->get_logger(), "[waypoint_tour] WP %zu rejected.", i + 1);
        continue;
      }

      auto result_future = client->async_get_result(goal_handle);
      const auto rc = rclcpp::spin_until_future_complete(
          node, result_future, std::chrono::duration<double>(goal_timeout));
      if (rc != rclcpp::FutureReturnCode::SUCCESS)
      {
        RCLCPP_WARN(node->get_logger(), "[waypoint_tour] WP %zu timeout (%.0fs), cancelling.",
                    i + 1, goal_timeout);
        client->async_cancel_goal(goal_handle);
        continue;
      }

      const auto result = result_future.get();
      RCLCPP_INFO(node->get_logger(), "[waypoint_tour] WP %zu -> code=%d",
                  i + 1, static_cast<int>(result.code));
    }

    act.data = false;
    tour_pub->publish(act);
    RCLCPP_INFO(node->get_logger(), "[waypoint_tour] ===== TOUR %d/%d END =====", rep + 1, repeats);

    if (rep + 1 < repeats && rclcpp::ok())
      rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(pause_between)));
  }

  RCLCPP_INFO(node->get_logger(), "[waypoint_tour] done (%d tours).", repeats);
  rclcpp::shutdown();
  return 0;
}
