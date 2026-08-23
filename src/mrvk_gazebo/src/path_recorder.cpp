// ROS2 (rclcpp) port. Accumulates robot positions from /odom into a
// nav_msgs/Path for RViz. A point is added only when the robot has moved at
// least ~min_distance (so the path does not grow unbounded).
//
// Params (declared on the node):
//   odom_topic    (string, "/shoddy/odom")
//   path_topic    (string, "/traveled_path")
//   frame_id      (string, "odom")
//   min_distance  (double, 0.05)  min move between points [m]
//   max_points    (int,    0)     ring buffer limit (0 = unlimited)
//   publish_rate  (double, 5.0)   Hz for publishing the whole Path
//   reset_topic   (string, "/traveled_path/reset")  std_msgs/Empty -> clear

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/empty.hpp>

#include <cmath>
#include <deque>
#include <memory>
#include <string>

class PathRecorder : public rclcpp::Node
{
public:
  PathRecorder() : rclcpp::Node("path_recorder")
  {
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/shoddy/odom");
    path_topic_ = declare_parameter<std::string>("path_topic", "/traveled_path");
    frame_id_ = declare_parameter<std::string>("frame_id", "odom");
    reset_topic_ = declare_parameter<std::string>("reset_topic", "/traveled_path/reset");
    min_distance_ = declare_parameter<double>("min_distance", 0.05);
    // Skok pozicie vacsi ako toto = teleport/respawn -> stopa sa vymaze.
    reset_jump_distance_ = declare_parameter<double>("reset_jump_distance", 1.0);
    max_points_ = declare_parameter<int>("max_points", 0);
    const double rate = declare_parameter<double>("publish_rate", 5.0);

    // Latched equivalent: transient_local so late RViz subscribers get the path.
    rclcpp::QoS latched(1);
    latched.transient_local();
    path_pub_ = create_publisher<nav_msgs::msg::Path>(path_topic_, latched);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, 50,
        std::bind(&PathRecorder::odomCb, this, std::placeholders::_1));
    reset_sub_ = create_subscription<std_msgs::msg::Empty>(
        reset_topic_, 1,
        std::bind(&PathRecorder::resetCb, this, std::placeholders::_1));

    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / std::max(rate, 0.1)),
        std::bind(&PathRecorder::publishPath, this));

    RCLCPP_INFO(get_logger(),
             "[path_recorder] odom=%s -> path=%s frame=%s min_dist=%.3fm max_pts=%d",
             odom_topic_.c_str(), path_topic_.c_str(), frame_id_.c_str(),
             min_distance_, max_points_);
  }

private:
  void odomCb(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    const double x = msg->pose.pose.position.x;
    const double y = msg->pose.pose.position.y;
    const double t = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

    if (have_last_)
    {
      const double dx = x - last_x_;
      const double dy = y - last_y_;
      const double jump2 = dx * dx + dy * dy;

      // Reset sveta v Gazebe (sim cas skoci dozadu) alebo teleport robota
      // (velky skok pozicie) -> zacni novu stopu, nespajaj so starou.
      if (t < last_t_ - 0.5 || jump2 > reset_jump_distance_ * reset_jump_distance_)
      {
        RCLCPP_INFO(get_logger(),
                    "[path_recorder] simulacia restartovana/teleport -> stopa vymazana.");
        poses_.clear();
        have_last_ = false;
      }
      else if (jump2 < min_distance_ * min_distance_)
      {
        last_t_ = t;
        return;
      }
    }

    geometry_msgs::msg::PoseStamped p;
    p.header.stamp = msg->header.stamp;
    p.header.frame_id = frame_id_;
    p.pose = msg->pose.pose;
    poses_.push_back(p);

    if (max_points_ > 0 && static_cast<int>(poses_.size()) > max_points_)
      poses_.pop_front();

    last_x_ = x;
    last_y_ = y;
    last_t_ = t;
    have_last_ = true;
  }

  void resetCb(const std_msgs::msg::Empty::ConstSharedPtr)
  {
    poses_.clear();
    have_last_ = false;
    RCLCPP_INFO(get_logger(), "[path_recorder] path reset.");
    publishPath();
  }

  void publishPath()
  {
    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = frame_id_;
    path.poses.assign(poses_.begin(), poses_.end());
    path_pub_->publish(path);
  }

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string odom_topic_, path_topic_, frame_id_, reset_topic_;
  double min_distance_;
  double reset_jump_distance_;
  int max_points_;

  std::deque<geometry_msgs::msg::PoseStamped> poses_;
  double last_x_ = 0.0, last_y_ = 0.0, last_t_ = 0.0;
  bool have_last_ = false;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathRecorder>());
  rclcpp::shutdown();
  return 0;
}
