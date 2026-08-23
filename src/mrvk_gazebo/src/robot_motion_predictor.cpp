// ROS2 (rclcpp) port of the RobotMotionPredictor safety filter that sits
// between the local planner (TEB/DWB/Trajectory) and the robot. It forward
// simulates the commanded velocity against the local costmap and against the
// dynamic obstacle prediction (/predicted_obstacle_path) and, depending on
// time-to-collision and head-on/crossing classification, scales / stops /
// reverses / swerves the command. See inline comments for the full logic
// (unchanged from the ROS1 original apart from the rclcpp API).

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>
#include <limits>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace
{

struct Point2D
{
  double x;
  double y;
};

class RobotMotionPredictor : public rclcpp::Node
{
public:
  RobotMotionPredictor()
    : rclcpp::Node("robot_motion_predictor")
  {
    cmd_in_topic_ = declare_parameter<std::string>("cmd_vel_in_topic", "/cmd_vel_raw");
    cmd_out_topic_ = declare_parameter<std::string>("cmd_vel_out_topic", "/shoddy/cmd_vel");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom");
    local_costmap_topic_ = declare_parameter<std::string>("local_costmap_topic",
                                                          "/local_costmap/costmap");
    obstacle_path_topic_ = declare_parameter<std::string>("obstacle_path_topic",
                                                         "/predicted_obstacle_path");
    prediction_path_topic_ = declare_parameter<std::string>("prediction_path_topic",
                                                          "/robot_motion_prediction");
    prediction_marker_topic_ = declare_parameter<std::string>("prediction_marker_topic",
                                                            "/robot_motion_prediction_marker");

    prediction_time_ = declare_parameter<double>("prediction_time", 1.5);
    viz_rate_ = declare_parameter<double>("viz_rate", 20.0);
    marker_line_width_ = declare_parameter<double>("marker_line_width", 0.05);
    marker_z_ = declare_parameter<double>("marker_z", 0.10);
    prediction_dt_ = declare_parameter<double>("prediction_dt", 0.1);
    slow_horizon_ = declare_parameter<double>("slow_horizon", 1.0);
    stop_horizon_ = declare_parameter<double>("stop_horizon", 0.4);
    reverse_horizon_ = declare_parameter<double>("reverse_horizon", 0.2);
    min_check_linear_velocity_ = declare_parameter<double>("min_check_linear_velocity", 0.02);
    min_check_angular_velocity_ = declare_parameter<double>("min_check_angular_velocity", 0.05);
    lethal_threshold_ = declare_parameter<int>("lethal_threshold", 85);
    reverse_velocity_ = declare_parameter<double>("reverse_velocity", -0.15);
    reverse_clear_probe_distance_ = declare_parameter<double>("reverse_clear_probe_distance", 0.45);
    publish_prediction_path_ = declare_parameter<bool>("publish_prediction_path", true);

    temporal_safety_distance_ = declare_parameter<double>("temporal_safety_distance", 0.48);
    temporal_path_max_age_ = declare_parameter<double>("temporal_path_max_age", 1.0);
    obstacle_prediction_dt_ = declare_parameter<double>("obstacle_prediction_dt", 0.1);
    temporal_prediction_time_ = declare_parameter<double>("temporal_prediction_time", 3.0);
    temporal_match_window_ = declare_parameter<double>("temporal_match_window", 2.0);

    escape_horizon_ = declare_parameter<double>("escape_horizon", 1.5);
    escape_v_ = declare_parameter<double>("escape_linear_velocity", 0.25);
    escape_w_ = declare_parameter<double>("escape_angular_velocity", 0.6);

    headon_v_ = declare_parameter<double>("headon_forward_velocity", 0.25);
    headon_w_ = declare_parameter<double>("headon_swerve_angular", 0.7);
    headon_rev_v_ = declare_parameter<double>("headon_reverse_velocity", -0.25);
    headon_horizon_ = declare_parameter<double>("headon_horizon", 2.0);
    headon_side_lock_time_ = declare_parameter<double>("headon_side_lock_time", 2.0);
    headon_side_lock_bonus_ = declare_parameter<double>("headon_side_lock_bonus", 0.25);
    headon_hard_lock_ = declare_parameter<bool>("headon_hard_lock", true);
    headon_min_feasible_steps_ = declare_parameter<int>("headon_min_feasible_steps", 2);

    cmd_latch_time_ = declare_parameter<double>("cmd_latch_time", 0.6);

    if (prediction_dt_ < 1e-3)
      prediction_dt_ = 1e-3;
    if (prediction_time_ < prediction_dt_)
      prediction_time_ = prediction_dt_;

    last_headon_time_ = zeroTime();
    cmd_latch_until_ = zeroTime();

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_out_topic_, 1);
    path_pub_ = create_publisher<nav_msgs::msg::Path>(prediction_path_topic_, 1);
    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(prediction_marker_topic_, 1);

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        cmd_in_topic_, 10,
        std::bind(&RobotMotionPredictor::cmdCallback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, 10,
        std::bind(&RobotMotionPredictor::odomCallback, this, std::placeholders::_1));
    costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        local_costmap_topic_, rclcpp::QoS(1),
        std::bind(&RobotMotionPredictor::costmapCallback, this, std::placeholders::_1));
    obstacle_path_sub_ = create_subscription<nav_msgs::msg::Path>(
        obstacle_path_topic_, 1,
        std::bind(&RobotMotionPredictor::obstaclePathCallback, this, std::placeholders::_1));

    if (viz_rate_ > 0.0)
    {
      viz_timer_ = create_wall_timer(
          std::chrono::duration<double>(1.0 / viz_rate_),
          std::bind(&RobotMotionPredictor::vizTimerCallback, this));
    }

    RCLCPP_INFO(get_logger(),
             "RobotMotionPredictor: in=%s out=%s odom=%s costmap=%s horizon=%.2fs viz=%.1fHz",
             cmd_in_topic_.c_str(), cmd_out_topic_.c_str(),
             odom_topic_.c_str(), local_costmap_topic_.c_str(),
             prediction_time_, viz_rate_);
  }

private:
  rclcpp::Time zeroTime() const { return rclcpp::Time(0, 0, RCL_ROS_TIME); }
  static bool isZero(const rclcpp::Time& t) { return t.nanoseconds() == 0; }

  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_odom_ = *msg;
    has_odom_ = true;
  }

  void costmapCallback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_costmap_ = *msg;
    has_costmap_ = true;
  }

  void obstaclePathCallback(const nav_msgs::msg::Path::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_obstacle_path_ = *msg;
    has_obstacle_path_ = !msg->poses.empty();
  }

  struct PredictionResult
  {
    nav_msgs::msg::Path path;
    double t_collision;  // -1 if no collision within horizon
  };

  struct EscapeCandidate
  {
    double v;
    double w;
    const char* name;
  };

  struct EscapeScore
  {
    bool feasible = false;
    double t_exit = 0.0;
    double min_obs_dist = std::numeric_limits<double>::infinity();
  };

  struct HeadOnCandidate
  {
    double v;
    double w;
    int side;          // -1 = R, 0 = neutral, +1 = L
    const char* name;
  };

  struct HeadOnScore
  {
    bool feasible = false;
    double min_obs_dist = std::numeric_limits<double>::infinity();
  };

  PredictionResult runPrediction(const nav_msgs::msg::Odometry& odom,
                                  const nav_msgs::msg::OccupancyGrid& costmap,
                                  double v, double w) const
  {
    PredictionResult res;
    res.t_collision = -1.0;
    res.path.header.stamp = now();
    res.path.header.frame_id = costmap.header.frame_id.empty()
                                   ? odom.header.frame_id
                                   : costmap.header.frame_id;

    const double yaw0 = yawFromQuaternion(odom.pose.pose.orientation);
    double cx = odom.pose.pose.position.x;
    double cy = odom.pose.pose.position.y;
    double cyaw = yaw0;

    const int n_steps = static_cast<int>(std::ceil(prediction_time_ / prediction_dt_));
    res.path.poses.reserve(n_steps);
    for (int i = 1; i <= n_steps; ++i)
    {
      cyaw += w * prediction_dt_;
      cx += v * std::cos(cyaw) * prediction_dt_;
      cy += v * std::sin(cyaw) * prediction_dt_;

      geometry_msgs::msg::PoseStamped p;
      p.header = res.path.header;
      p.pose.position.x = cx;
      p.pose.position.y = cy;
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, cyaw);
      p.pose.orientation = tf2::toMsg(q);
      res.path.poses.push_back(p);

      if (res.t_collision < 0.0 && cellLethal(costmap, cx, cy))
        res.t_collision = static_cast<double>(i) * prediction_dt_;
    }
    return res;
  }

  const char* simpleMode(double t_collision) const
  {
    if (t_collision < 0.0 || t_collision >= slow_horizon_)
      return "PASS";
    if (t_collision >= stop_horizon_)
      return "SLOW";
    if (t_collision >= reverse_horizon_)
      return "STOP";
    return "HARDSTOP";
  }

  void cmdCallback(const geometry_msgs::msg::Twist::ConstSharedPtr msg)
  {
    geometry_msgs::msg::Twist out = *msg;

    nav_msgs::msg::Odometry odom;
    nav_msgs::msg::OccupancyGrid costmap;
    bool have_state = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (has_odom_ && has_costmap_)
      {
        odom = latest_odom_;
        costmap = latest_costmap_;
        have_state = true;
      }
    }

    if (!have_state)
    {
      cmd_pub_->publish(out);
      return;
    }

    // ---- Command latch --------------------------------------------------
    {
      const rclcpp::Time now_latch = now();
      if (!isZero(cmd_latch_until_) && now_latch < cmd_latch_until_)
      {
        bool break_latch = false;
        {
          nav_msgs::msg::Path obs_peek;
          bool have_obs_peek = false;
          {
            std::lock_guard<std::mutex> lock(mutex_);
            if (has_obstacle_path_)
            {
              obs_peek = latest_obstacle_path_;
              have_obs_peek = true;
            }
          }
          if (have_obs_peek)
          {
            const TemporalConflict tc_peek = checkTemporalConflict(
                odom, obs_peek, msg->linear.x, msg->angular.z);
            if (tc_peek.present && !isHeadOnScenario(odom, obs_peek))
              break_latch = true;
          }
        }

        if (break_latch)
        {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                            "RobotMotionPredictor: %s LATCH broken (scenario "
                            "now CROSSING, was reverse cmd=%.2f)",
                            cmd_latch_reason_.c_str(),
                            cmd_latch_value_.linear.x);
          cmd_latch_until_ = zeroTime();
        }
        else
        {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                            "RobotMotionPredictor: %s LATCH hold (v=%.2f w=%.2f) "
                            "remaining=%.2fs cmd=(v=%.2f w=%.2f)",
                            cmd_latch_reason_.c_str(),
                            cmd_latch_value_.linear.x, cmd_latch_value_.angular.z,
                            (cmd_latch_until_ - now_latch).seconds(),
                            msg->linear.x, msg->angular.z);
          cmd_pub_->publish(cmd_latch_value_);
          return;
        }
      }
    }

    // ---- Lethal escape mode ---------------------------------------------
    if (cellLethal(costmap, odom.pose.pose.position.x, odom.pose.pose.position.y))
    {
      nav_msgs::msg::Path obs_path_local;
      const nav_msgs::msg::Path* obs_ptr = nullptr;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (has_obstacle_path_)
        {
          obs_path_local = latest_obstacle_path_;
          obs_ptr = &obs_path_local;
        }
      }

      geometry_msgs::msg::Twist esc;
      const char* esc_mode = nullptr;
      std::string esc_scores;
      if (selectEscape(odom, costmap, obs_ptr, esc, esc_mode, esc_scores))
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                          "RobotMotionPredictor: LETHAL_ESCAPE picked=%s (v=%.2f w=%.2f) "
                          "(cmd v=%.2f w=%.2f) scores:%s",
                          esc_mode, esc.linear.x, esc.angular.z,
                          msg->linear.x, msg->angular.z,
                          esc_scores.c_str());
        if (esc.linear.x < 0.0)
        {
          cmd_latch_until_ = now() + rclcpp::Duration::from_seconds(cmd_latch_time_);
          cmd_latch_value_ = esc;
          cmd_latch_reason_ = std::string("LETHAL_ESCAPE/") + esc_mode;
        }
        cmd_pub_->publish(esc);
        return;
      }

      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                        "RobotMotionPredictor: LETHAL_NO_ESCAPE (hardstop, "
                        "cmd v=%.2f w=%.2f) scores:%s",
                        msg->linear.x, msg->angular.z,
                        esc_scores.c_str());
      out.linear.x = 0.0;
      out.linear.y = 0.0;
      out.angular.z = 0.0;
      cmd_pub_->publish(out);
      return;
    }

    const double v_cmd = msg->linear.x;
    const double w_cmd = msg->angular.z;

    if (std::fabs(v_cmd) < min_check_linear_velocity_ &&
        std::fabs(w_cmd) < min_check_angular_velocity_)
    {
      cmd_pub_->publish(out);
      return;
    }

    const PredictionResult pred = runPrediction(odom, costmap, v_cmd, w_cmd);
    const double t_collision = pred.t_collision;

    const char* mode = "PASS";
    if (t_collision < 0.0 || t_collision >= slow_horizon_)
    {
      // passthrough
    }
    else if (t_collision >= stop_horizon_)
    {
      const double span = std::max(slow_horizon_ - stop_horizon_, 1e-3);
      double scale = (t_collision - stop_horizon_) / span;
      scale = std::max(0.0, std::min(1.0, scale));
      out.linear.x = msg->linear.x * scale;
      out.linear.y = msg->linear.y * scale;
      mode = "SLOW";
    }
    else if (t_collision >= reverse_horizon_)
    {
      out.linear.x = 0.0;
      out.linear.y = 0.0;
      mode = "STOP";
    }
    else
    {
      const Point2D p0{odom.pose.pose.position.x, odom.pose.pose.position.y};
      const double yaw0 = yawFromQuaternion(odom.pose.pose.orientation);
      const bool was_forward = msg->linear.x > 0.0;
      const bool rear_clear = was_forward ? isRearClear(costmap, p0, yaw0) : false;
      if (was_forward && rear_clear)
      {
        out.linear.x = reverse_velocity_;
        out.linear.y = 0.0;
        out.angular.z = 0.0;
        mode = "REVERSE";
      }
      else
      {
        out.linear.x = 0.0;
        out.linear.y = 0.0;
        out.angular.z = 0.0;
        mode = "HARDSTOP";
      }
    }

    if (t_collision >= 0.0)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                        "RobotMotionPredictor: %s (TTC=%.2fs cmd v=%.2f w=%.2f -> v=%.2f w=%.2f)",
                        mode, t_collision, v_cmd, w_cmd, out.linear.x, out.angular.z);
    }

    // ---- Temporal overlap layer ---------------------------------------
    nav_msgs::msg::Path obs_path_local;
    bool have_obs_path_local = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (has_obstacle_path_)
      {
        obs_path_local = latest_obstacle_path_;
        have_obs_path_local = true;
      }
    }

    if (have_obs_path_local)
    {
      const TemporalConflict tc =
          checkTemporalConflict(odom, obs_path_local, out.linear.x, out.angular.z);

      const bool following = isFollowingScenario(odom, obs_path_local);

      if (tc.present && !following)
      {
        const char* tmode = nullptr;

        const TemporalConflict tc_stop =
            checkTemporalConflict(odom, obs_path_local, 0.0, 0.0);
        const bool head_on = tc_stop.present &&
                             isHeadOnScenario(odom, obs_path_local);

        if (!head_on)
        {
          out.linear.x = 0.0;
          out.linear.y = 0.0;
          tmode = "TEMPORAL_STOP";

          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                            "RobotMotionPredictor: %s (t_conflict=%.2fs dist=%.2fm "
                            "cmd v=%.2f w=%.2f -> v=%.2f w=%.2f)",
                            tmode, tc.t_conflict, tc.dist, v_cmd, w_cmd,
                            out.linear.x, out.angular.z);
        }
        else
        {
          geometry_msgs::msg::Twist hd;
          const char* hd_mode = nullptr;
          std::string hd_scores;
          if (selectHeadOnManeuver(odom, costmap, obs_path_local,
                                    hd, hd_mode, hd_scores))
          {
            out.linear.x = hd.linear.x;
            out.linear.y = 0.0;
            out.angular.z = hd.angular.z;
            tmode = hd_mode;
            if (hd.linear.x < 0.0)
            {
              cmd_latch_until_ = now() + rclcpp::Duration::from_seconds(cmd_latch_time_);
              cmd_latch_value_ = hd;
              cmd_latch_reason_ = std::string("HEADON/") + hd_mode;
            }
          }
          else
          {
            out.linear.x = 0.0;
            out.linear.y = 0.0;
            out.angular.z = 0.0;
            tmode = "TEMPORAL_HEADON_STUCK";
          }

          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                            "RobotMotionPredictor: %s (t_conflict=%.2fs dist=%.2fm "
                            "cmd v=%.2f w=%.2f -> v=%.2f w=%.2f) scores:%s",
                            tmode, tc.t_conflict, tc.dist, v_cmd, w_cmd,
                            out.linear.x, out.angular.z,
                            hd_scores.c_str());
        }
      }
    }

    cmd_pub_->publish(out);
  }

  void vizTimerCallback()
  {
    if (!publish_prediction_path_)
      return;

    nav_msgs::msg::Odometry odom;
    nav_msgs::msg::OccupancyGrid costmap;
    bool have_odom_local = false;
    bool have_costmap_local = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      have_odom_local = has_odom_;
      have_costmap_local = has_costmap_;
      if (have_odom_local)
        odom = latest_odom_;
      if (have_costmap_local)
        costmap = latest_costmap_;
    }

    if (!have_odom_local || !have_costmap_local)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                        "RobotMotionPredictor viz: waiting for topics (odom=%s costmap=%s).",
                        have_odom_local ? "OK" : "MISSING",
                        have_costmap_local ? "OK" : "MISSING");
      return;
    }

    const double v = odom.twist.twist.linear.x;
    const double w = odom.twist.twist.angular.z;

    const std::string frame = costmap.header.frame_id.empty()
                                  ? odom.header.frame_id
                                  : costmap.header.frame_id;

    if (std::fabs(v) < min_check_linear_velocity_ &&
        std::fabs(w) < min_check_angular_velocity_)
    {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                        "RobotMotionPredictor viz: robot stopped (v=%.3f w=%.3f), marker off.",
                        v, w);
      publishEmptyPath(frame);
      publishEmptyMarker(frame);
      return;
    }

    const PredictionResult pred = runPrediction(odom, costmap, v, w);

    nav_msgs::msg::Path obs_path_local;
    bool have_obs_path_local = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (has_obstacle_path_)
      {
        obs_path_local = latest_obstacle_path_;
        have_obs_path_local = true;
      }
    }

    const char* mode = simpleMode(pred.t_collision);
    double t_conflict_viz = -1.0;
    if (have_obs_path_local)
    {
      const TemporalConflict tc = checkTemporalConflict(odom, obs_path_local, v, w);
      if (tc.present)
      {
        mode = "TEMPORAL";
        t_conflict_viz = tc.t_conflict;
      }
    }

    path_pub_->publish(pred.path);
    publishMarker(pred.path, mode);

    if (t_conflict_viz >= 0.0)
    {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                        "RobotMotionPredictor viz: publishing %zu points (v=%.2f w=%.2f frame=%s mode=%s t_conflict=%.2fs)",
                        pred.path.poses.size(), v, w, frame.c_str(), mode, t_conflict_viz);
    }
    else
    {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                        "RobotMotionPredictor viz: publishing %zu points (v=%.2f w=%.2f frame=%s mode=%s)",
                        pred.path.poses.size(), v, w, frame.c_str(), mode);
    }
  }

  void publishMarker(const nav_msgs::msg::Path& path, const char* mode)
  {
    visualization_msgs::msg::Marker marker;
    marker.header = path.header;
    marker.ns = "robot_motion_prediction";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = marker_line_width_;
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);

    std::string m = mode ? mode : "PASS";
    if (m == "SLOW")
    {
      marker.color.r = 1.0; marker.color.g = 1.0; marker.color.b = 0.0;
    }
    else if (m == "STOP")
    {
      marker.color.r = 1.0; marker.color.g = 0.5; marker.color.b = 0.0;
    }
    else if (m == "REVERSE")
    {
      marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 1.0;
    }
    else if (m == "HARDSTOP")
    {
      marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0;
    }
    else if (m == "TEMPORAL")
    {
      marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 1.0;
    }
    else
    {
      marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 0.0;
    }
    marker.color.a = 0.9;

    marker.points.reserve(path.poses.size());
    for (const auto& p : path.poses)
    {
      geometry_msgs::msg::Point pt;
      pt.x = p.pose.position.x;
      pt.y = p.pose.position.y;
      pt.z = marker_z_;
      marker.points.push_back(pt);
    }

    marker_pub_->publish(marker);
  }

  void publishEmptyMarker(const std::string& frame_id)
  {
    if (!publish_prediction_path_)
      return;
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = now();
    marker.header.frame_id = frame_id;
    marker.ns = "robot_motion_prediction";
    marker.id = 0;
    marker.action = visualization_msgs::msg::Marker::DELETE;
    marker_pub_->publish(marker);
  }

  bool cellLethal(const nav_msgs::msg::OccupancyGrid& cm, double wx, double wy) const
  {
    const double res = cm.info.resolution;
    if (res <= 0.0)
      return false;
    const double ox = cm.info.origin.position.x;
    const double oy = cm.info.origin.position.y;
    const int W = static_cast<int>(cm.info.width);
    const int H = static_cast<int>(cm.info.height);

    const int mx = static_cast<int>(std::floor((wx - ox) / res));
    const int my = static_cast<int>(std::floor((wy - oy) / res));
    if (mx < 0 || my < 0 || mx >= W || my >= H)
      return false;

    const std::size_t idx = static_cast<std::size_t>(my) * static_cast<std::size_t>(W) +
                            static_cast<std::size_t>(mx);
    if (idx >= cm.data.size())
      return false;
    const int v = static_cast<int>(cm.data[idx]);
    return v >= lethal_threshold_;
  }

  bool isFollowingScenario(const nav_msgs::msg::Odometry& odom,
                            const nav_msgs::msg::Path& obs_path) const
  {
    if (obs_path.poses.size() < 2) return false;

    const double age = (now() - rclcpp::Time(obs_path.header.stamp)).seconds();
    if (age > temporal_path_max_age_) return false;

    const auto& p0 = obs_path.poses.front().pose.position;
    const auto& p1 = obs_path.poses[1].pose.position;
    const double ovx = p1.x - p0.x;
    const double ovy = p1.y - p0.y;
    const double ospeed = std::hypot(ovx, ovy);
    if (ospeed < 1e-3) return false;

    const double yaw = yawFromQuaternion(odom.pose.pose.orientation);
    const double hx = std::cos(yaw);
    const double hy = std::sin(yaw);

    const double rx = p0.x - odom.pose.pose.position.x;
    const double ry = p0.y - odom.pose.pose.position.y;
    const double forward_proj = rx * hx + ry * hy;
    if (forward_proj <= 0.0) return false;

    const double align = (ovx * hx + ovy * hy) / ospeed;
    return align > 0.5;
  }

  bool isHeadOnScenario(const nav_msgs::msg::Odometry& odom,
                        const nav_msgs::msg::Path& obs_path) const
  {
    if (obs_path.poses.size() < 2) return false;

    const double age = (now() - rclcpp::Time(obs_path.header.stamp)).seconds();
    if (age > temporal_path_max_age_) return false;

    const auto& p0 = obs_path.poses.front().pose.position;
    const auto& p1 = obs_path.poses[1].pose.position;
    const double ovx = p1.x - p0.x;
    const double ovy = p1.y - p0.y;
    const double ospeed = std::hypot(ovx, ovy);
    if (ospeed < 1e-3) return false;

    const double yaw = yawFromQuaternion(odom.pose.pose.orientation);
    const double hx = std::cos(yaw);
    const double hy = std::sin(yaw);

    const double align = (ovx * hx + ovy * hy) / ospeed;
    return align < -0.5;
  }

  struct TemporalConflict
  {
    bool   present    = false;
    int    step       = -1;
    double t_conflict = 0.0;
    double dist       = 0.0;
  };

  double minObsDistInTimeWindow(const nav_msgs::msg::Path& obs_path,
                                double t_abs, double cx, double cy) const
  {
    const int n_obs = static_cast<int>(obs_path.poses.size());
    if (n_obs == 0) return std::numeric_limits<double>::infinity();
    const int k_mid  = static_cast<int>(std::round(t_abs / obstacle_prediction_dt_));
    const int half_w = std::max(0, static_cast<int>(std::round(
        temporal_match_window_ / obstacle_prediction_dt_)));
    const int k_lo = std::max(0, k_mid - half_w);
    const int k_hi = std::min(n_obs - 1, k_mid + half_w);
    if (k_lo > k_hi) return std::numeric_limits<double>::infinity();
    double d_min = std::numeric_limits<double>::infinity();
    for (int k = k_lo; k <= k_hi; ++k)
    {
      const auto& op = obs_path.poses[k].pose.position;
      const double d = std::hypot(cx - op.x, cy - op.y);
      if (d < d_min) d_min = d;
    }
    return d_min;
  }

  TemporalConflict checkTemporalConflict(const nav_msgs::msg::Odometry& odom,
                                         const nav_msgs::msg::Path& obs_path,
                                         double v, double w) const
  {
    TemporalConflict tc;
    if (obs_path.poses.empty())
      return tc;

    const double age = (now() - rclcpp::Time(obs_path.header.stamp)).seconds();
    if (age > temporal_path_max_age_)
      return tc;

    const double yaw0 = yawFromQuaternion(odom.pose.pose.orientation);
    double cx = odom.pose.pose.position.x;
    double cy = odom.pose.pose.position.y;
    double cyaw = yaw0;

    const int n_steps = static_cast<int>(std::ceil(temporal_prediction_time_ / prediction_dt_));
    const double base_offset = age;

    for (int i = 1; i <= n_steps; ++i)
    {
      cyaw += w * prediction_dt_;
      cx   += v * std::cos(cyaw) * prediction_dt_;
      cy   += v * std::sin(cyaw) * prediction_dt_;

      const double t_abs = base_offset + static_cast<double>(i) * prediction_dt_;
      const double d = minObsDistInTimeWindow(obs_path, t_abs, cx, cy);
      if (d < temporal_safety_distance_)
      {
        tc.present    = true;
        tc.step       = i;
        tc.t_conflict = static_cast<double>(i) * prediction_dt_;
        tc.dist       = d;
        return tc;
      }
    }
    return tc;
  }

  EscapeScore evaluateEscape(const nav_msgs::msg::Odometry& odom,
                              const nav_msgs::msg::OccupancyGrid& cm,
                              const nav_msgs::msg::Path* obs_path,
                              double v, double w) const
  {
    EscapeScore s;

    const double yaw0 = yawFromQuaternion(odom.pose.pose.orientation);
    double cx = odom.pose.pose.position.x;
    double cy = odom.pose.pose.position.y;
    double cyaw = yaw0;

    const int n_steps = static_cast<int>(std::ceil(escape_horizon_ / prediction_dt_));

    double obs_age = 0.0;
    bool obs_valid = false;
    if (obs_path != nullptr && !obs_path->poses.empty())
    {
      obs_age = (now() - rclcpp::Time(obs_path->header.stamp)).seconds();
      obs_valid = (obs_age <= temporal_path_max_age_);
    }

    for (int i = 1; i <= n_steps; ++i)
    {
      cyaw += w * prediction_dt_;
      cx   += v * std::cos(cyaw) * prediction_dt_;
      cy   += v * std::sin(cyaw) * prediction_dt_;

      if (!s.feasible && !cellLethal(cm, cx, cy))
      {
        s.feasible = true;
        s.t_exit = static_cast<double>(i) * prediction_dt_;
      }

      if (obs_valid)
      {
        const double t_abs = obs_age + static_cast<double>(i) * prediction_dt_;
        const double d = minObsDistInTimeWindow(*obs_path, t_abs, cx, cy);
        if (std::isfinite(d))
          s.min_obs_dist = std::min(s.min_obs_dist, d);
      }
    }
    return s;
  }

  bool selectEscape(const nav_msgs::msg::Odometry& odom,
                    const nav_msgs::msg::OccupancyGrid& cm,
                    const nav_msgs::msg::Path* obs_path,
                    geometry_msgs::msg::Twist& out,
                    const char*& mode_out,
                    std::string& scores_out) const
  {
    const EscapeCandidate candidates[] = {
        { +escape_v_,  0.0,          "ESC_FWD"   },
        { -escape_v_,  0.0,          "ESC_REV"   },
        { +escape_v_, +escape_w_,    "ESC_FWD_L" },
        { +escape_v_, -escape_w_,    "ESC_FWD_R" },
        { -escape_v_, +escape_w_,    "ESC_REV_L" },
        { -escape_v_, -escape_w_,    "ESC_REV_R" },
        {  0.0,       +escape_w_,    "ESC_ROT_L" },
        {  0.0,       -escape_w_,    "ESC_ROT_R" },
    };

    int best_idx = -1;
    double best_score = std::numeric_limits<double>::infinity();
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2);

    for (std::size_t i = 0; i < sizeof(candidates) / sizeof(candidates[0]); ++i)
    {
      const EscapeScore s = evaluateEscape(odom, cm, obs_path,
                                           candidates[i].v, candidates[i].w);

      oss << " " << candidates[i].name << "[";
      if (!s.feasible) {
        oss << "BLOCKED]";
        continue;
      }

      double score = s.t_exit;
      const bool penalized = (s.min_obs_dist < temporal_safety_distance_);
      if (penalized)
        score += 10.0;

      if (std::isfinite(s.min_obs_dist))
        score -= 0.01 * std::min(s.min_obs_dist, 3.0);

      oss << "t=" << s.t_exit
          << " d=" << (std::isfinite(s.min_obs_dist) ? s.min_obs_dist : -1.0)
          << " s=" << score
          << (penalized ? " !" : "")
          << "]";

      if (score < best_score)
      {
        best_score = score;
        best_idx = static_cast<int>(i);
      }
    }

    scores_out = oss.str();

    if (best_idx < 0)
      return false;

    out.linear.x = candidates[best_idx].v;
    out.linear.y = 0.0;
    out.angular.z = candidates[best_idx].w;
    mode_out = candidates[best_idx].name;
    return true;
  }

  HeadOnScore evaluateHeadOnCandidate(const nav_msgs::msg::Odometry& odom,
                                       const nav_msgs::msg::OccupancyGrid& cm,
                                       const nav_msgs::msg::Path& obs_path,
                                       double v, double w) const
  {
    HeadOnScore s;
    s.feasible = false;

    const double yaw0 = yawFromQuaternion(odom.pose.pose.orientation);
    double cx = odom.pose.pose.position.x;
    double cy = odom.pose.pose.position.y;
    double cyaw = yaw0;

    const int n_steps = static_cast<int>(std::ceil(headon_horizon_ / prediction_dt_));
    const double obs_age = (now() - rclcpp::Time(obs_path.header.stamp)).seconds();
    const int n_obs = static_cast<int>(obs_path.poses.size());
    const bool obs_valid = (obs_age <= temporal_path_max_age_) && (n_obs > 0);

    for (int i = 1; i <= n_steps; ++i)
    {
      cyaw += w * prediction_dt_;
      cx   += v * std::cos(cyaw) * prediction_dt_;
      cy   += v * std::sin(cyaw) * prediction_dt_;

      const bool lethal_here = cellLethal(cm, cx, cy);

      if (lethal_here && i <= headon_min_feasible_steps_)
      {
        return s;
      }

      s.feasible = true;

      if (lethal_here)
        break;

      if (obs_valid)
      {
        const double t_abs = obs_age + static_cast<double>(i) * prediction_dt_;
        const double d = minObsDistInTimeWindow(obs_path, t_abs, cx, cy);
        if (std::isfinite(d))
          s.min_obs_dist = std::min(s.min_obs_dist, d);
      }
    }
    return s;
  }

  bool selectHeadOnManeuver(const nav_msgs::msg::Odometry& odom,
                             const nav_msgs::msg::OccupancyGrid& cm,
                             const nav_msgs::msg::Path& obs_path,
                             geometry_msgs::msg::Twist& out,
                             const char*& mode_out,
                             std::string& scores_out) const
  {
    const HeadOnCandidate rev_turn_candidates[] = {
        {  headon_rev_v_,+headon_w_,  +1, "HEADON_REV_L"    },
        {  headon_rev_v_,-headon_w_,  -1, "HEADON_REV_R"    },
    };
    const HeadOnCandidate fallback_candidates[] = {
        {  headon_rev_v_, 0.0,         0, "HEADON_REV"      },
        { +headon_v_,    +headon_w_,  +1, "HEADON_SWERVE_L" },
        { +headon_v_,    -headon_w_,  -1, "HEADON_SWERVE_R" },
    };

    const rclcpp::Time now_t = now();
    const bool lock_active = (last_headon_side_ != 0) &&
                              !isZero(last_headon_time_) &&
                              ((now_t - last_headon_time_).seconds() < headon_side_lock_time_);

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2);

    auto score_group = [&](const HeadOnCandidate* arr, std::size_t n,
                            int& best_idx, double& best_score) {
      best_idx = -1;
      best_score = -std::numeric_limits<double>::infinity();
      for (std::size_t i = 0; i < n; ++i)
      {
        if (headon_hard_lock_ && lock_active &&
            arr[i].side != 0 && arr[i].side != last_headon_side_)
        {
          oss << " " << arr[i].name << "[LOCKED]";
          continue;
        }

        const HeadOnScore s = evaluateHeadOnCandidate(odom, cm, obs_path,
                                                       arr[i].v, arr[i].w);
        oss << " " << arr[i].name << "[";
        if (!s.feasible)
        {
          oss << "BLOCKED]";
          continue;
        }
        double score = std::isfinite(s.min_obs_dist) ? s.min_obs_dist : 999.0;
        const bool boosted = (lock_active && arr[i].side == last_headon_side_);
        if (boosted) score += headon_side_lock_bonus_;
        oss << "d=" << (std::isfinite(s.min_obs_dist) ? s.min_obs_dist : -1.0)
            << " s=" << score
            << (boosted ? " *" : "")
            << "]";
        if (score > best_score)
        {
          best_score = score;
          best_idx = static_cast<int>(i);
        }
      }
    };

    int rev_turn_best = -1;
    double rev_turn_score = 0.0;
    score_group(rev_turn_candidates,
                sizeof(rev_turn_candidates) / sizeof(rev_turn_candidates[0]),
                rev_turn_best, rev_turn_score);

    const HeadOnCandidate* picked = nullptr;
    if (rev_turn_best >= 0)
    {
      picked = &rev_turn_candidates[rev_turn_best];
    }
    else
    {
      int fb_best = -1;
      double fb_score = 0.0;
      score_group(fallback_candidates,
                  sizeof(fallback_candidates) / sizeof(fallback_candidates[0]),
                  fb_best, fb_score);
      if (fb_best >= 0)
        picked = &fallback_candidates[fb_best];
    }

    scores_out = oss.str();

    if (!picked)
      return false;

    out.linear.x = picked->v;
    out.linear.y = 0.0;
    out.angular.z = picked->w;
    mode_out = picked->name;

    if (picked->side != 0)
    {
      last_headon_side_ = picked->side;
      last_headon_time_ = now_t;
    }
    return true;
  }

  bool isRearClear(const nav_msgs::msg::OccupancyGrid& cm, const Point2D& base, double yaw) const
  {
    const double cy = std::cos(yaw);
    const double sy = std::sin(yaw);
    const double step = 0.10;
    const int n = std::max(1, static_cast<int>(std::ceil(reverse_clear_probe_distance_ / step)));
    for (int i = 1; i <= n; ++i)
    {
      const double d = static_cast<double>(i) * step;
      const double px = base.x - cy * d;
      const double py = base.y - sy * d;
      if (cellLethal(cm, px, py))
        return false;
    }
    return true;
  }

  static double yawFromQuaternion(const geometry_msgs::msg::Quaternion& q)
  {
    tf2::Quaternion tq;
    tf2::fromMsg(q, tq);
    double r = 0.0;
    double p = 0.0;
    double y = 0.0;
    tf2::Matrix3x3(tq).getRPY(r, p, y);
    return y;
  }

  void publishEmptyPath(const std::string& frame_id)
  {
    if (!publish_prediction_path_)
      return;
    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = frame_id;
    path_pub_->publish(path);
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr obstacle_path_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr viz_timer_;

  std::mutex mutex_;
  nav_msgs::msg::Odometry latest_odom_;
  nav_msgs::msg::OccupancyGrid latest_costmap_;
  nav_msgs::msg::Path latest_obstacle_path_;
  bool has_odom_ = false;
  bool has_costmap_ = false;
  bool has_obstacle_path_ = false;

  std::string cmd_in_topic_;
  std::string cmd_out_topic_;
  std::string odom_topic_;
  std::string local_costmap_topic_;
  std::string obstacle_path_topic_;
  std::string prediction_path_topic_;
  std::string prediction_marker_topic_;

  double prediction_time_;
  double viz_rate_;
  double marker_line_width_;
  double marker_z_;
  double prediction_dt_;
  double slow_horizon_;
  double stop_horizon_;
  double reverse_horizon_;
  double min_check_linear_velocity_;
  double min_check_angular_velocity_;
  int lethal_threshold_;
  double reverse_velocity_;
  double reverse_clear_probe_distance_;
  bool publish_prediction_path_;

  double temporal_safety_distance_;
  double temporal_path_max_age_;
  double obstacle_prediction_dt_;
  double temporal_prediction_time_;
  double temporal_match_window_;

  double escape_horizon_;
  double escape_v_;
  double escape_w_;

  double headon_v_;
  double headon_w_;
  double headon_rev_v_;
  double headon_horizon_;
  double headon_side_lock_time_;
  double headon_side_lock_bonus_;
  bool   headon_hard_lock_;
  int    headon_min_feasible_steps_;
  mutable int    last_headon_side_ = 0;
  mutable rclcpp::Time last_headon_time_{0, 0, RCL_ROS_TIME};

  double cmd_latch_time_;
  mutable rclcpp::Time cmd_latch_until_{0, 0, RCL_ROS_TIME};
  mutable geometry_msgs::msg::Twist cmd_latch_value_;
  mutable std::string cmd_latch_reason_;
};

}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotMotionPredictor>());
  rclcpp::shutdown();
  return 0;
}
