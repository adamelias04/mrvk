// ROS2 (rclcpp) port of the dynamic obstacle detector/predictor.
// Detects and tracks a single dynamic obstacle from a 2D lidar scan using an
// alpha-beta filter (position+velocity) and a constant-velocity prediction.
// Publishes /detected_obstacle_pose (instantaneous position) and
// /predicted_obstacle_path (predicted trajectory).

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <vector>
#include <string>
#include <cmath>
#include <limits>
#include <algorithm>
#include <memory>

struct ScanPoint
{
    double x_laser;
    double y_laser;
    double range;
    double angle;
    int index;
};

struct Segment
{
    std::vector<ScanPoint> points;
    double mean_range;
    double mean_angle;
    double width;
};

struct Point2D
{
    double x;
    double y;
};

class DynamicObstaclePredictor : public rclcpp::Node
{
public:
    DynamicObstaclePredictor()
        : rclcpp::Node("dynamic_obstacle_predictor")
    {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        loadParams();
        setupIO();
        resetTrack();
        RCLCPP_INFO(get_logger(),
                    "dynamic_obstacle_predictor started (scan=%s frame=%s)",
                    scan_topic_.c_str(), output_frame_.c_str());
    }

private:
    rclcpp::Time zeroTime() const
    {
        return rclcpp::Time(0, 0, RCL_ROS_TIME);
    }

    static bool isZero(const rclcpp::Time& t)
    {
        return t.nanoseconds() == 0;
    }

    void loadParams()
    {
        scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
        output_frame_ = declare_parameter<std::string>("output_frame", "odom");

        prediction_time_ = declare_parameter<double>("prediction_time", 1.0);
        prediction_dt_ = declare_parameter<double>("prediction_dt", 0.1);
        alpha_gain_ = declare_parameter<double>("alpha_gain", 0.65);
        beta_gain_ = declare_parameter<double>("beta_gain", 0.20);
        max_speed_ = declare_parameter<double>("max_speed", 2.0);
        track_timeout_ = declare_parameter<double>("track_timeout", 0.8);
        lost_track_prediction_time_ = declare_parameter<double>("lost_track_prediction_time", 1.0);
        prediction_hold_time_ = declare_parameter<double>("prediction_hold_time", 0.8);
        prediction_hold_speed_epsilon_ = declare_parameter<double>("prediction_hold_speed_epsilon", 0.08);
        min_dynamic_speed_ = declare_parameter<double>("min_dynamic_speed", 0.35);
        min_dynamic_persist_ = declare_parameter<double>("min_dynamic_persist", 0.5);

        point_spacing_ = declare_parameter<double>("point_spacing", 0.05);
        corridor_radius_ = declare_parameter<double>("corridor_radius", 0.18);
        object_half_extent_ = declare_parameter<double>("object_half_extent", 0.18);
        z_height_ = declare_parameter<double>("z_height", 0.10);

        scan_range_min_use_ = declare_parameter<double>("scan_range_min_use", 0.10);
        scan_range_max_use_ = declare_parameter<double>("scan_range_max_use", 8.00);
        segment_jump_distance_ = declare_parameter<double>("segment_jump_distance", 0.30);
        segment_min_points_ = declare_parameter<int>("segment_min_points", 2);
        segment_max_points_ = declare_parameter<int>("segment_max_points", 120);
        segment_min_width_ = declare_parameter<double>("segment_min_width", 0.03);
        segment_max_width_ = declare_parameter<double>("segment_max_width", 2.00);

        use_angle_roi_ = declare_parameter<bool>("use_angle_roi", false);
        angle_min_roi_ = declare_parameter<double>("angle_min_roi", -1.57);
        angle_max_roi_ = declare_parameter<double>("angle_max_roi", 1.57);
        use_range_roi_ = declare_parameter<bool>("use_range_roi", false);
        range_min_roi_ = declare_parameter<double>("range_min_roi", 0.20);
        range_max_roi_ = declare_parameter<double>("range_max_roi", 8.00);

        initial_expected_angle_ = declare_parameter<double>("initial_expected_angle", 0.0);
        initial_expected_range_ = declare_parameter<double>("initial_expected_range", 2.0);
        max_association_distance_ = declare_parameter<double>("max_association_distance", 1.5);
        publish_debug_segment_ = declare_parameter<bool>("publish_debug_segment", true);
    }

    void setupIO()
    {
        rclcpp::QoS qos(rclcpp::KeepLast(1));
        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            scan_topic_, rclcpp::SensorDataQoS(),
            std::bind(&DynamicObstaclePredictor::scanCallback, this, std::placeholders::_1));

        detected_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/detected_obstacle_pose", qos);
        predicted_path_pub_ = create_publisher<nav_msgs::msg::Path>("/predicted_obstacle_path", qos);
        predicted_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud>("/predicted_obstacle_cloud", qos);
        predicted_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("/predicted_obstacle_markers", qos);
        debug_segment_pub_ = create_publisher<visualization_msgs::msg::Marker>("/predicted_obstacle_debug_segment", qos);
        velocity_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("/predicted_obstacle_velocity_marker", qos);
    }

    // ---- track management -------------------------------------------

    void resetTrack()
    {
        has_track_ = false;
        last_track_stamp_ = zeroTime();
        tracked_position_ = {0.0, 0.0};
        vx_ = 0.0;
        vy_ = 0.0;
        has_held_velocity_ = false;
        held_velocity_stamp_ = zeroTime();
        held_vx_ = 0.0;
        held_vy_ = 0.0;
        dynamic_streak_start_ = zeroTime();
    }

    void initializeTrack(const Point2D& p, const rclcpp::Time& stamp)
    {
        tracked_position_ = p;
        vx_ = 0.0;
        vy_ = 0.0;
        last_track_stamp_ = stamp;
        has_track_ = true;
        dynamic_streak_start_ = zeroTime();
    }

    void updateTrack(const Point2D& measurement, double dt)
    {
        Point2D predicted;
        predicted.x = tracked_position_.x + vx_ * dt;
        predicted.y = tracked_position_.y + vy_ * dt;

        const double rx = measurement.x - predicted.x;
        const double ry = measurement.y - predicted.y;

        tracked_position_.x = predicted.x + alpha_gain_ * rx;
        tracked_position_.y = predicted.y + alpha_gain_ * ry;

        if (dt > 1e-4)
        {
            vx_ += (beta_gain_ / dt) * rx;
            vy_ += (beta_gain_ / dt) * ry;
        }

        vx_ = clamp(vx_, -max_speed_, max_speed_);
        vy_ = clamp(vy_, -max_speed_, max_speed_);
    }

    // ---- velocity helpers -------------------------------------------

    double speedNorm(double vx, double vy) const
    {
        return std::sqrt(vx * vx + vy * vy);
    }

    void refreshHeldVelocity(const rclcpp::Time& stamp)
    {
        if (speedNorm(vx_, vy_) < prediction_hold_speed_epsilon_)
            return;
        held_vx_ = vx_;
        held_vy_ = vy_;
        held_velocity_stamp_ = stamp;
        has_held_velocity_ = true;
    }

    void selectPredictionVelocity(const rclcpp::Time& stamp, double& pred_vx, double& pred_vy) const
    {
        pred_vx = vx_;
        pred_vy = vy_;

        if (speedNorm(pred_vx, pred_vy) >= prediction_hold_speed_epsilon_)
            return;
        if (!has_held_velocity_ || isZero(held_velocity_stamp_))
            return;
        if (std::fabs((stamp - held_velocity_stamp_).seconds()) > prediction_hold_time_)
            return;

        pred_vx = held_vx_;
        pred_vy = held_vy_;
    }

    Point2D predictTrackPosition(const rclcpp::Time& stamp) const
    {
        Point2D p = tracked_position_;
        if (!has_track_) return p;
        const double dt = std::max(0.0, (stamp - last_track_stamp_).seconds());
        p.x += vx_ * dt;
        p.y += vy_ * dt;
        return p;
    }

    // ---- small utilities -------------------------------------------

    double clamp(double v, double lo, double hi) const
    {
        return std::max(lo, std::min(hi, v));
    }

    bool validRange(float r) const
    {
        return std::isfinite(r) && r >= scan_range_min_use_ && r <= scan_range_max_use_;
    }

    bool insideAngleROI(double angle) const
    {
        return !use_angle_roi_ || (angle >= angle_min_roi_ && angle <= angle_max_roi_);
    }

    bool insideRangeROI(double range) const
    {
        return !use_range_roi_ || (range >= range_min_roi_ && range <= range_max_roi_);
    }

    double distLaser(const ScanPoint& a, const ScanPoint& b) const
    {
        return std::hypot(a.x_laser - b.x_laser, a.y_laser - b.y_laser);
    }

    double distWorld(const Point2D& a, const Point2D& b) const
    {
        return std::hypot(a.x - b.x, a.y - b.y);
    }

    // ---- scan processing -------------------------------------------

    std::vector<ScanPoint> extractValidPoints(const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan_msg)
    {
        std::vector<ScanPoint> pts;
        pts.reserve(scan_msg->ranges.size());
        double angle = scan_msg->angle_min;

        for (std::size_t i = 0; i < scan_msg->ranges.size(); ++i, angle += scan_msg->angle_increment)
        {
            const float r = scan_msg->ranges[i];
            if (!validRange(r) || !insideAngleROI(angle) || !insideRangeROI(r))
                continue;

            ScanPoint p;
            p.range = static_cast<double>(r);
            p.angle = angle;
            p.x_laser = p.range * std::cos(angle);
            p.y_laser = p.range * std::sin(angle);
            p.index = static_cast<int>(i);
            pts.push_back(p);
        }
        return pts;
    }

    void finalizeSegment(const std::vector<ScanPoint>& raw, std::vector<Segment>& out)
    {
        if (raw.size() < static_cast<std::size_t>(segment_min_points_)) return;
        if (raw.size() > static_cast<std::size_t>(segment_max_points_)) return;

        Segment s;
        s.points = raw;

        double sum_r = 0.0, sum_a = 0.0;
        for (const auto& p : raw) { sum_r += p.range; sum_a += p.angle; }
        s.mean_range = sum_r / raw.size();
        s.mean_angle = sum_a / raw.size();
        s.width = distLaser(raw.front(), raw.back());

        if (s.width < segment_min_width_ || s.width > segment_max_width_) return;
        out.push_back(s);
    }

    std::vector<Segment> buildSegments(const std::vector<ScanPoint>& pts)
    {
        std::vector<Segment> segments;
        if (pts.empty()) return segments;

        std::vector<ScanPoint> current;
        current.push_back(pts.front());

        for (std::size_t i = 1; i < pts.size(); ++i)
        {
            if (distLaser(pts[i - 1], pts[i]) <= segment_jump_distance_)
            {
                current.push_back(pts[i]);
            }
            else
            {
                finalizeSegment(current, segments);
                current.clear();
                current.push_back(pts[i]);
            }
        }
        finalizeSegment(current, segments);
        return segments;
    }

    // ---- segment geometry ------------------------------------------

    Point2D segmentMidpointLaser(const Segment& s) const
    {
        const ScanPoint& a = s.points.front();
        const ScanPoint& b = s.points.back();
        return {0.5 * (a.x_laser + b.x_laser), 0.5 * (a.y_laser + b.y_laser)};
    }

    Point2D shiftPoint(const Point2D& origin, const Point2D& dir, double distance) const
    {
        return {origin.x + dir.x * distance, origin.y + dir.y * distance};
    }

    bool estimateSegmentCenterLaser(const Segment& s,
                                    const std::string& laser_frame,
                                    const rclcpp::Time& stamp,
                                    Point2D& center_laser)
    {
        const Point2D midpoint = segmentMidpointLaser(s);
        const ScanPoint& first = s.points.front();
        const ScanPoint& last  = s.points.back();

        Point2D tangent;
        tangent.x = last.x_laser - first.x_laser;
        tangent.y = last.y_laser - first.y_laser;

        const double tangent_norm = std::sqrt(tangent.x * tangent.x + tangent.y * tangent.y);
        if (tangent_norm < 1e-6)
        {
            center_laser = midpoint;
            return true;
        }

        tangent.x /= tangent_norm;
        tangent.y /= tangent_norm;

        Point2D normal_a{-tangent.y,  tangent.x};
        Point2D normal_b{ tangent.y, -tangent.x};

        const Point2D candidate_a = shiftPoint(midpoint, normal_a, object_half_extent_);
        const Point2D candidate_b = shiftPoint(midpoint, normal_b, object_half_extent_);

        if (!has_track_)
        {
            const double range_a = std::hypot(candidate_a.x, candidate_a.y);
            const double range_b = std::hypot(candidate_b.x, candidate_b.y);
            center_laser = (range_a >= range_b) ? candidate_a : candidate_b;
            return true;
        }

        Point2D cand_a_world, cand_b_world;
        if (!laserPointToWorld(candidate_a, laser_frame, stamp, cand_a_world) ||
            !laserPointToWorld(candidate_b, laser_frame, stamp, cand_b_world))
        {
            center_laser = midpoint;
            return true;
        }

        center_laser = (distWorld(cand_a_world, tracked_position_) <=
                         distWorld(cand_b_world, tracked_position_))
                            ? candidate_a : candidate_b;
        return true;
    }

    // ---- TF helper --------------------------------------------------

    bool laserPointToWorld(const Point2D& p_laser,
                           const std::string& laser_frame,
                           const rclcpp::Time& stamp,
                           Point2D& p_world)
    {
        geometry_msgs::msg::PointStamped in_pt, out_pt;
        in_pt.header.stamp = stamp;
        in_pt.header.frame_id = laser_frame;
        in_pt.point.x = p_laser.x;
        in_pt.point.y = p_laser.y;

        try
        {
            tf_buffer_->transform(in_pt, out_pt, output_frame_, tf2::durationFromSec(0.1));
        }
        catch (const tf2::TransformException& ex)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Point transform failed: %s", ex.what());
            return false;
        }

        p_world.x = out_pt.point.x;
        p_world.y = out_pt.point.y;
        return true;
    }

    // ---- segment selection ------------------------------------------

    bool selectSegment(const std::vector<Segment>& segments,
                       const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan_msg,
                       Segment& selected,
                       Point2D& selected_world)
    {
        if (segments.empty()) return false;

        const rclcpp::Time stamp(scan_msg->header.stamp);
        const Point2D assoc_ref = has_track_
            ? predictTrackPosition(stamp)
            : Point2D{0.0, 0.0};

        double best_score = std::numeric_limits<double>::infinity();
        int best_idx = -1;
        Point2D best_world{};

        for (std::size_t i = 0; i < segments.size(); ++i)
        {
            std::vector<Point2D> candidates_laser;
            Point2D est_center;
            if (estimateSegmentCenterLaser(segments[i], scan_msg->header.frame_id,
                                           stamp, est_center))
                candidates_laser.push_back(est_center);
            candidates_laser.push_back(segmentMidpointLaser(segments[i]));

            double seg_best = std::numeric_limits<double>::infinity();
            Point2D seg_best_world{};

            for (const Point2D& cl : candidates_laser)
            {
                Point2D cw;
                if (!laserPointToWorld(cl, scan_msg->header.frame_id, stamp, cw))
                    continue;

                double score = 0.0;
                if (has_track_)
                {
                    const double d = distWorld(cw, assoc_ref);
                    if (d > max_association_distance_ + object_half_extent_)
                        continue;
                    score = d;
                }
                else
                {
                    const double da = segments[i].mean_angle - initial_expected_angle_;
                    const double dr = segments[i].mean_range - initial_expected_range_;
                    score = std::sqrt(da * da + dr * dr);
                }

                if (score < seg_best)
                {
                    seg_best = score;
                    seg_best_world = cw;
                }
            }

            if (seg_best < best_score)
            {
                best_score = seg_best;
                best_idx = static_cast<int>(i);
                best_world = seg_best_world;
            }
        }

        if (best_idx < 0) return false;
        selected = segments[best_idx];
        selected_world = best_world;
        return true;
    }

    // ---- publishing -------------------------------------------------

    void publishDetectedPose(const Point2D& p, const rclcpp::Time& stamp)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = stamp;
        pose.header.frame_id = output_frame_;
        pose.pose.orientation.w = 1.0;
        pose.pose.position.x = p.x;
        pose.pose.position.y = p.y;
        pose.pose.position.z = z_height_;
        detected_pose_pub_->publish(pose);
    }

    void addDisc(sensor_msgs::msg::PointCloud& cloud, double cx, double cy, double cz)
    {
        for (double ox = -corridor_radius_; ox <= corridor_radius_; ox += point_spacing_)
            for (double oy = -corridor_radius_; oy <= corridor_radius_; oy += point_spacing_)
                if (ox * ox + oy * oy <= corridor_radius_ * corridor_radius_)
                {
                    geometry_msgs::msg::Point32 p;
                    p.x = cx + ox;
                    p.y = cy + oy;
                    p.z = cz;
                    cloud.points.push_back(p);
                }
    }

    void addDenseSegment(sensor_msgs::msg::PointCloud& cloud,
                         double x0, double y0, double z0,
                         double x1, double y1, double z1)
    {
        const double dx = x1 - x0, dy = y1 - y0, dz = z1 - z0;
        const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
        const int steps = std::max(1, static_cast<int>(std::ceil(dist / point_spacing_)));

        for (int i = 0; i <= steps; ++i)
        {
            const double r = static_cast<double>(i) / steps;
            addDisc(cloud, x0 + r * dx, y0 + r * dy, z0 + r * dz);
        }
    }

    void publishPrediction(const rclcpp::Time& stamp, double horizon)
    {
        publishPredictionFrom(tracked_position_, stamp, horizon);
    }

    void publishPredictionFrom(const Point2D& origin, const rclcpp::Time& stamp, double horizon)
    {
        if (!has_track_) return;

        double pred_vx = 0.0, pred_vy = 0.0;
        selectPredictionVelocity(stamp, pred_vx, pred_vy);

        if (speedNorm(pred_vx, pred_vy) < min_dynamic_speed_)
        {
            dynamic_streak_start_ = zeroTime();
            clearPublishedPrediction(stamp);
            return;
        }
        if (isZero(dynamic_streak_start_))
            dynamic_streak_start_ = stamp;
        if ((stamp - dynamic_streak_start_).seconds() < min_dynamic_persist_)
        {
            clearPublishedPrediction(stamp);
            return;
        }

        nav_msgs::msg::Path path;
        path.header.stamp = stamp;
        path.header.frame_id = output_frame_;

        visualization_msgs::msg::Marker marker;
        marker.header = path.header;
        marker.ns = "predicted_obstacle";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.05;
        marker.color.r = 1.0;
        marker.color.a = 1.0;

        sensor_msgs::msg::PointCloud cloud;
        cloud.header = path.header;

        double last_x = origin.x, last_y = origin.y, last_z = z_height_;

        for (double t = 0.0; t <= horizon + 1e-6; t += prediction_dt_)
        {
            const double px = origin.x + pred_vx * t;
            const double py = origin.y + pred_vy * t;
            const double pz = z_height_;

            geometry_msgs::msg::PoseStamped pose;
            pose.header = path.header;
            pose.pose.orientation.w = 1.0;
            pose.pose.position.x = px;
            pose.pose.position.y = py;
            pose.pose.position.z = pz;
            path.poses.push_back(pose);

            geometry_msgs::msg::Point pt;
            pt.x = px; pt.y = py; pt.z = pz;
            marker.points.push_back(pt);

            addDenseSegment(cloud, last_x, last_y, last_z, px, py, pz);
            last_x = px; last_y = py; last_z = pz;
        }

        predicted_path_pub_->publish(path);
        predicted_marker_pub_->publish(marker);
        predicted_cloud_pub_->publish(cloud);
        publishVelocityMarker(origin, stamp);
    }

    void publishVelocityMarker(const Point2D& origin, const rclcpp::Time& stamp)
    {
        if (!has_track_) return;

        double marker_vx = 0.0, marker_vy = 0.0;
        selectPredictionVelocity(stamp, marker_vx, marker_vy);

        visualization_msgs::msg::Marker marker;
        marker.header.stamp = stamp;
        marker.header.frame_id = output_frame_;
        marker.ns = "predicted_obstacle_velocity";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.05;
        marker.scale.y = 0.10;
        marker.scale.z = 0.10;
        marker.color.b = 1.0;
        marker.color.g = 0.4;
        marker.color.a = 1.0;

        geometry_msgs::msg::Point p0, p1;
        p0.x = origin.x; p0.y = origin.y; p0.z = z_height_;
        p1.x = origin.x + 0.5 * marker_vx;
        p1.y = origin.y + 0.5 * marker_vy;
        p1.z = z_height_;
        marker.points.push_back(p0);
        marker.points.push_back(p1);

        velocity_marker_pub_->publish(marker);
    }

    void publishDebugSegment(const Segment& seg,
                             const std::string& laser_frame,
                             const rclcpp::Time& stamp)
    {
        if (!publish_debug_segment_) return;

        visualization_msgs::msg::Marker marker;
        marker.header.stamp = stamp;
        marker.header.frame_id = output_frame_;
        marker.ns = "predicted_obstacle_debug_segment";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::POINTS;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.06;
        marker.scale.y = 0.06;
        marker.color.g = 1.0;
        marker.color.a = 1.0;

        for (const auto& sp : seg.points)
        {
            Point2D p_world;
            if (!laserPointToWorld({sp.x_laser, sp.y_laser}, laser_frame, stamp, p_world))
                continue;
            geometry_msgs::msg::Point p;
            p.x = p_world.x; p.y = p_world.y; p.z = z_height_;
            marker.points.push_back(p);
        }

        debug_segment_pub_->publish(marker);
    }

    template <typename PubT>
    void deleteMarker(const PubT& pub, const std::string& ns,
                      const std::string& frame_id, const rclcpp::Time& stamp)
    {
        visualization_msgs::msg::Marker m;
        m.header.stamp = stamp;
        m.header.frame_id = frame_id;
        m.ns = ns;
        m.id = 0;
        m.action = visualization_msgs::msg::Marker::DELETE;
        pub->publish(m);
    }

    void clearPublishedPrediction(const rclcpp::Time& stamp)
    {
        nav_msgs::msg::Path empty_path;
        empty_path.header.stamp = stamp;
        empty_path.header.frame_id = output_frame_;
        predicted_path_pub_->publish(empty_path);

        sensor_msgs::msg::PointCloud empty_cloud;
        empty_cloud.header = empty_path.header;
        predicted_cloud_pub_->publish(empty_cloud);

        deleteMarker(predicted_marker_pub_, "predicted_obstacle", output_frame_, stamp);
        deleteMarker(velocity_marker_pub_, "predicted_obstacle_velocity", output_frame_, stamp);
        deleteMarker(debug_segment_pub_, "predicted_obstacle_debug_segment", output_frame_, stamp);
    }

    // ---- missing detection handler ----------------------------------

    void handleMissingDetection(const rclcpp::Time& stamp, const std::string& reason)
    {
        if (!has_track_)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "%s", reason.c_str());
            clearPublishedPrediction(stamp);
            return;
        }

        const double dt = (stamp - last_track_stamp_).seconds();
        const double allowed = std::max(track_timeout_, lost_track_prediction_time_);

        if (dt < 0.0 || dt > allowed)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "%s, track expired", reason.c_str());
            resetTrack();
            clearPublishedPrediction(stamp);
            return;
        }

        Point2D predicted_origin;
        predicted_origin.x = tracked_position_.x + vx_ * dt;
        predicted_origin.y = tracked_position_.y + vy_ * dt;

        const double horizon = std::max(prediction_dt_, prediction_time_);
        if (horizon <= 1e-4)
        {
            resetTrack();
            clearPublishedPrediction(stamp);
            return;
        }

        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "%s, using predicted continuation", reason.c_str());
        publishPredictionFrom(predicted_origin, stamp, horizon);
    }

    // ---- main callback ---------------------------------------------

    void scanCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
    {
        const rclcpp::Time stamp(scan_msg->header.stamp);
        const auto valid_pts = extractValidPoints(scan_msg);
        if (valid_pts.empty())
        {
            handleMissingDetection(stamp, "No valid scan points after ROI filtering");
            return;
        }

        const auto segments = buildSegments(valid_pts);
        if (segments.empty())
        {
            handleMissingDetection(stamp, "No valid scan segments");
            return;
        }

        Segment selected;
        Point2D measurement_world;
        if (!selectSegment(segments, scan_msg, selected, measurement_world))
        {
            handleMissingDetection(stamp, "No segment selected");
            return;
        }

        publishDebugSegment(selected, scan_msg->header.frame_id, stamp);
        publishDetectedPose(measurement_world, stamp);

        if (!has_track_)
        {
            initializeTrack(measurement_world, stamp);
            publishPrediction(stamp, prediction_time_);
            return;
        }

        const double dt = (stamp - last_track_stamp_).seconds();
        if (dt < 0.0 || dt > track_timeout_)
        {
            initializeTrack(measurement_world, stamp);
            publishPrediction(stamp, prediction_time_);
            return;
        }

        if (dt < 1e-4)
        {
            publishPrediction(stamp, prediction_time_);
            return;
        }

        updateTrack(measurement_world, dt);
        last_track_stamp_ = stamp;
        refreshHeldVelocity(stamp);
        publishPrediction(stamp, prediction_time_);
    }

    // ---- members ----------------------------------------------------

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr detected_pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr predicted_path_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr predicted_cloud_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr predicted_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr debug_segment_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr velocity_marker_pub_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::string scan_topic_;
    std::string output_frame_;

    double prediction_time_;
    double prediction_dt_;
    double alpha_gain_;
    double beta_gain_;
    double max_speed_;
    double track_timeout_;
    double lost_track_prediction_time_;
    double prediction_hold_time_;
    double prediction_hold_speed_epsilon_;
    double min_dynamic_speed_;
    double min_dynamic_persist_;
    rclcpp::Time dynamic_streak_start_{0, 0, RCL_ROS_TIME};

    double point_spacing_;
    double corridor_radius_;
    double object_half_extent_;
    double z_height_;

    double scan_range_min_use_;
    double scan_range_max_use_;
    double segment_jump_distance_;
    int segment_min_points_;
    int segment_max_points_;
    double segment_min_width_;
    double segment_max_width_;

    bool use_angle_roi_;
    double angle_min_roi_;
    double angle_max_roi_;
    bool use_range_roi_;
    double range_min_roi_;
    double range_max_roi_;

    double initial_expected_angle_;
    double initial_expected_range_;
    double max_association_distance_;
    bool publish_debug_segment_;

    bool has_track_{false};
    rclcpp::Time last_track_stamp_{0, 0, RCL_ROS_TIME};
    Point2D tracked_position_{0.0, 0.0};
    double vx_{0.0};
    double vy_{0.0};
    bool has_held_velocity_{false};
    rclcpp::Time held_velocity_stamp_{0, 0, RCL_ROS_TIME};
    double held_vx_{0.0};
    double held_vy_{0.0};
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DynamicObstaclePredictor>());
    rclcpp::shutdown();
    return 0;
}
