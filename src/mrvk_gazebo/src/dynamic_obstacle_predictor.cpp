// ROS2 (rclcpp) port of the dynamic obstacle detector/predictor.
// Detects and tracks multiple simultaneous dynamic obstacles from the
// ground-segmented obstacle point cloud (/ground_segmentation/obstacle_points)
// using per-object alpha-beta filters (position+velocity) and
// constant-velocity predictions. Publishes /detected_obstacle_pose and
// /predicted_obstacle_path for the single closest ("primary") track only,
// for backward compatibility with robot_motion_predictor (not yet updated
// to consume multiple tracks); /predicted_obstacle_markers,
// /predicted_obstacle_velocity_marker and /predicted_obstacle_debug_segment
// are MarkerArrays covering every currently tracked object.

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <vector>
#include <string>
#include <cmath>
#include <limits>
#include <algorithm>
#include <memory>
#include <unordered_map>
#include <cstdint>

// Simple union-find used to merge points into Euclidean-connected clusters
// (see buildSegments) -- independent of any angular ordering.
struct DisjointSet
{
    std::vector<int> parent;
    explicit DisjointSet(int n) : parent(n) { for (int i = 0; i < n; ++i) parent[i] = i; }
    int find(int x) { while (parent[x] != x) { parent[x] = parent[parent[x]]; x = parent[x]; } return x; }
    void unite(int a, int b) { a = find(a); b = find(b); if (a != b) parent[a] = b; }
};

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

struct Track
{
    int id{0};
    Point2D position{0.0, 0.0};
    double vx{0.0};
    double vy{0.0};
    rclcpp::Time last_stamp{0, 0, RCL_ROS_TIME};
    bool has_held_velocity{false};
    rclcpp::Time held_velocity_stamp{0, 0, RCL_ROS_TIME};
    double held_vx{0.0};
    double held_vy{0.0};
    rclcpp::Time dynamic_streak_start{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_above_threshold_stamp{0, 0, RCL_ROS_TIME};
    double last_range{0.0};
    bool matched_this_cycle{false};
    bool pending_dead_reckon{false};
    Point2D dead_reckon_origin{0.0, 0.0};
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
        RCLCPP_INFO(get_logger(),
                    "dynamic_obstacle_predictor started (obstacle_cloud=%s frame=%s, max_tracks=%d)",
                    obstacle_cloud_topic_.c_str(), output_frame_.c_str(), max_tracks_);
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
        obstacle_cloud_topic_ = declare_parameter<std::string>(
            "obstacle_cloud_topic", "/ground_segmentation/obstacle_points");
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
        dynamic_dip_tolerance_ = declare_parameter<double>("dynamic_dip_tolerance", 0.3);

        point_spacing_ = declare_parameter<double>("point_spacing", 0.05);
        corridor_radius_ = declare_parameter<double>("corridor_radius", 0.18);
        object_half_extent_ = declare_parameter<double>("object_half_extent", 0.18);
        center_side_hysteresis_ = declare_parameter<double>("center_side_hysteresis", 0.5);
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
        max_tracks_ = declare_parameter<int>("max_tracks", 5);
    }

    void setupIO()
    {
        rclcpp::QoS qos(rclcpp::KeepLast(1));
        obstacle_cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            obstacle_cloud_topic_, rclcpp::SensorDataQoS(),
            std::bind(&DynamicObstaclePredictor::cloudCallback, this, std::placeholders::_1));

        detected_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/detected_obstacle_pose", qos);
        predicted_path_pub_ = create_publisher<nav_msgs::msg::Path>("/predicted_obstacle_path", qos);
        predicted_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud>("/predicted_obstacle_cloud", qos);
        predicted_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/predicted_obstacle_markers", qos);
        debug_segment_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/predicted_obstacle_debug_segment", qos);
        velocity_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/predicted_obstacle_velocity_marker", qos);
        raw_segments_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/predicted_obstacle_raw_segments", qos);
    }

    // ---- track management -------------------------------------------

    Track& spawnTrack(const Point2D& p, const rclcpp::Time& stamp)
    {
        Track t;
        t.id = next_track_id_++;
        t.position = p;
        t.vx = 0.0;
        t.vy = 0.0;
        t.last_stamp = stamp;
        t.dynamic_streak_start = zeroTime();
        t.last_above_threshold_stamp = zeroTime();
        tracks_.push_back(t);
        return tracks_.back();
    }

    void reinitTrackInPlace(Track& t, const Point2D& p, const rclcpp::Time& stamp)
    {
        t.position = p;
        t.vx = 0.0;
        t.vy = 0.0;
        t.last_stamp = stamp;
        t.has_held_velocity = false;
        t.held_velocity_stamp = zeroTime();
        t.held_vx = 0.0;
        t.held_vy = 0.0;
        t.dynamic_streak_start = zeroTime();
        t.last_above_threshold_stamp = zeroTime();
    }

    void updateTrack(Track& t, const Point2D& measurement, double dt)
    {
        Point2D predicted;
        predicted.x = t.position.x + t.vx * dt;
        predicted.y = t.position.y + t.vy * dt;

        const double rx = measurement.x - predicted.x;
        const double ry = measurement.y - predicted.y;

        t.position.x = predicted.x + alpha_gain_ * rx;
        t.position.y = predicted.y + alpha_gain_ * ry;

        if (dt > 1e-4)
        {
            t.vx += (beta_gain_ / dt) * rx;
            t.vy += (beta_gain_ / dt) * ry;
        }

        t.vx = clamp(t.vx, -max_speed_, max_speed_);
        t.vy = clamp(t.vy, -max_speed_, max_speed_);
    }

    // ---- velocity helpers -------------------------------------------

    double speedNorm(double vx, double vy) const
    {
        return std::sqrt(vx * vx + vy * vy);
    }

    void refreshHeldVelocity(Track& t, const rclcpp::Time& stamp)
    {
        if (speedNorm(t.vx, t.vy) < prediction_hold_speed_epsilon_)
            return;
        t.held_vx = t.vx;
        t.held_vy = t.vy;
        t.held_velocity_stamp = stamp;
        t.has_held_velocity = true;
    }

    void selectPredictionVelocity(const Track& t, const rclcpp::Time& stamp, double& pred_vx, double& pred_vy) const
    {
        pred_vx = t.vx;
        pred_vy = t.vy;

        if (speedNorm(pred_vx, pred_vy) >= prediction_hold_speed_epsilon_)
            return;
        if (!t.has_held_velocity || isZero(t.held_velocity_stamp))
            return;
        if (std::fabs((stamp - t.held_velocity_stamp).seconds()) > prediction_hold_time_)
            return;

        pred_vx = t.held_vx;
        pred_vy = t.held_vy;
    }

    Point2D predictTrackPosition(const Track& t, const rclcpp::Time& stamp) const
    {
        Point2D p = t.position;
        const double dt = std::max(0.0, (stamp - t.last_stamp).seconds());
        p.x += t.vx * dt;
        p.y += t.vy * dt;
        return p;
    }

    // ---- small utilities -------------------------------------------

    double clamp(double v, double lo, double hi) const
    {
        return std::max(lo, std::min(hi, v));
    }

    bool validRange(double r) const
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

    // ---- cloud processing -------------------------------------------

    // Ground already removed upstream by ground_segmentation_ros2_node, so
    // every point here is a candidate obstacle. Flatten to 2D (x,y) in the
    // cloud's own frame and derive range/angle around that frame's origin,
    // then sort by angle so buildSegments' sequential jump-distance
    // clustering (written for angularly-ordered LaserScan rays) still holds
    // on an otherwise unordered point cloud.
    std::vector<ScanPoint> extractValidPointsFromCloud(
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg)
    {
        std::vector<ScanPoint> pts;
        pts.reserve(static_cast<std::size_t>(cloud_msg->width) * cloud_msg->height);

        sensor_msgs::PointCloud2ConstIterator<float> it_x(*cloud_msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> it_y(*cloud_msg, "y");

        int idx = 0;
        for (; it_x != it_x.end(); ++it_x, ++it_y, ++idx)
        {
            const double x = static_cast<double>(*it_x);
            const double y = static_cast<double>(*it_y);
            if (!std::isfinite(x) || !std::isfinite(y)) continue;

            const double range = std::hypot(x, y);
            const double angle = std::atan2(y, x);
            if (!validRange(range) || !insideAngleROI(angle) || !insideRangeROI(range))
                continue;

            ScanPoint p;
            p.range = range;
            p.angle = angle;
            p.x_laser = x;
            p.y_laser = y;
            p.index = idx;
            pts.push_back(p);
        }

        std::sort(pts.begin(), pts.end(),
                  [](const ScanPoint& a, const ScanPoint& b) { return a.angle < b.angle; });
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

    // Euclidean (radius-connected) clustering over the raw 2D point set,
    // using a uniform spatial hash grid (cell size = segment_jump_distance_)
    // for O(N) average neighbor queries and union-find to merge points
    // within segment_jump_distance_ of each other into one cluster. Replaces
    // the old angle-sorted 1D jump-distance walk, which assumed a
    // sequentially-swept LaserScan; on an unordered pointcloud-derived point
    // set that assumption let spatially close points end up in different
    // segments (or vice versa) purely because of where they fell in the
    // angular sort, which was the dominant source of the segment
    // composition/position instability seen once the predictor moved to
    // consuming /ground_segmentation/obstacle_points directly.
    std::vector<Segment> buildSegments(const std::vector<ScanPoint>& pts)
    {
        std::vector<Segment> segments;
        if (pts.empty()) return segments;

        const double cell = std::max(segment_jump_distance_, 1e-3);
        auto cellIndex = [cell](double v) {
            return static_cast<int32_t>(std::floor(v / cell));
        };
        auto cellKey = [](int32_t cx, int32_t cy) -> int64_t {
            return (static_cast<int64_t>(cx) << 32) | static_cast<uint32_t>(cy);
        };

        std::unordered_map<int64_t, std::vector<int>> grid;
        grid.reserve(pts.size());
        for (std::size_t i = 0; i < pts.size(); ++i)
        {
            const int64_t key = cellKey(cellIndex(pts[i].x_laser), cellIndex(pts[i].y_laser));
            grid[key].push_back(static_cast<int>(i));
        }

        DisjointSet ds(static_cast<int>(pts.size()));
        const double jump2 = segment_jump_distance_ * segment_jump_distance_;

        for (std::size_t i = 0; i < pts.size(); ++i)
        {
            const int32_t cx = cellIndex(pts[i].x_laser);
            const int32_t cy = cellIndex(pts[i].y_laser);

            for (int dx = -1; dx <= 1; ++dx)
            {
                for (int dy = -1; dy <= 1; ++dy)
                {
                    const auto it = grid.find(cellKey(cx + dx, cy + dy));
                    if (it == grid.end()) continue;

                    for (int j : it->second)
                    {
                        if (j <= static_cast<int>(i)) continue;
                        const double ddx = pts[i].x_laser - pts[j].x_laser;
                        const double ddy = pts[i].y_laser - pts[j].y_laser;
                        if (ddx * ddx + ddy * ddy <= jump2)
                            ds.unite(static_cast<int>(i), j);
                    }
                }
            }
        }

        std::unordered_map<int, std::vector<ScanPoint>> clusters;
        for (std::size_t i = 0; i < pts.size(); ++i)
            clusters[ds.find(static_cast<int>(i))].push_back(pts[i]);

        for (auto& entry : clusters)
        {
            std::vector<ScanPoint>& cluster_pts = entry.second;
            // Keep points angle-sorted within each cluster: downstream code
            // (estimateSegmentCenterLaser) uses the front/back points to
            // derive a tangent/normal for near-side/far-side disambiguation.
            std::sort(cluster_pts.begin(), cluster_pts.end(),
                      [](const ScanPoint& a, const ScanPoint& b) { return a.angle < b.angle; });
            finalizeSegment(cluster_pts, segments);
        }

        return segments;
    }

    // ---- segment geometry ------------------------------------------

    // Centroid over every point in the segment, not just the two angular
    // extremes: with the dense pointcloud-derived segments (dozens to
    // hundreds of points) the extreme points are the noisiest ones, and a
    // first/last average made the estimated center jump between frames
    // enough to break track continuity or masquerade as velocity.
    Point2D segmentMidpointLaser(const Segment& s) const
    {
        double sum_x = 0.0, sum_y = 0.0;
        for (const auto& p : s.points)
        {
            sum_x += p.x_laser;
            sum_y += p.y_laser;
        }
        const double n = static_cast<double>(s.points.size());
        return {sum_x / n, sum_y / n};
    }

    Point2D shiftPoint(const Point2D& origin, const Point2D& dir, double distance) const
    {
        return {origin.x + dir.x * distance, origin.y + dir.y * distance};
    }

    // ref == nullptr: no candidate track to disambiguate against, pick the
    // candidate farther from the sensor. ref != nullptr: pick whichever
    // candidate is closer (in world frame) to *ref (a specific track's
    // predicted position).
    bool estimateSegmentCenterLaser(const Segment& s,
                                    const std::string& laser_frame,
                                    const rclcpp::Time& stamp,
                                    Point2D& center_laser,
                                    const Point2D* ref = nullptr)
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

        if (!ref)
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

        // Two disambiguation signals: geometric (farther from the sensor is
        // almost always the true far side of the object, and is stable
        // frame to frame since it only depends on this segment) and
        // kinematic (closer to the track's predicted position, which is
        // what actually matters when the object is genuinely moving). They
        // usually agree. When they don't, trust kinematics only if it wins
        // by a clear margin -- otherwise a near-tie in predicted-position
        // distance (typical while viewing a static object near a symmetric
        // angle) flips the choice every few frames, and the resulting
        // ~2*object_half_extent_ position jump gets read by the alpha-beta
        // filter as spurious velocity on an object that never moved.
        const double range_a = std::hypot(candidate_a.x, candidate_a.y);
        const double range_b = std::hypot(candidate_b.x, candidate_b.y);
        const bool geometric_prefers_a = range_a >= range_b;

        const double da = distWorld(cand_a_world, *ref);
        const double db = distWorld(cand_b_world, *ref);
        const bool kinematic_prefers_a = da <= db;

        bool prefer_a = geometric_prefers_a;
        if (geometric_prefers_a != kinematic_prefers_a)
        {
            const double kinematic_margin = std::fabs(da - db);
            if (kinematic_margin > center_side_hysteresis_ * object_half_extent_)
                prefer_a = kinematic_prefers_a;
        }

        center_laser = prefer_a ? candidate_a : candidate_b;
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

    // ---- segment <-> track assignment --------------------------------

    struct Assignment
    {
        std::vector<int> track_to_segment;   // size == tracks_.size(), -1 if unmatched
        std::vector<Point2D> track_measurement_world;
        std::vector<bool> segment_used;      // size == segments.size()
    };

    Assignment assignSegmentsToTracks(const std::vector<Segment>& segments,
                                      const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg,
                                      const rclcpp::Time& stamp)
    {
        Assignment result;
        result.track_to_segment.assign(tracks_.size(), -1);
        result.track_measurement_world.assign(tracks_.size(), Point2D{0.0, 0.0});
        result.segment_used.assign(segments.size(), false);

        struct Candidate { std::size_t track_i; std::size_t seg_j; double cost; Point2D world; };
        std::vector<Candidate> candidates;

        for (std::size_t i = 0; i < tracks_.size(); ++i)
        {
            const Point2D assoc_ref = predictTrackPosition(tracks_[i], stamp);

            for (std::size_t j = 0; j < segments.size(); ++j)
            {
                std::vector<Point2D> candidates_laser;
                Point2D est_center;
                if (estimateSegmentCenterLaser(segments[j], cloud_msg->header.frame_id,
                                               stamp, est_center, &assoc_ref))
                    candidates_laser.push_back(est_center);
                candidates_laser.push_back(segmentMidpointLaser(segments[j]));

                double best = std::numeric_limits<double>::infinity();
                Point2D best_world{};
                for (const Point2D& cl : candidates_laser)
                {
                    Point2D cw;
                    if (!laserPointToWorld(cl, cloud_msg->header.frame_id, stamp, cw))
                        continue;
                    const double d = distWorld(cw, assoc_ref);
                    if (d > max_association_distance_ + object_half_extent_)
                        continue;
                    if (d < best)
                    {
                        best = d;
                        best_world = cw;
                    }
                }

                if (std::isfinite(best))
                    candidates.push_back({i, j, best, best_world});
            }
        }

        std::sort(candidates.begin(), candidates.end(),
                  [](const Candidate& a, const Candidate& b) { return a.cost < b.cost; });

        std::vector<bool> track_claimed(tracks_.size(), false);
        for (const auto& c : candidates)
        {
            if (track_claimed[c.track_i] || result.segment_used[c.seg_j])
                continue;
            track_claimed[c.track_i] = true;
            result.segment_used[c.seg_j] = true;
            result.track_to_segment[c.track_i] = static_cast<int>(c.seg_j);
            result.track_measurement_world[c.track_i] = c.world;
        }

        return result;
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

    // Result of evaluating one track's prediction for this cycle. Does not
    // publish anything itself; the caller aggregates these across tracks
    // into MarkerArrays and picks the primary track for the singular topics.
    struct TrackPrediction
    {
        bool dynamic{false};
        Point2D origin{0.0, 0.0};
        double vx{0.0};
        double vy{0.0};
        std::vector<geometry_msgs::msg::PoseStamped> poses;
    };

    TrackPrediction buildTrackPrediction(Track& t, const Point2D& origin,
                                         const rclcpp::Time& stamp, double horizon,
                                         const std_msgs::msg::Header& header)
    {
        TrackPrediction result;
        result.origin = origin;

        double pred_vx = 0.0, pred_vy = 0.0;
        selectPredictionVelocity(t, stamp, pred_vx, pred_vy);

        if (speedNorm(pred_vx, pred_vy) < min_dynamic_speed_)
        {
            // A single noisy below-threshold reading shouldn't throw away
            // an otherwise-real motion streak: the raw per-cycle velocity
            // estimate straddles min_dynamic_speed_ for a genuinely moving
            // object often enough that a hard reset-on-any-dip never lets
            // min_dynamic_persist_ accumulate. Only reset once the object
            // has been below threshold for longer than the dip tolerance
            // (i.e. it plausibly stopped, not just one jittery sample).
            if (isZero(t.last_above_threshold_stamp) ||
                (stamp - t.last_above_threshold_stamp).seconds() > dynamic_dip_tolerance_)
            {
                t.dynamic_streak_start = zeroTime();
            }
            return result;
        }
        t.last_above_threshold_stamp = stamp;
        if (isZero(t.dynamic_streak_start))
            t.dynamic_streak_start = stamp;
        if ((stamp - t.dynamic_streak_start).seconds() < min_dynamic_persist_)
            return result;

        result.dynamic = true;
        result.vx = pred_vx;
        result.vy = pred_vy;

        for (double time = 0.0; time <= horizon + 1e-6; time += prediction_dt_)
        {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = header;
            pose.pose.orientation.w = 1.0;
            pose.pose.position.x = origin.x + pred_vx * time;
            pose.pose.position.y = origin.y + pred_vy * time;
            pose.pose.position.z = z_height_;
            result.poses.push_back(pose);
        }
        return result;
    }

    visualization_msgs::msg::Marker lineStripMarkerFor(const TrackPrediction& pred,
                                                       const std_msgs::msg::Header& header,
                                                       int id)
    {
        visualization_msgs::msg::Marker marker;
        marker.header = header;
        marker.ns = "predicted_obstacle";
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.05;
        marker.color.r = 1.0;
        marker.color.a = 1.0;
        for (const auto& pose : pred.poses)
            marker.points.push_back(pose.pose.position);
        return marker;
    }

    visualization_msgs::msg::Marker velocityMarkerFor(const TrackPrediction& pred,
                                                       const rclcpp::Time& stamp,
                                                       int id)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.stamp = stamp;
        marker.header.frame_id = output_frame_;
        marker.ns = "predicted_obstacle_velocity";
        marker.id = id;
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
        p0.x = pred.origin.x; p0.y = pred.origin.y; p0.z = z_height_;
        p1.x = pred.origin.x + 0.5 * pred.vx;
        p1.y = pred.origin.y + 0.5 * pred.vy;
        p1.z = z_height_;
        marker.points.push_back(p0);
        marker.points.push_back(p1);
        return marker;
    }

    visualization_msgs::msg::Marker debugSegmentMarkerFor(const Segment& seg,
                                                           const std::string& laser_frame,
                                                           const rclcpp::Time& stamp,
                                                           int id)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.stamp = stamp;
        marker.header.frame_id = output_frame_;
        marker.ns = "predicted_obstacle_debug_segment";
        marker.id = id;
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
        return marker;
    }

    visualization_msgs::msg::Marker deleteMarker(const std::string& ns, int id,
                                                 const rclcpp::Time& stamp) const
    {
        visualization_msgs::msg::Marker m;
        m.header.stamp = stamp;
        m.header.frame_id = output_frame_;
        m.ns = ns;
        m.id = id;
        m.action = visualization_msgs::msg::Marker::DELETE;
        return m;
    }

    // Every valid cluster straight out of buildSegments, before any track
    // association or dynamic/static classification. Lets you see
    // immediately whether an object is being segmented at all (and how
    // many points / what shape) from the very first frame it enters view,
    // independent of whether a track ever spawns for it or how long it
    // takes to be classified as moving -- unlike
    // /predicted_obstacle_debug_segment, which only ever shows segments
    // that already got matched to a track.
    void publishRawSegments(const std::vector<Segment>& segments,
                            const std::string& cloud_frame,
                            const rclcpp::Time& stamp)
    {
        visualization_msgs::msg::MarkerArray array;
        for (std::size_t j = 0; j < segments.size(); ++j)
        {
            visualization_msgs::msg::Marker marker;
            marker.header.stamp = stamp;
            marker.header.frame_id = output_frame_;
            marker.ns = "raw_segment";
            marker.id = static_cast<int>(j);
            marker.type = visualization_msgs::msg::Marker::POINTS;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.04;
            marker.scale.y = 0.04;
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.a = 0.8;

            for (const auto& sp : segments[j].points)
            {
                Point2D p_world;
                if (!laserPointToWorld({sp.x_laser, sp.y_laser}, cloud_frame, stamp, p_world))
                    continue;
                geometry_msgs::msg::Point p;
                p.x = p_world.x; p.y = p_world.y; p.z = z_height_;
                marker.points.push_back(p);
            }
            array.markers.push_back(marker);
        }

        for (std::size_t j = segments.size(); j < last_raw_segment_count_; ++j)
            array.markers.push_back(deleteMarker("raw_segment", static_cast<int>(j), stamp));

        last_raw_segment_count_ = segments.size();
        raw_segments_pub_->publish(array);
    }

    void clearPrimaryOutputs(const rclcpp::Time& stamp)
    {
        nav_msgs::msg::Path empty_path;
        empty_path.header.stamp = stamp;
        empty_path.header.frame_id = output_frame_;
        predicted_path_pub_->publish(empty_path);

        sensor_msgs::msg::PointCloud empty_cloud;
        empty_cloud.header = empty_path.header;
        predicted_cloud_pub_->publish(empty_cloud);
    }

    // ---- per-track missing-detection / expiry ------------------------

    enum class MissingOutcome { DEAD_RECKONED, EXPIRED };

    MissingOutcome handleTrackMissing(Track& t, const rclcpp::Time& stamp, Point2D& out_origin)
    {
        const double dt = (stamp - t.last_stamp).seconds();
        const double allowed = std::max(track_timeout_, lost_track_prediction_time_);

        if (dt < 0.0 || dt > allowed)
            return MissingOutcome::EXPIRED;

        out_origin.x = t.position.x + t.vx * dt;
        out_origin.y = t.position.y + t.vy * dt;

        const double horizon = std::max(prediction_dt_, prediction_time_);
        if (horizon <= 1e-4)
            return MissingOutcome::EXPIRED;

        return MissingOutcome::DEAD_RECKONED;
    }

    // ---- new-track spawning -------------------------------------------

    void spawnNewTracks(const std::vector<Segment>& segments,
                        const Assignment& assignment,
                        const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg,
                        const rclcpp::Time& stamp,
                        visualization_msgs::msg::MarkerArray& debug_array)
    {
        std::vector<std::size_t> unmatched;
        for (std::size_t j = 0; j < segments.size(); ++j)
            if (!assignment.segment_used[j])
                unmatched.push_back(j);

        const int capacity = max_tracks_ - static_cast<int>(tracks_.size());
        if (capacity <= 0 || unmatched.empty()) return;

        if (static_cast<int>(unmatched.size()) > capacity)
        {
            std::sort(unmatched.begin(), unmatched.end(),
                      [&](std::size_t a, std::size_t b) {
                          const double da = segments[a].mean_angle - initial_expected_angle_;
                          const double dr = segments[a].mean_range - initial_expected_range_;
                          const double db = segments[b].mean_angle - initial_expected_angle_;
                          const double dr2 = segments[b].mean_range - initial_expected_range_;
                          return (da * da + dr * dr) < (db * db + dr2 * dr2);
                      });
            unmatched.resize(static_cast<std::size_t>(capacity));
        }

        for (std::size_t j : unmatched)
        {
            Point2D center_laser;
            estimateSegmentCenterLaser(segments[j], cloud_msg->header.frame_id, stamp, center_laser, nullptr);
            Point2D center_world;
            if (!laserPointToWorld(center_laser, cloud_msg->header.frame_id, stamp, center_world))
                continue;

            Track& t = spawnTrack(center_world, stamp);
            t.last_range = segments[j].mean_range;
            t.matched_this_cycle = true;
            debug_array.markers.push_back(debugSegmentMarkerFor(
                segments[j], cloud_msg->header.frame_id, stamp, t.id));
        }
    }

    // ---- main callback ---------------------------------------------

    void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg)
    {
        const rclcpp::Time stamp(cloud_msg->header.stamp);
        const auto valid_pts = extractValidPointsFromCloud(cloud_msg);
        if (valid_pts.empty())
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                "No valid obstacle cloud points after ROI filtering");
            handleAllTracksMissing(stamp);
            return;
        }

        const auto segments = buildSegments(valid_pts);
        publishRawSegments(segments, cloud_msg->header.frame_id, stamp);

        if (segments.empty())
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "No valid obstacle segments");
            handleAllTracksMissing(stamp);
            return;
        }

        const Assignment assignment = assignSegmentsToTracks(segments, cloud_msg, stamp);

        std_msgs::msg::Header header;
        header.stamp = stamp;
        header.frame_id = output_frame_;

        visualization_msgs::msg::MarkerArray debug_array;
        for (auto& t : tracks_)
        {
            t.matched_this_cycle = false;
            t.pending_dead_reckon = false;
        }

        // 1) Matched tracks: update or (if stale) re-init in place.
        for (std::size_t i = 0; i < tracks_.size(); ++i)
        {
            const int seg_idx = assignment.track_to_segment[i];
            if (seg_idx < 0) continue;

            Track& t = tracks_[i];
            t.matched_this_cycle = true;
            t.last_range = segments[static_cast<std::size_t>(seg_idx)].mean_range;
            const Point2D measurement_world = assignment.track_measurement_world[i];

            debug_array.markers.push_back(debugSegmentMarkerFor(
                segments[static_cast<std::size_t>(seg_idx)], cloud_msg->header.frame_id, stamp, t.id));

            const double dt = (stamp - t.last_stamp).seconds();
            if (dt < 0.0 || dt > track_timeout_)
            {
                reinitTrackInPlace(t, measurement_world, stamp);
            }
            else if (dt < 1e-4)
            {
                // duplicate timestamp: skip the filter update this cycle
            }
            else
            {
                updateTrack(t, measurement_world, dt);
                t.last_stamp = stamp;
                refreshHeldVelocity(t, stamp);
            }
        }

        // 2) Unmatched existing tracks: dead-reckon or expire. Dead-reckon
        //    state is stored on the Track itself (not a parallel array) so
        //    it survives the erase()/spawn() index churn below.
        std::vector<int> expired_ids;
        for (std::size_t i = 0; i < tracks_.size(); ++i)
        {
            if (assignment.track_to_segment[i] >= 0) continue;

            Point2D origin;
            const MissingOutcome outcome = handleTrackMissing(tracks_[i], stamp, origin);
            if (outcome == MissingOutcome::EXPIRED)
            {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                    "Track %d expired", tracks_[i].id);
                expired_ids.push_back(tracks_[i].id);
            }
            else
            {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                    "Track %d missing, using predicted continuation", tracks_[i].id);
                tracks_[i].dead_reckon_origin = origin;
                tracks_[i].pending_dead_reckon = true;
            }
        }

        // Erase expired tracks (reverse order to keep indices valid).
        for (int id : expired_ids)
        {
            tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
                                         [id](const Track& t) { return t.id == id; }),
                          tracks_.end());
        }

        // 3) Spawn new tracks from unmatched segments (respecting max_tracks_).
        spawnNewTracks(segments, assignment, cloud_msg, stamp, debug_array);

        // 4) Build predictions for every remaining track, aggregate into
        //    MarkerArrays, pick the primary (closest last-matched-range)
        //    track for the backward-compatible singular topics.
        visualization_msgs::msg::MarkerArray marker_array;
        visualization_msgs::msg::MarkerArray velocity_array;
        double best_range = std::numeric_limits<double>::infinity();
        bool have_primary = false;
        TrackPrediction primary_pred;
        Point2D primary_measurement{0.0, 0.0};
        bool primary_matched_this_cycle = false;

        for (std::size_t i = 0; i < tracks_.size(); ++i)
        {
            Track& t = tracks_[i];
            const Point2D origin = t.pending_dead_reckon ? t.dead_reckon_origin : t.position;
            const double horizon = t.pending_dead_reckon
                ? std::max(prediction_dt_, prediction_time_)
                : prediction_time_;

            TrackPrediction pred = buildTrackPrediction(t, origin, stamp, horizon, header);

            if (pred.dynamic)
            {
                marker_array.markers.push_back(lineStripMarkerFor(pred, header, t.id));
                velocity_array.markers.push_back(velocityMarkerFor(pred, stamp, t.id));
            }
            else
            {
                marker_array.markers.push_back(deleteMarker("predicted_obstacle", t.id, stamp));
                velocity_array.markers.push_back(deleteMarker("predicted_obstacle_velocity", t.id, stamp));
            }

            if (pred.dynamic && t.last_range < best_range)
            {
                best_range = t.last_range;
                have_primary = true;
                primary_pred = pred;
                primary_measurement = assignment.track_to_segment[i] >= 0
                    ? assignment.track_measurement_world[i] : origin;
                primary_matched_this_cycle = t.matched_this_cycle;
            }
        }

        for (int id : expired_ids)
        {
            marker_array.markers.push_back(deleteMarker("predicted_obstacle", id, stamp));
            velocity_array.markers.push_back(deleteMarker("predicted_obstacle_velocity", id, stamp));
            debug_array.markers.push_back(deleteMarker("predicted_obstacle_debug_segment", id, stamp));
        }

        if (publish_debug_segment_)
            debug_segment_pub_->publish(debug_array);
        predicted_marker_pub_->publish(marker_array);
        velocity_marker_pub_->publish(velocity_array);

        if (have_primary)
        {
            nav_msgs::msg::Path path;
            path.header = header;
            path.poses = primary_pred.poses;
            predicted_path_pub_->publish(path);

            sensor_msgs::msg::PointCloud cloud;
            cloud.header = header;
            double last_x = primary_pred.origin.x, last_y = primary_pred.origin.y, last_z = z_height_;
            for (const auto& pose : primary_pred.poses)
            {
                addDenseSegment(cloud, last_x, last_y, last_z,
                               pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
                last_x = pose.pose.position.x;
                last_y = pose.pose.position.y;
                last_z = pose.pose.position.z;
            }
            predicted_cloud_pub_->publish(cloud);

            if (primary_matched_this_cycle)
                publishDetectedPose(primary_measurement, stamp);
        }
        else
        {
            clearPrimaryOutputs(stamp);
        }
    }

    void handleAllTracksMissing(const rclcpp::Time& stamp)
    {
        std::vector<int> expired_ids;
        for (auto& t : tracks_)
        {
            Point2D origin;
            if (handleTrackMissing(t, stamp, origin) == MissingOutcome::EXPIRED)
                expired_ids.push_back(t.id);
        }
        for (int id : expired_ids)
        {
            tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
                                         [id](const Track& t) { return t.id == id; }),
                          tracks_.end());
        }

        // Always clear markers for every id that expired this cycle, on all
        // three MarkerArray topics with their correct namespaces, regardless
        // of whether any other track survives.
        visualization_msgs::msg::MarkerArray expired_markers;
        visualization_msgs::msg::MarkerArray expired_velocity;
        visualization_msgs::msg::MarkerArray expired_debug;
        for (int id : expired_ids)
        {
            expired_markers.markers.push_back(deleteMarker("predicted_obstacle", id, stamp));
            expired_velocity.markers.push_back(deleteMarker("predicted_obstacle_velocity", id, stamp));
            expired_debug.markers.push_back(deleteMarker("predicted_obstacle_debug_segment", id, stamp));
        }

        if (tracks_.empty())
        {
            clearPrimaryOutputs(stamp);
            predicted_marker_pub_->publish(expired_markers);
            velocity_marker_pub_->publish(expired_velocity);
            if (publish_debug_segment_)
                debug_segment_pub_->publish(expired_debug);
            return;
        }

        // Tracks remain (within timeout) but no scan data at all this cycle:
        // dead-reckon every remaining track the same way scanCallback would.
        std_msgs::msg::Header header;
        header.stamp = stamp;
        header.frame_id = output_frame_;

        visualization_msgs::msg::MarkerArray marker_array = expired_markers;
        visualization_msgs::msg::MarkerArray velocity_array = expired_velocity;
        double best_range = std::numeric_limits<double>::infinity();
        bool have_primary = false;
        TrackPrediction primary_pred;

        for (auto& t : tracks_)
        {
            Point2D origin;
            handleTrackMissing(t, stamp, origin);
            const double horizon = std::max(prediction_dt_, prediction_time_);
            TrackPrediction pred = buildTrackPrediction(t, origin, stamp, horizon, header);

            if (pred.dynamic)
            {
                marker_array.markers.push_back(lineStripMarkerFor(pred, header, t.id));
                velocity_array.markers.push_back(velocityMarkerFor(pred, stamp, t.id));
                if (t.last_range < best_range)
                {
                    best_range = t.last_range;
                    have_primary = true;
                    primary_pred = pred;
                }
            }
            else
            {
                marker_array.markers.push_back(deleteMarker("predicted_obstacle", t.id, stamp));
                velocity_array.markers.push_back(deleteMarker("predicted_obstacle_velocity", t.id, stamp));
            }
        }

        predicted_marker_pub_->publish(marker_array);
        velocity_marker_pub_->publish(velocity_array);
        if (publish_debug_segment_)
            debug_segment_pub_->publish(expired_debug);

        if (have_primary)
        {
            nav_msgs::msg::Path path;
            path.header = header;
            path.poses = primary_pred.poses;
            predicted_path_pub_->publish(path);

            sensor_msgs::msg::PointCloud cloud;
            cloud.header = header;
            double last_x = primary_pred.origin.x, last_y = primary_pred.origin.y, last_z = z_height_;
            for (const auto& pose : primary_pred.poses)
            {
                addDenseSegment(cloud, last_x, last_y, last_z,
                               pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
                last_x = pose.pose.position.x;
                last_y = pose.pose.position.y;
                last_z = pose.pose.position.z;
            }
            predicted_cloud_pub_->publish(cloud);
        }
        else
        {
            clearPrimaryOutputs(stamp);
        }
    }

    // ---- members ----------------------------------------------------

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_cloud_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr detected_pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr predicted_path_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr predicted_cloud_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr predicted_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_segment_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr velocity_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr raw_segments_pub_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::string obstacle_cloud_topic_;
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
    double dynamic_dip_tolerance_;

    double point_spacing_;
    double corridor_radius_;
    double object_half_extent_;
    double center_side_hysteresis_;
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
    int max_tracks_;

    std::vector<Track> tracks_;
    int next_track_id_{0};
    std::size_t last_raw_segment_count_{0};
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DynamicObstaclePredictor>());
    rclcpp::shutdown();
    return 0;
}
