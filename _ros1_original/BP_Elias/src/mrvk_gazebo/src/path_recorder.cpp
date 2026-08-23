// Akumuluje pozicie robota z /odom do nav_msgs/Path a publikuje
// pre vizualizaciu v RViz. Body pridava len ak sa robot posunul
// aspon o ~min_distance (aby path nerastol donekonecna).
//
// Parametre (~private):
//   ~odom_topic    (string, "/shoddy/odom")
//   ~path_topic    (string, "/traveled_path")
//   ~frame_id      (string, "odom")
//   ~min_distance  (double, 0.05)  min posun medzi bodmi [m]
//   ~max_points    (int,    0)     ring buffer limit (0 = bez limitu)
//   ~publish_rate  (double, 5.0)   Hz pre publikovanie celej Path
//   ~reset_topic   (string, "/traveled_path/reset")  std_msgs/Empty -> vycistit

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Empty.h>

#include <cmath>
#include <deque>
#include <string>

class PathRecorder {
public:
  PathRecorder() : pnh_("~") {
    odom_topic_   = pnh_.param<std::string>("odom_topic",  std::string("/shoddy/odom"));
    path_topic_   = pnh_.param<std::string>("path_topic",  std::string("/traveled_path"));
    frame_id_     = pnh_.param<std::string>("frame_id",    std::string("odom"));
    reset_topic_  = pnh_.param<std::string>("reset_topic", std::string("/traveled_path/reset"));
    min_distance_ = pnh_.param("min_distance", 0.05);
    max_points_   = pnh_.param("max_points",   0);
    double rate   = pnh_.param("publish_rate", 5.0);

    path_pub_  = nh_.advertise<nav_msgs::Path>(path_topic_, 1, /*latch=*/true);
    odom_sub_  = nh_.subscribe(odom_topic_, 50, &PathRecorder::odomCb, this);
    reset_sub_ = nh_.subscribe(reset_topic_, 1, &PathRecorder::resetCb, this);

    timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(rate, 0.1)),
                             &PathRecorder::publishCb, this);

    have_last_ = false;
    ROS_INFO("[path_recorder] odom=%s -> path=%s frame=%s min_dist=%.3fm max_pts=%d",
             odom_topic_.c_str(), path_topic_.c_str(), frame_id_.c_str(),
             min_distance_, max_points_);
  }

private:
  void odomCb(const nav_msgs::Odometry::ConstPtr& msg) {
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;

    if (have_last_) {
      double dx = x - last_x_;
      double dy = y - last_y_;
      if (dx * dx + dy * dy < min_distance_ * min_distance_) return;
    }

    geometry_msgs::PoseStamped p;
    p.header.stamp    = msg->header.stamp;
    p.header.frame_id = frame_id_;
    p.pose            = msg->pose.pose;
    poses_.push_back(p);

    if (max_points_ > 0 && static_cast<int>(poses_.size()) > max_points_) {
      poses_.pop_front();
    }

    last_x_    = x;
    last_y_    = y;
    have_last_ = true;
  }

  void resetCb(const std_msgs::Empty::ConstPtr&) {
    poses_.clear();
    have_last_ = false;
    ROS_INFO("[path_recorder] path resetnuty.");
    // okamzity prazdny publish, aby sa RViz prekreslil
    publishPath();
  }

  void publishCb(const ros::TimerEvent&) {
    publishPath();
  }

  void publishPath() {
    nav_msgs::Path path;
    path.header.stamp    = ros::Time::now();
    path.header.frame_id = frame_id_;
    path.poses.assign(poses_.begin(), poses_.end());
    path_pub_.publish(path);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher  path_pub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber reset_sub_;
  ros::Timer      timer_;

  std::string odom_topic_, path_topic_, frame_id_, reset_topic_;
  double      min_distance_;
  int         max_points_;

  std::deque<geometry_msgs::PoseStamped> poses_;
  double last_x_ = 0.0, last_y_ = 0.0;
  bool   have_last_ = false;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "path_recorder");
  PathRecorder node;
  ros::spin();
  return 0;
}
