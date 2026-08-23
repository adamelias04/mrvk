// Posiela robota cez sekvenciu 3 waypointov (cela mapa) cez move_base
// actionlib. Pred prvym goalom publikuje /nav_tour/active=true, po
// poslednom =false — nav_metrics.py v tour_mode tak meria CELU turu
// ako jeden beh (nie 3 samostatne).
//
// Defaultne waypointy: (-6,-6) -> (-3,5) -> (3,0)
//
// Parametre (~):
//   ~wpN_x / ~wpN_y   (N=1..3) suradnice waypointov
//   ~frame_id         (str, "odom")
//   ~goal_timeout     (double, 120) max cas na jeden waypoint [s]
//   ~repeats          (int, 1) kolko krat zopakovat celu turu
//   ~pause_between    (double, 1.0) pauza medzi turami [s]

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <std_msgs/Bool.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>
#include <utility>

using MoveBaseClient = actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;

namespace {

const char* stateName(actionlib::SimpleClientGoalState::StateEnum s) {
  switch (s) {
    case actionlib::SimpleClientGoalState::SUCCEEDED: return "SUCCEEDED";
    case actionlib::SimpleClientGoalState::ABORTED:   return "ABORTED";
    case actionlib::SimpleClientGoalState::REJECTED:  return "REJECTED";
    case actionlib::SimpleClientGoalState::PREEMPTED: return "PREEMPTED";
    case actionlib::SimpleClientGoalState::LOST:      return "LOST";
    default:                                          return "OTHER";
  }
}

move_base_msgs::MoveBaseGoal makeGoal(const std::string& frame,
                                      double x, double y, double yaw) {
  move_base_msgs::MoveBaseGoal g;
  g.target_pose.header.frame_id = frame;
  g.target_pose.header.stamp    = ros::Time::now();
  g.target_pose.pose.position.x = x;
  g.target_pose.pose.position.y = y;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  g.target_pose.pose.orientation = tf2::toMsg(q);
  return g;
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "waypoint_tour");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::string frame = pnh.param<std::string>("frame_id", std::string("odom"));
  double goal_timeout = pnh.param("goal_timeout", 120.0);
  int    repeats      = pnh.param("repeats", 1);
  double pause_between = pnh.param("pause_between", 1.0);

  // Defaultne 3 waypointy (test_static tura). Volitelne wp4..wp8 sa
  // pridaju iba ak su nastavene oba parametre (wpN_x aj wpN_y) — to
  // umoznuje dlhsie tury napr. v Baylands bez zmeny kodu.
  std::vector<std::pair<double, double>> wps = {
    {pnh.param("wp1_x", -6.0), pnh.param("wp1_y", -6.0)},
    {pnh.param("wp2_x", -3.0), pnh.param("wp2_y",  5.0)},
    {pnh.param("wp3_x",  3.0), pnh.param("wp3_y",  0.0)},
  };
  for (int n = 4; n <= 8; ++n) {
    const std::string kx = "wp" + std::to_string(n) + "_x";
    const std::string ky = "wp" + std::to_string(n) + "_y";
    if (pnh.hasParam(kx) && pnh.hasParam(ky)) {
      double x = 0.0, y = 0.0;
      pnh.getParam(kx, x);
      pnh.getParam(ky, y);
      wps.emplace_back(x, y);
    }
  }

  ros::Publisher tour_pub =
      nh.advertise<std_msgs::Bool>("/nav_tour/active", 1, /*latch=*/true);

  MoveBaseClient client("move_base", true);
  ROS_INFO("[waypoint_tour] cakam na move_base action server...");
  if (!client.waitForServer(ros::Duration(30.0))) {
    ROS_ERROR("[waypoint_tour] move_base nedostupny.");
    return 1;
  }
  {
    std::string wp_log;
    for (const auto& w : wps) {
      char buf[64];
      std::snprintf(buf, sizeof(buf), "(%.1f,%.1f) ", w.first, w.second);
      wp_log += buf;
    }
    ROS_INFO("[waypoint_tour] pripojene. %zu waypointov: %s repeats=%d",
             wps.size(), wp_log.c_str(), repeats);
  }

  for (int rep = 0; rep < repeats && ros::ok(); ++rep) {
    ROS_INFO("[waypoint_tour] ===== TURA %d/%d START =====", rep + 1, repeats);

    std_msgs::Bool act;
    act.data = true;
    tour_pub.publish(act);
    ros::Duration(0.3).sleep();  // nech metrics stihne zachytit start

    for (std::size_t i = 0; i < wps.size() && ros::ok(); ++i) {
      const auto& cur = wps[i];
      // natoc sa do smeru dalsieho wp (posledny -> 0)
      double yaw = 0.0;
      if (i + 1 < wps.size())
        yaw = std::atan2(wps[i + 1].second - cur.second,
                         wps[i + 1].first  - cur.first);

      ROS_INFO("[waypoint_tour] WP %zu/%zu -> (%.2f, %.2f)",
               i + 1, wps.size(), cur.first, cur.second);
      client.sendGoal(makeGoal(frame, cur.first, cur.second, yaw));

      bool fin = client.waitForResult(ros::Duration(goal_timeout));
      if (!fin) {
        ROS_WARN("[waypoint_tour] WP %zu timeout (%.0fs), rusim a pokracujem.",
                 i + 1, goal_timeout);
        client.cancelGoal();
      } else {
        ROS_INFO("[waypoint_tour] WP %zu -> %s", i + 1,
                 stateName(client.getState().state_));
      }
      if (!ros::ok()) break;
    }

    act.data = false;
    tour_pub.publish(act);
    ROS_INFO("[waypoint_tour] ===== TURA %d/%d KONIEC =====", rep + 1, repeats);

    if (rep + 1 < repeats && ros::ok())
      ros::Duration(pause_between).sleep();
  }

  ROS_INFO("[waypoint_tour] hotovo (%d tur).", repeats);
  return 0;
}
