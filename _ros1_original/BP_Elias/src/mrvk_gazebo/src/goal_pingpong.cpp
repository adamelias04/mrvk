// Pingpong experiment: robot strieda dva ciele A <-> B N-krat.
// Publikuje /nav_tour/active (latched std_msgs/Bool) pre nav_metrics:
//   true  -- pred prvym cielom (nav_metrics zacne merat beh)
//   false -- po poslednom cieli   (nav_metrics ukonci a zapise vysledky)
// Tym sa cely pingpong odmera ako JEDEN suvisly experiment (rovnaky
// mechanizmus ako waypoint_tour).
//
// Parametre (private namespace, defaulty zodpovedaju zadaniu (0,-6) <-> (0,6), 8x):
//   pose_a_x, pose_a_y      [m]   prvy ciel (default 0, -6)
//   pose_b_x, pose_b_y      [m]   druhy ciel (default 0,  6)
//   repeats                 [-]   pocet cielov spolu (default 8)
//   goal_timeout            [s]   max cas na dosiahnutie jedneho ciela (default 60)
//   pause_min, pause_max    [s]   nahodna pauza medzi cielmi v intervale
//                                 [pause_min, pause_max] (default 0..10)
//   seed                    [-]   seed RNG pre pauzy. FIXNY default 42 ->
//                                 vsetky behy/planovace dostanu IDENTICKU
//                                 sekvenciu pauz (ziadny nie je zvyhodneny).
//   frame_id                [-]   ramec cielov (default "odom")

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <std_msgs/Bool.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <random>

using MoveBaseClient = actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;

static move_base_msgs::MoveBaseGoal makeGoal(const std::string& frame,
                                             double x, double y,
                                             double yaw,
                                             const ros::Time& stamp)
{
  move_base_msgs::MoveBaseGoal g;
  g.target_pose.header.frame_id = frame;
  g.target_pose.header.stamp    = stamp;
  g.target_pose.pose.position.x = x;
  g.target_pose.pose.position.y = y;
  g.target_pose.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  g.target_pose.pose.orientation = tf2::toMsg(q);
  return g;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "goal_pingpong");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  double ax, ay, bx, by;
  int repeats, seed;
  double goal_timeout, pause_min, pause_max;
  std::string frame_id;

  pnh.param("pose_a_x",       ax,             0.0);
  pnh.param("pose_a_y",       ay,            -6.0);
  pnh.param("pose_b_x",       bx,             0.0);
  pnh.param("pose_b_y",       by,             6.0);
  pnh.param("repeats",        repeats,       8);
  pnh.param("goal_timeout",   goal_timeout,  60.0);
  pnh.param("pause_min",      pause_min,     0.0);
  pnh.param("pause_max",      pause_max,    10.0);
  pnh.param("seed",           seed,         42);
  pnh.param<std::string>("frame_id", frame_id, "odom");

  if (pause_max < pause_min) std::swap(pause_min, pause_max);

  // Fixny seed -> identicka sekvencia pauz pre vsetky behy/planovace.
  std::mt19937 rng(static_cast<uint32_t>(seed));
  std::uniform_real_distribution<double> pause_dist(pause_min, pause_max);

  if (repeats < 1) {
    ROS_ERROR("goal_pingpong: 'repeats' must be >= 1 (got %d)", repeats);
    return 1;
  }

  // Latched publisher pre nav_metrics
  ros::Publisher active_pub =
      nh.advertise<std_msgs::Bool>("/nav_tour/active", 1, /*latch=*/true);

  ROS_INFO("goal_pingpong: A=(%.2f, %.2f)  B=(%.2f, %.2f)  repeats=%d",
           ax, ay, bx, by, repeats);

  MoveBaseClient client("move_base", true);
  ROS_INFO("goal_pingpong: waiting for move_base action server...");
  client.waitForServer();
  ROS_INFO("goal_pingpong: move_base ready.");

  // Aktivuj meranie
  {
    std_msgs::Bool m;
    m.data = true;
    active_pub.publish(m);
  }
  ros::Duration(0.5).sleep();   // daj nav_metrics cas pripojit sa

  int successes = 0;
  for (int i = 0; i < repeats && ros::ok(); ++i)
  {
    const bool use_a = (i % 2 == 0);
    const double gx  = use_a ? ax : bx;
    const double gy  = use_a ? ay : by;
    // Smer = od aktualneho ciela k druhemu (priblizne, aby robot necelil dozadu)
    const double yaw = std::atan2((use_a ? by : ay) - gy,
                                  (use_a ? bx : ax) - gx);

    auto goal = makeGoal(frame_id, gx, gy, yaw, ros::Time::now());
    ROS_INFO("goal_pingpong [%d/%d]: -> %s (%.2f, %.2f)",
             i + 1, repeats, use_a ? "A" : "B", gx, gy);

    client.sendGoal(goal);
    const bool finished = client.waitForResult(ros::Duration(goal_timeout));

    if (!finished) {
      ROS_WARN("goal_pingpong: goal %d timed out, cancelling.", i + 1);
      client.cancelGoal();
    } else if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO("goal_pingpong: goal %d SUCCEEDED.", i + 1);
      ++successes;
    } else {
      ROS_WARN("goal_pingpong: goal %d ended in state %s.",
               i + 1, client.getState().toString().c_str());
    }

    if (i + 1 < repeats && ros::ok()) {
      const double pause = pause_dist(rng);
      ROS_INFO("goal_pingpong: pausing %.2f s before next goal "
               "(seed=%d, range=[%.1f, %.1f])",
               pause, seed, pause_min, pause_max);
      if (pause > 0.0) ros::Duration(pause).sleep();
    }
  }

  // Ukonci meranie
  {
    std_msgs::Bool m;
    m.data = false;
    active_pub.publish(m);
  }
  ROS_INFO("goal_pingpong: done. %d/%d goals SUCCEEDED.",
           successes, repeats);
  ros::Duration(0.5).sleep();   // daj nav_metrics cas zapisat
  return 0;
}
