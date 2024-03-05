#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib_msgs/GoalStatus.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>

namespace ade {
  namespace r_arm_traj {
    typedef actionlib::SimpleActionClient< pr2_controllers_msgs::JointTrajectoryAction > TrajClient;

    TrajClient* r_traj_client;
    ros::Publisher result_pub;

    void callback(const trajectory_msgs::JointTrajectory::ConstPtr& traj) {
      pr2_controllers_msgs::JointTrajectoryGoal goal;
      goal.trajectory = *traj;
      goal.trajectory.joint_names.push_back("r_shoulder_pan_joint");
      goal.trajectory.joint_names.push_back("r_shoulder_lift_joint");
      goal.trajectory.joint_names.push_back("r_upper_arm_roll_joint");
      goal.trajectory.joint_names.push_back("r_elbow_flex_joint");
      goal.trajectory.joint_names.push_back("r_forearm_roll_joint");
      goal.trajectory.joint_names.push_back("r_wrist_flex_joint");
      goal.trajectory.joint_names.push_back("r_wrist_roll_joint");
      ROS_INFO("Goal Received (%d joints, %d pos, %d vel)", 
               goal.trajectory.joint_names.size(),
               goal.trajectory.points[0].positions.size(),
               goal.trajectory.points[0].velocities.size());
      // Print out goal
      // for (int i = 0; i < 7; i++) {
      //   ROS_INFO("%s :: %f :: %f",
      //            goal.trajectory.joint_names[i].c_str(),
      //            goal.trajectory.points[0].positions[i],
      //            goal.trajectory.points[0].velocities[i]);
      // }

      r_traj_client->sendGoal(goal);
      r_traj_client->waitForResult(ros::Duration(200));
      actionlib::SimpleClientGoalState state = r_traj_client->getState();
      actionlib_msgs::GoalStatus msg;
      msg.status = static_cast<uint8_t>(state.state_);
      result_pub.publish(msg);
      ROS_INFO("Result published");
    }
  }
}


int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "ADE/proxy/r_arm_joint_controller");
  ros::NodeHandle nh;

  ade::r_arm_traj::r_traj_client = new ade::r_arm_traj::TrajClient("r_arm_controller/joint_trajectory_action", true);
  while (!ade::r_arm_traj::r_traj_client->waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the r_arm_joint_controller proxy to come up");
  }

  // setup pub/sub
  ade::r_arm_traj::result_pub = nh.advertise<actionlib_msgs::GoalStatus>("/ADE/proxy/r_arm_joint/result", 10);
  ros::Subscriber sub = nh.subscribe("/ADE/proxy/r_arm_joint/goal", 10, ade::r_arm_traj::callback);
  ros::spin();
}
