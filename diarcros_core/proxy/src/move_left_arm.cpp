#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib_msgs/GoalStatus.h>
#include <arm_navigation_msgs/MoveArmAction.h>
#include <arm_navigation_msgs/SimplePoseConstraint.h>
#include <arm_navigation_msgs/utils.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <kinematics_msgs/GetKinematicSolverInfo.h>
#include <kinematics_msgs/GetPositionIK.h>

namespace diarc {
  namespace l_arm {
    
    // Our Action interface type, provided as a typedef for convenience
    typedef actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> MoveArmClient;
    typedef actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction> TrajClient;
    
    MoveArmClient* pose_client;
    TrajClient* traj_client;

    ros::Publisher pose_result_pub;
    ros::Publisher traj_result_pub;

    ros::ServiceClient query_client;
    ros::ServiceClient ik_client;

    kinematics_msgs::GetKinematicSolverInfo::Request request;
    kinematics_msgs::GetKinematicSolverInfo::Response response;

    /**
     * Given a pose, send it to the action server for the left arm,
     * and publish the result.
     */
    void actionGoalCallback(const arm_navigation_msgs::SimplePoseConstraint::ConstPtr& desired_pose) {
      //fill in goal
      arm_navigation_msgs::MoveArmGoal goal;
      goal.motion_plan_request.group_name = "left_arm";
      goal.motion_plan_request.num_planning_attempts = 200;
      goal.motion_plan_request.planner_id = std::string("");
      goal.planner_service_name = std::string("/ompl_planning/plan_kinematic_path");
      goal.motion_plan_request.allowed_planning_time = ros::Duration(5.0);
     
      ROS_INFO("LEFT ARM :: Goal Pose (%.4f, %.4f, %.4f) :: Planning attempts: %d",
	       desired_pose->pose.position.x,
	       desired_pose->pose.position.y,
	       desired_pose->pose.position.z,
	       goal.motion_plan_request.num_planning_attempts);

      
      arm_navigation_msgs::addGoalConstraintToMoveArmGoal(*desired_pose, goal);
      
      pose_client->sendGoal(goal);
      pose_client->waitForResult(ros::Duration(200));

      actionlib::SimpleClientGoalState state = pose_client->getState();
      actionlib_msgs::GoalStatus state_msg;
      state_msg.status = static_cast<uint8_t>(state.state_);
      pose_result_pub.publish(state_msg);
    }
    
    /**
     * Given a pose, solves IK without checking for collisions and
     * sends a joint configuration goal to the robot.
     *
     * Warning: There is NO collision checking.
     */
    void noCollisionCheckCB(const arm_navigation_msgs::SimplePoseConstraint::ConstPtr& dp) {
      ROS_INFO("Goal recieved");
      arm_navigation_msgs::SimplePoseConstraint desired_pose = *dp;
      kinematics_msgs::GetPositionIK::Request  ik_req;
      kinematics_msgs::GetPositionIK::Response ik_res;
      pr2_controllers_msgs::JointTrajectoryGoal goal;
      actionlib_msgs::GoalStatus msg;

      // Call IK to get joint values (no collision checking)
      ik_req.timeout = ros::Duration(5.0);
      ik_req.ik_request.ik_link_name = "l_wrist_roll_link";
      ik_req.ik_request.pose_stamped.header.frame_id = "torso_lift_link";
      ik_req.ik_request.pose_stamped.pose.position.x = desired_pose.pose.position.x;
      ik_req.ik_request.pose_stamped.pose.position.y = desired_pose.pose.position.y;
      ik_req.ik_request.pose_stamped.pose.position.z = desired_pose.pose.position.z;
      ik_req.ik_request.pose_stamped.pose.orientation.x = desired_pose.pose.orientation.x;
      ik_req.ik_request.pose_stamped.pose.orientation.y = desired_pose.pose.orientation.y;
      ik_req.ik_request.pose_stamped.pose.orientation.z = desired_pose.pose.orientation.z;
      ik_req.ik_request.pose_stamped.pose.orientation.w = desired_pose.pose.orientation.w;
      
      ik_req.ik_request.ik_seed_state.joint_state.position.resize(diarc::l_arm::response.kinematic_solver_info.joint_names.size());
      ik_req.ik_request.ik_seed_state.joint_state.name = diarc::l_arm::response.kinematic_solver_info.joint_names;
      
      // seed an initial guess for IK
      for(unsigned int i = 0; i < diarc::l_arm::response.kinematic_solver_info.joint_names.size(); i++) {
        ik_req.ik_request.ik_seed_state.joint_state.position[i] =
          (diarc::l_arm::response.kinematic_solver_info.limits[i].min_position +
           diarc::l_arm::response.kinematic_solver_info.limits[i].max_position) / 2.0;
      }


      // Construct a joint goal if IK succeeds      
      ROS_INFO("Request IK solution");
      if(diarc::l_arm::ik_client.call(ik_req, ik_res) &&
         ik_res.error_code.val == ik_res.error_code.SUCCESS) {
        goal.trajectory.joint_names.push_back("l_shoulder_pan_joint");
        goal.trajectory.joint_names.push_back("l_shoulder_lift_joint");
        goal.trajectory.joint_names.push_back("l_upper_arm_roll_joint");
        goal.trajectory.joint_names.push_back("l_elbow_flex_joint");
        goal.trajectory.joint_names.push_back("l_forearm_roll_joint");
        goal.trajectory.joint_names.push_back("l_wrist_flex_joint");
        goal.trajectory.joint_names.push_back("l_wrist_roll_joint");
        goal.trajectory.points.resize(1);
	goal.trajectory.points[0].time_from_start = ros::Duration(1,0);
        goal.trajectory.points[0].positions.resize(7);
        goal.trajectory.points[0].velocities.resize(7);
        for(unsigned int i = 0; i < ik_res.solution.joint_state.name.size(); i ++) {
          goal.trajectory.points[0].positions[i] = ik_res.solution.joint_state.position[i];
          goal.trajectory.points[0].velocities[i] = 0.15;
          ROS_INFO("[Joint: %s %f]", ik_res.solution.joint_state.name[i].c_str(), ik_res.solution.joint_state.position[i]);
        }

        // Send to client & get result
        ROS_INFO("Sending goal");
        traj_client->sendGoal(goal);
        traj_client->waitForResult(ros::Duration(200));
        actionlib::SimpleClientGoalState state = traj_client->getState();
        msg.status = static_cast<uint8_t>(state.state_);
      } else {
        ROS_ERROR("Inverse kinematics service call failed");
        actionlib::SimpleClientGoalState state = actionlib::SimpleClientGoalState::ABORTED;
        msg.status = static_cast<uint8_t>(state.state_);
      }

      // Publish result code
      ROS_INFO("Result published: %d", msg.status);
      traj_result_pub.publish(msg);
    }
    
  } //diarc
} //move_left_arm

int main(int argc, char** argv) {
  //init the ROS node
  ros::init(argc, argv, "l_arm_proxy");
  ros::NodeHandle nh;
  
  // Initialize the client for the Action interface to the head controller
  diarc::l_arm::pose_client = new diarc::l_arm::MoveArmClient("/move_left_arm", true);
  diarc::l_arm::traj_client = new diarc::l_arm::TrajClient("/l_arm_controller/joint_trajectory_action", true);
  
  // wait for head controller action server to come up
  while (!diarc::l_arm::pose_client->waitForServer(ros::Duration(5.0)) &&
         !diarc::l_arm::traj_client->waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the MoveArm & Trajectory server to come up");
  }


  // Wait for IK services
  ros::service::waitForService("pr2_left_arm_kinematics_nocoll/get_ik_solver_info");
  ros::service::waitForService("pr2_left_arm_kinematics_nocoll/get_ik");

  diarc::l_arm::query_client =
    nh.serviceClient<kinematics_msgs::GetKinematicSolverInfo>("pr2_left_arm_kinematics_nocoll/get_ik_solver_info");
  diarc::l_arm::ik_client =
    nh.serviceClient<kinematics_msgs::GetPositionIK>("pr2_left_arm_kinematics_nocoll/get_ik");

  // Get IK solver info
  if(diarc::l_arm::query_client.call(diarc::l_arm::request, diarc::l_arm::response)) {
    for(unsigned int i = 0; i < diarc::l_arm::response.kinematic_solver_info.joint_names.size(); i++) {
      ROS_DEBUG("Joint: %d %s", i, diarc::l_arm::response.kinematic_solver_info.joint_names[i].c_str());
    }
  }
  else {
    ROS_ERROR("Could not call query service");
    ros::shutdown();
    exit(1);
  }

   
  // Result publishers
  diarc::l_arm::pose_result_pub = nh.advertise<actionlib_msgs::GoalStatus>("/DIARC/proxy/move_left_arm/result", 10);
  diarc::l_arm::traj_result_pub = nh.advertise<actionlib_msgs::GoalStatus>("/DIARC/proxy/l_arm_joint/result", 10);
  

  // Goal subscribers
  ros::Subscriber pose_sub = nh.subscribe("/DIARC/proxy/move_left_arm/goal", 10, diarc::l_arm::actionGoalCallback);
  ros::Subscriber traj_sub = nh.subscribe("/DIARC/proxy/move_left_arm_nocoll/goal", 10, diarc::l_arm::noCollisionCheckCB);
  ROS_INFO("Left arm proxy initialized");
  ros::spin();
}
