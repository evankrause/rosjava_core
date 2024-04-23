#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib_msgs/GoalStatus.h>
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>
#include <pr2_controllers_msgs/Pr2GripperCommand.h>

namespace diarc {
  namespace right_gripper {
    
    // Our Action interface type, provided as a typedef for convenience
    typedef actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction> GripperClient;
    
    GripperClient* gripper_client_;    
    ros::Publisher action_state_pub;
    
    
    void actionGoalCallback(const pr2_controllers_msgs::Pr2GripperCommand::ConstPtr& desired_goal) {
      ROS_INFO("Received goal");
      //fill in goal
      pr2_controllers_msgs::Pr2GripperCommandGoal goal;
      goal.command = *desired_goal;
      gripper_client_->sendGoal(goal);
      gripper_client_->waitForResult();

      actionlib::SimpleClientGoalState state = gripper_client_->getState();
      ROS_INFO("RESULT: %s", gripper_client_->getState().toString().c_str());
      ROS_INFO("Result: %d", state.state_);
      actionlib_msgs::GoalStatus state_msg;
      state_msg.status = static_cast<uint8_t>(state.state_);
      gripper_client_->cancelAllGoals();
      action_state_pub.publish(state_msg);
    }
  } //diarc
} //right_gripper

int main(int argc, char** argv) {
  //init the ROS node
  ros::init(argc, argv, "/DIARC/proxy/right_gripper");
  
  ros::NodeHandle n;
  
  //Initialize the client for the Action interface to the head controller
  diarc::right_gripper::gripper_client_ = new diarc::right_gripper::GripperClient("/r_gripper_controller/gripper_action", true);
  
  //wait for head controller action server to come up
  while (!diarc::right_gripper::gripper_client_->waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_arm_client server to come up");
  }
  
  //publish and subscribe
  diarc::right_gripper::action_state_pub = n.advertise<actionlib_msgs::GoalStatus>("/DIARC/proxy/right_gripper/result", 10);
  ros::Subscriber action_goal_sub = n.subscribe("/DIARC/proxy/right_gripper/goal", 1, diarc::right_gripper::actionGoalCallback);
  ros::spin();
}
