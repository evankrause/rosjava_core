#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib_msgs/GoalStatus.h>
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>
#include <pr2_controllers_msgs/Pr2GripperCommand.h>

namespace ade {
  namespace left_gripper {
    
    // Our Action interface type, provided as a typedef for convenience
    typedef actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction> GripperClient;
    
    GripperClient* gripper_client_;
    pr2_controllers_msgs::Pr2GripperCommandGoal goal;
    ros::Publisher action_state_pub;
    
    
    void actionGoalCallback(const pr2_controllers_msgs::Pr2GripperCommand::ConstPtr& desired_goal) {
      //fill in goal
      goal.command = *desired_goal;
      gripper_client_->sendGoal(goal);
      gripper_client_->waitForResult(ros::Duration(5.0));

      actionlib::SimpleClientGoalState state = gripper_client_->getState();
      actionlib_msgs::GoalStatus state_msg;
      state_msg.status = static_cast<uint8_t>(state.state_);
      action_state_pub.publish(state_msg);      
    }
    
  } //ade
} //left_gripper

int main(int argc, char** argv) {
  //init the ROS node
  ros::init(argc, argv, "left_gripper_proxy");
  
  ros::NodeHandle n;
  
  //Initialize the client for the Action interface to the head controller
  ade::left_gripper::gripper_client_ = new ade::left_gripper::GripperClient("/l_gripper_controller/gripper_action", true);
  
  //wait for head controller action server to come up
  while (!ade::left_gripper::gripper_client_->waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_arm_client server to come up");
  }
  
  //publish and subscribe
  ade::left_gripper::action_state_pub = n.advertise<actionlib_msgs::GoalStatus>("/ADE/proxy/left_gripper/result", 10);
  ros::Subscriber action_goal_sub = n.subscribe("/ADE/proxy/left_gripper/goal", 10, ade::left_gripper::actionGoalCallback);
  ros::spin();
}
