#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <pr2_controllers_msgs/PointHeadAction.h>
#include <actionlib_msgs/GoalStatus.h>
#include <geometry_msgs/PointStamped.h>

namespace ade {
  namespace move_head {
    // Our Action interface type, provided as a typedef for convenience
    typedef actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction> PointHeadClient;
    
    PointHeadClient* point_head_client_;
    pr2_controllers_msgs::PointHeadGoal goal;
    geometry_msgs::PointStamped point;
    bool newGoal = false; //TODO: lock
    ros::Publisher action_state_pub;
    
    void actionGoalCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
      //fill in goal
      //the goal message we will be sending
      //pr2_controllers_msgs::PointHeadGoal _goal;
      ROS_INFO("[HEAD PROXY] :: New Goal");
      
      //the target point, expressed in the requested frame
      goal.target.header.frame_id = msg->header.frame_id;
      goal.target.point.x = msg->point.x;
      goal.target.point.y = msg->point.y;
      goal.target.point.z = msg->point.z;
      
      //we are pointing the high-def camera frame
      //(pointing_axis defaults to X-axis)
      goal.pointing_frame = "high_def_frame";
      goal.pointing_axis.x = 1;
      goal.pointing_axis.y = 0;
      goal.pointing_axis.z = 0;
      
      //take at least 0.5 seconds to get there
      goal.min_duration = ros::Duration(0.5);
      
      //and go no faster than ...
      goal.max_velocity = 0.5;
      
      point_head_client_->sendGoal(ade::move_head::goal);
      point_head_client_->waitForResult(ros::Duration(3));
      actionlib::SimpleClientGoalState state = point_head_client_->getState();
      actionlib_msgs::GoalStatus state_msg;
      state_msg.status = static_cast<uint8_t>(state.state_);
      action_state_pub.publish(state_msg);
    }
  }	//ade 
}	//move_head

int main(int argc, char** argv) {
  //init the ROS node
  ros::init(argc, argv, "ADEproxyhead");
  
  ros::NodeHandle n;
  
  //Initialize the client for the Action interface to the head controller
  ade::move_head::point_head_client_ = new ade::move_head::PointHeadClient("/head_traj_controller/point_head_action", true);
  
  //wait for head controller action server to come up
  while (!ade::move_head::point_head_client_->waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the point_head_action server to come up");
  }
  
  //publish and subscribe
  ade::move_head::action_state_pub = n.advertise<actionlib_msgs::GoalStatus>("/ADE/proxy/head_traj_controller/point_head_action/result", 100);
  ros::Subscriber action_goal_sub = n.subscribe("/ADE/proxy/head_traj_controller/point_head_action/goal", 100, ade::move_head::actionGoalCallback);
  ros::spin();

  // ROS_INFO("Ready");
  // //ros::Rate loop_rate(10);
  
  // while (ros::ok()) {
  
  //     if (ade::move_head::newGoal) {
  //         ROS_INFO("New goal");
  //         //send the goal
  //         ade::move_head::point_head_client_->sendGoal(ade::move_head::goal);
  //         ROS_INFO("Goal Sent");
  //         //wait for it to get there (abort after 3 secs to prevent getting stuck)
  //         ade::move_head::point_head_client_->waitForResult(ros::Duration(30));
  //         ROS_INFO("Finished");
  //         //publish result
  //         actionlib::SimpleClientGoalState state = ade::move_head::point_head_client_->getState();
  //         actionlib_msgs::GoalStatus state_msg;
  //         state_msg.status = static_cast<uint8_t>(state.state_);
  //         //action_state_pub.publish(state_msg);
  
  //         ade::move_head::newGoal = false;
  //     }
  
  //     ros::spinOnce();
  
  //     //loop_rate.sleep();

  // }
  
}
