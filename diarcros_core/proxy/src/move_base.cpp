/**
 * A proxt node for interacint with the move_base action server.
 *
 * Jeremiah Via <jeremiah.via@gmail.com>
 */
#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib_msgs/GoalStatus.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>

namespace diarc {
  namespace move_base {

    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

    MoveBaseClient* move_base_client;
    ros::Publisher action_state_pub;
    ros::Subscriber action_goal_sub;

    void mbCallback(const geometry_msgs::PoseStamped::ConstPtr& goal) {
      ROS_INFO("Received goal");
      move_base_msgs::MoveBaseGoal goal_pose;
      goal_pose.target_pose = *goal;
      goal_pose.target_pose.header.stamp = ros::Time::now();
      move_base_client->sendGoal(goal_pose);
      // move_base_client->waitForResult();
      // actionlib::SimpleClientGoalState state = move_base_client->getState();
      // actionlib_msgs::GoalStatus state_msg;
      // state_msg.status = static_cast<uint8_t>(state.state_);
      // action_state_pub.publish(state_msg);         
    }
  }
}

int main(int argc, char** argv){
  // Node initialization
  ros::init(argc, argv, "move_base_proxy");
  ros::NodeHandle node_handle;

  // Connection to move_base action server
diarc::move_base::move_base_client = new diarc::move_base::MoveBaseClient("move_base", true);
  while(!diarc::move_base::move_base_client->waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  // Set up publisher and subscriber
  diarc::move_base::action_state_pub = node_handle.advertise<actionlib_msgs::GoalStatus>("/DIARC/proxy/move_base/result", 10);
  diarc::move_base::action_goal_sub  = node_handle.subscribe("/DIARC/proxy/move_base/goal", 10, diarc::move_base::mbCallback);

  ros::spin();
}
