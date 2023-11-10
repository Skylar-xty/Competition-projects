#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  float x1,x2,x3,y1,y2,y3;
  ros::param::get("px1",x1);
  ros::param::get("px2",x2);
  ros::param::get("px3",x3);
  ros::param::get("py1",y1);
  ros::param::get("py2",y2);
  ros::param::get("py3",y3);
  ROS_INFO("%f",x1);
  ROS_INFO("%f",y1);
  int index=1;
  while(1)
  {
    if(index==1)
    {
      index++;
      goal.target_pose.header.frame_id = "map";
      goal.target_pose.header.stamp = ros::Time::now();
      ROS_INFO("%f",x1);
      ROS_INFO("%f",y1);
      goal.target_pose.pose.position.x = x1;
      goal.target_pose.pose.position.y = y1;
      goal.target_pose.pose.position.z = -2.7;
      goal.target_pose.pose.orientation.w = 1;

      ROS_INFO("Sending goal");
      ac.sendGoal(goal);

      ac.waitForResult();

      if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Hooray, the base moved 1 meter forward");
      else
      {
        ROS_INFO("The base failed to move forward 1 meter for some reason");
        break;
      }
    }
    else if(index==2)
    {
      index++;
      goal.target_pose.header.frame_id = "map";
      goal.target_pose.header.stamp = ros::Time::now();
      ROS_INFO("%f",x2);
      ROS_INFO("%f",y2);
      goal.target_pose.pose.position.x = x2;
      goal.target_pose.pose.position.y = y2;
      goal.target_pose.pose.position.z = -2.7;
      goal.target_pose.pose.orientation.w = 1;

      ROS_INFO("Sending goal");
      ac.sendGoal(goal);

      ac.waitForResult();

      if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Hooray, the base moved 1 meter forward");
      else
      {
        ROS_INFO("The base failed to move forward 1 meter for some reason");
        break;
      }     
    }
    else
    {
      index=1;
      goal.target_pose.header.frame_id = "map";
      goal.target_pose.header.stamp = ros::Time::now();
      ROS_INFO("%f",x3);
      ROS_INFO("%f",y3);
      goal.target_pose.pose.position.x = x3;
      goal.target_pose.pose.position.y = y3;
      goal.target_pose.pose.position.z = -2.7;
      goal.target_pose.pose.orientation.w = 1;

      ROS_INFO("Sending goal");
      ac.sendGoal(goal);

      ac.waitForResult();

      if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Hooray, the base moved 1 meter forward");
      else
      {
        ROS_INFO("The base failed to move forward 1 meter for some reason");
        break;
      }
    }
  }
  return 0;
}