#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <unistd.h>
#include <tf/transform_listener.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
float fast_vel,slow_vel;
int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);
  
  //创建cmd_vel的发布者
  ros::NodeHandle nhs;
  ros::Publisher vel_pub = nhs.advertise<geometry_msgs::Twist>("cmd_vel", 10);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  ros::param::get("fast",fast_vel);
  ros::param::get("slow",slow_vel);
  ros::param::set("PR",-1);
  move_base_msgs::MoveBaseGoal goal;
  bool Tstart=false;//红绿灯信号开始
  bool Sstart=false;
  bool pathpointchange=false;//表示目标点强行改变
  ros::param::set("pathpointchange",pathpointchange);
  ros::param::set("Tstart",Tstart);
  ros::param::set("Sstart",Sstart);
  float x1,x2,x3,y1,y2,y3,a1,a2,a3,b1,b2,b3,c1,c2,c3,w1,w2,w3;
  ros::param::get("px1",x1);
  ros::param::get("px2",x2);
  ros::param::get("px3",x3);

  ros::param::get("py1",y1);
  ros::param::get("py2",y2);
  ros::param::get("py3",y3);

  ros::param::get("pa1",a1);
  ros::param::get("pa2",a2);
  ros::param::get("pa3",a3);

  ros::param::get("pb1",b1);
  ros::param::get("pb2",b2);
  ros::param::get("pb3",b3);

  ros::param::get("pc1",c1);
  ros::param::get("pc2",c2);
  ros::param::get("pc3",c3);

  ros::param::get("pw1",w1);
  ros::param::get("pw2",w2);
  ros::param::get("pw3",w3);
  int index;
  while(1)
  {
    ros::param::get("goal_index",index);
    if(index==1)//发送目标点1
    {
      ros::param::set("pathpointchange",false);
      
      ros::param::get("px1",x1);
      ros::param::get("py1",y1);
      ros::param::get("pa1",a1);
      ros::param::get("pb1",b1);
      ros::param::get("pc1",c1);
      ros::param::get("pw1",w1);

      index=-1;
      ros::param::set("goal_index",-1);
      goal.target_pose.header.frame_id = "map";
      goal.target_pose.header.stamp = ros::Time::now();
      goal.target_pose.pose.position.x = x1;
      goal.target_pose.pose.position.y = y1;
      goal.target_pose.pose.position.z = 0;
      goal.target_pose.pose.orientation.x = a1;
      goal.target_pose.pose.orientation.y = b1;
      goal.target_pose.pose.orientation.z = c1;
      goal.target_pose.pose.orientation.w = w1;
      
      ROS_INFO("Sending goal1");
      
      ac.sendGoal(goal);
      ros::param::set("pathpointchange",false);
      ros::param::get("pathpointchange",pathpointchange);
      while(!pathpointchange)
      {
        ros::param::get("pathpointchange",pathpointchange);
        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
          Tstart=true;
          ros::param::set("Tstart",Tstart);
          ROS_INFO("PathPoint 1 Reach!");
          ros::param::set("PR",1);
          // //降低导航速度上限
          // ros::param::set("max_vel",slow_vel);
          // sleep(2);
          break;
        }
        sleep(0.2);
      }
    }
    else if(index==2)//发送目标点2
    {
      ros::param::set("RP3",0);
      ros::param::set("RP2",0);
      geometry_msgs::Twist cmd_vel;
      cmd_vel.linear.x=0.5;
      cmd_vel.angular.z=0;
      vel_pub.publish(cmd_vel);
      sleep(1);
      //通过参数服务器开始红绿灯程序
      ros::param::set("pathpointchange",false);
      ros::param::get("pathpointchange",pathpointchange);
     
      ros::param::get("px2",x2);
      ros::param::get("py2",y2);
      ros::param::get("pa2",a2);
      ros::param::get("pb2",b2);
      ros::param::get("pc2",c2);
      ros::param::get("pw2",w2);

      index=-1;
      ros::param::set("goal_index",-1);
      
      goal.target_pose.header.frame_id = "map";
      goal.target_pose.header.stamp = ros::Time::now();
      goal.target_pose.pose.position.x = x2;
      goal.target_pose.pose.position.y = y2;
      goal.target_pose.pose.position.z = 0;
      goal.target_pose.pose.orientation.x = a2;
      goal.target_pose.pose.orientation.y = b2;
      goal.target_pose.pose.orientation.z = c2;
      goal.target_pose.pose.orientation.w = w2;
      int TrafficFlag;
      //ros::param::get("pathpointchange",pathpointchange);
      
      //ROS_INFO("%lf",c2);
      ROS_INFO("Sending goal2");

      ac.sendGoal(goal);
      //在目标点发布后判断是否到红绿灯，如果不到继续走
      //只有在收到红绿灯是红灯的时候才停车，否则都继续走
      ros::param::get("pathpointchange",pathpointchange);
      while(!pathpointchange)
      {
        ros::param::get("pathpointchange",pathpointchange);
        // ROS_INFO("4. %d",pathpointchange);
        ros::param::get("TrafficFlag",TrafficFlag);
        if(TrafficFlag==0)//0表示还没到达红绿灯
          continue;
        else if(TrafficFlag==1)//红灯停
        {
          //取消目标点，等待一定时间之后再发送目标点
          ac.cancelGoal();
          while(!pathpointchange)//等待绿灯
          {
            ros::param::get("pathpointchange",pathpointchange);
            ros::param::get("TrafficFlag",TrafficFlag);
            if(TrafficFlag==2)
            {
              break;
            }
          }
          ac.sendGoal(goal);
          break;
        }
        else if(TrafficFlag==2)//绿灯行
        {
          break;
        }
      }
      ros::param::get("TrafficFlag",TrafficFlag);
      ROS_INFO("TrafficFlag:%d",TrafficFlag);
      ros::param::get("pathpointchange",pathpointchange);
      ROS_INFO("change:%d",pathpointchange);
      
      //提高导航速度上限
      ros::param::set("max_vel",fast_vel);
      ROS_INFO("OUT");
      Tstart=false;
      ros::param::set("Tstart",Tstart);
      TrafficFlag=0;
      ros::param::set("TrafficFlag",TrafficFlag);
      ros::param::get("pathpointchange",pathpointchange);
      //判断目标点是否到达
      while(!pathpointchange)
      {
        int ReachPoint2=0;
        ros::param::get("pathpointchange",pathpointchange);
        ros::param::get("RP2",ReachPoint2);
        if(ReachPoint2==1)//表明到达目标点2附近
        {
          ROS_INFO("NearPathPoint2");
          ac.cancelGoal();
          ros::param::set("RP2",0);
          ros::param::set("PR",2);
          break;
        }
        // if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        // {
        //   ROS_INFO("PathPoint 2 Reach!");
        //   sleep(1);
        //   break;
        // }
        sleep(0.2);
      }     
    }
    else if(index==3)//发送目标点3
    {
      ros::param::set("pathpointchange",false);

      ros::param::get("px3",x3);
      ros::param::get("py3",y3);
      ros::param::get("pa3",a3);
      ros::param::get("pb3",b3);
      ros::param::get("pc3",c3);
      ros::param::get("pw3",w3);
      index=-1;
      ros::param::set("goal_index",-1);
      goal.target_pose.header.frame_id = "map";
      goal.target_pose.header.stamp = ros::Time::now();
      goal.target_pose.pose.position.x = x3;
      goal.target_pose.pose.position.y = y3;
      goal.target_pose.pose.position.z = 0;
      goal.target_pose.pose.orientation.x = a2;
      goal.target_pose.pose.orientation.y = b2;
      goal.target_pose.pose.orientation.z = c2;
      goal.target_pose.pose.orientation.w = w2;

      ROS_INFO("Sending Goal 3");
      std::cout<<goal.target_pose.pose.position.x<<" "<<goal.target_pose.pose.position.y<<" "<<goal.target_pose.pose.position.z<<" "<<
      goal.target_pose.pose.orientation.x<<" "<<goal.target_pose.pose.orientation.y<<" "<<goal.target_pose.pose.orientation.z<<" "<<goal.target_pose.pose.orientation.w;
      ac.sendGoal(goal);
      //发送信号让s弯程序开始执行
      Sstart=true;
      ros::param::set("Sstart",Sstart);
      //等待s弯程序执行到终点
      int SroadFlag=0;
      int S_end=0;//0:完全没到终点  1：到终点附近
      ros::param::set("S_end",S_end);
      ros::param::set("Pos",true);
      ros::param::get("pathpointchange",pathpointchange);
      while(!pathpointchange)
      {
        ros::param::get("pathpointchange",pathpointchange);
        ros::param::get("SroadFlag",SroadFlag);
        if(SroadFlag==-1)//表明s弯可以开始循迹
        {
          ac.cancelGoal();
          while(!pathpointchange)
          {      
            ros::param::get("pathpointchange",pathpointchange);   
            ros::param::get("SroadFlag",SroadFlag);
            if(SroadFlag==1)//表明s弯已经结束
            {
              S_end=0;
              ros::param::set("S_end",S_end);
              ros::param::set("Pos",false);
              break;
            }
          }
          goal.target_pose.header.frame_id = "map";
          goal.target_pose.header.stamp = ros::Time::now();
          goal.target_pose.pose.position.x = x3;
          goal.target_pose.pose.position.y = y3;
          goal.target_pose.pose.position.z = 0;
          goal.target_pose.pose.orientation.x = a3;
          goal.target_pose.pose.orientation.y = b3;
          goal.target_pose.pose.orientation.z = c3;
          goal.target_pose.pose.orientation.w = w3;
          ac.sendGoal(goal);
          break;
        }
        else
          continue;
      }
      ros::param::get("pathpointchange",pathpointchange);
      while(!pathpointchange)
      {
        int ReachPoint3=0;
        ros::param::get("pathpointchange",pathpointchange);
        ros::param::get("RP3",ReachPoint3);
        if(ReachPoint3==1)//表明到达目标点3附近
        {
          ROS_INFO("NearPathPoint3");
          ac.cancelGoal();
          ros::param::set("RP3",0);
          ros::param::set("PR",3);
          break;
        }
        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
          ROS_INFO("PathPoint 3 Reach!");
          ros::param::set("PR",3);
          sleep(1);
          break;
        }
        sleep(0.2);
      }
      Sstart=false;
      SroadFlag=0;
      ros::param::set("Sstart",Sstart);
      ros::param::set("SroadFlag",SroadFlag);
      ros::param::set("S_end", 0);
    }
    else
    {
      sleep(0.3);
      continue;
    }
  }
  return 0;
}
