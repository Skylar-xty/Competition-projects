#include "ros/ros.h"  
#include "std_msgs/Int16.h"  
#include <move_base_msgs/MoveBaseAction.h>  
#include <actionlib/client/simple_action_client.h>  
#include "std_srvs/Empty.h"  
#include <cstdlib>  
int main(int argc, char **argv)  
{  
  ros::init(argc, argv, "costmap_clear");  
 
  ros::NodeHandle n;  
  ros::service::waitForService("/move_base/clear_costmaps");
	
  ros::ServiceClient client = n.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");  
  std_srvs::Empty srv;  
    
  client.call(srv);

  return 0;  
}  
