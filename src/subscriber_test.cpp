#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

#include "force_update.h"

// void forceUpdateCb(const std_msgs::Float64MultiArray::ConstPtr& msg) {
// 	float data = msg->data[6];
//   std::cout << data << std::endl;
// }


int main(int argc, char** argv) {
	ros::init(argc, argv, "force_test");
  	
  	// ros::NodeHandle nh;
  	// ros::Subscriber sub = nh.subscribe("force", 1, forceUpdateCb);
  	// ros::spin();

  	// ForceUpdate force_update = ForceUpdate();
  	// ros::spinOnce();
   //  ros::Rate sensor_rate(100);

  	// while (true)
  	// {
   //    if(force_update.getUpdated()) {
   //    float last_force = force_update.getForceValue();
   //    std::cout << last_force << std::endl;
   //    sensor_rate.sleep();  
   //    }
   //    ros::spinOnce();
      
  	// }    
  	

}