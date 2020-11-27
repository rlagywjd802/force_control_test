#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

class ForceUpdate {
public:
  ForceUpdate();
  void forceUpdateCb(const std_msgs::Float64MultiArray::ConstPtr& msg);
  double* getForceValue();
  double getDForceValue();
  bool getUpdated();
private:
  ros::NodeHandle nh;
  ros::Subscriber sub;
  double force_[3];
  double dforce_;
  double time_;
  bool is_updated;
};