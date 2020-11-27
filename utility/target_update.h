#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"

class TargetUpdate {
public:
  TargetUpdate(const std::array<double, 3> ref);
  void targetUpdateCb(const geometry_msgs::PointStamped::ConstPtr& msg);
  void setUpdated(bool updated);
  double* getTargetValue();
  bool getUpdated();
private:
  ros::NodeHandle nh;
  ros::Subscriber sub;
  double target[3];
  bool is_updated;
};