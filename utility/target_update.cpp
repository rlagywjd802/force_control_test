#include <iostream>
#include "target_update.h"

TargetUpdate::TargetUpdate(const std::array<double, 3> ref) {
	sub = nh.subscribe("target_pose", 100, &TargetUpdate::targetUpdateCb, this);
	is_updated = false;
	target[0] = ref[0];
	target[1] = ref[1];
	target[2] = ref[2];
}

void TargetUpdate::targetUpdateCb(const geometry_msgs::PointStamped::ConstPtr& msg) {
  is_updated = true;
  target[0] = msg->point.x;
  target[1] = msg->point.y;
  target[2] = msg->point.z;
  std::cout << "callback*****" << std::endl;
}

void TargetUpdate::setUpdated(bool updated) {
  is_updated = updated;
}

double* TargetUpdate::getTargetValue() {
	return &target[0];
}

bool TargetUpdate::getUpdated() {
	return is_updated;
}