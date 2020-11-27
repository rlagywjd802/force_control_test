#include <iostream>
#include "force_update.h"

ForceUpdate::ForceUpdate() {
	sub = nh.subscribe("force", 1, &ForceUpdate::forceUpdateCb, this);
	is_updated = false;
	force_[0] = 0.0;
	force_[1] = 0.0;
	force_[2] = 0.0;
}

void ForceUpdate::forceUpdateCb(const std_msgs::Float64MultiArray::ConstPtr& msg) {
  is_updated = true;
  force_[0] = msg->data[0];
  force_[1] = msg->data[1];
  force_[2] = msg->data[6];
}

double* ForceUpdate::getForceValue() {
	return &force_[0];
}

double ForceUpdate::getDForceValue() {
	return dforce_;
}

bool ForceUpdate::getUpdated() {
	return is_updated;
}