// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include "landing_motion.h"

#include <algorithm>
#include <array>
#include <cmath>

#include <iostream>

std::array<double, 2> solveCubicZero(double vf, double af, double tf){
  std::array<double, 2> coeff;
  coeff[0] = -2*vf/std::pow(tf, 3) + af/std::pow(tf, 2);
  coeff[1] = 3*vf/std::pow(tf, 2) - af/tf;
  return coeff;
}

LinearLanding::LinearLanding(double acc_time, const std::array<std::array<double, 3>, 3> p0)
: tf_(acc_time){
  xf_ = p0[0];
  vf_ = p0[1];
  af_ = p0[2];
  getMotionParameter();
}

void LinearLanding::getMotionParameter() {
  double v_n, a_n, th_n; 
  double d_n, dx, dy;

  // cubic
  v_n = std::sqrt(vf_[0]*vf_[0] + vf_[1]*vf_[1]);
  a_n = std::sqrt(af_[0]*af_[0] + af_[1]*af_[1]);
  th_n = std::atan2(vf_[1], vf_[0]);
  coeff_ = solveCubicZero(v_n, a_n, tf_);

  // get x0
  d_n = coeff_[0]/4.0*std::pow(tf_, 4) + coeff_[1]/3.0*std::pow(tf_, 3);
  dx = d_n*std::cos(th_n);
  dy = d_n*std::sin(th_n);
  x0_ = {xf_[0]-dx, xf_[1]-dy, xf_[2]};
}

void LinearLanding::printMotionParameter() {
  std::cout << std::endl << "x0_:";
  for (const auto& a:x0_)
    std::cout << a << ",";

  std::cout << std::endl << "coeff_:";
  for (const auto& a:coeff_)
    std::cout << a << ",";

  std::cout << std::endl << "==========================";

  std::cout << std::endl << "xf_:";
  for (const auto& a:xf_)
    std::cout << a << ",";

  std::cout << std::endl << "vf_:";
  for (const auto& a:vf_)
    std::cout << a << ",";

  std::cout << std::endl << "af_:";
  for (const auto& a:af_)
    std::cout << a << ",";
  std::cout << std::endl;
}

std::array<double, 3> LinearLanding::getDesiredPose(double t) const {
  std::array<double, 3> desired_pt;
  double th_n, d_n, dx, dy;

  th_n = std::atan2(vf_[1], vf_[0]);
  d_n = coeff_[0]/4.0*std::pow(t, 4) + coeff_[1]/3.0*std::pow(t, 3);
  dx = d_n*std::cos(th_n);
  dy = d_n*std::sin(th_n);

  desired_pt[0] = x0_[0] + dx;
  desired_pt[1] = x0_[1] + dy;
  desired_pt[2] = x0_[2];

  return desired_pt;
}

std::array<double, 3> LinearLanding::getDesiredVel(double t) const {
  std::array<double, 3> desired_vel;
  double th_n, d_n, dx, dy;

  th_n = std::atan2(vf_[1], vf_[0]);
  d_n = coeff_[0]*std::pow(t, 3) + coeff_[1]*std::pow(t, 2);
  dx = d_n*std::sin(th_n);
  dy = d_n*std::cos(th_n);

  desired_vel[0] = dx;
  desired_vel[1] = dy;
  desired_vel[2] = 0.0;

  return desired_vel;
}

/////////////////////////////////////////////////////////////
SmoothLanding::SmoothLanding(double acc_time, const std::array<std::array<double, 3>, 3> p0)
: tf_(acc_time){
  xf_ = p0[0];
  vf_ = p0[1];
  af_ = p0[2];
  getMotionParameter();
}

void SmoothLanding::getMotionParameter() {
  int i = 0;

  // cubic
  for (i=0; i<2; i++){
    coeff_[i] = solveCubicZero(vf_[i], af_[i], tf_);  
  } 
  // get x0
  for (i=0; i<2; i++) {
    x0_[i] = xf_[i] - (coeff_[i][0]/4.0*std::pow(tf_, 4) + coeff_[i][1]/3.0*std::pow(tf_, 3));  
  }
  x0_[2] = xf_[2];
}

void SmoothLanding::printMotionParameter() {
  std::cout << std::endl << "x0_:";
  for (const auto& a:x0_)
    std::cout << a << ",";

  std::cout << std::endl << "coeff_:";
  for (const auto& a:coeff_){
    for (const auto& b:a){
      std::cout << b << ",";
    }
    std::cout << std::endl;
  }
    

  std::cout << "==========================";

  std::cout << std::endl << "xf_:";
  for (const auto& a:xf_)
    std::cout << a << ",";

  std::cout << std::endl << "vf_:";
  for (const auto& a:vf_)
    std::cout << a << ",";

  std::cout << std::endl << "af_:";
  for (const auto& a:af_)
    std::cout << a << ",";
  std::cout << std::endl;
}


std::array<double, 3> SmoothLanding::getDesiredPose(double t) const {
  std::array<double, 3> desired_pt;
  int i = 0;
  
  // desired x, y value wrt base_frame
  for (i=0; i<3; i++){
    desired_pt[i] = x0_[i] + coeff_[i][0]/4.0*std::pow(t, 4) + coeff_[i][1]/3.0*std::pow(t, 3);
  }
  desired_pt[2] = x0_[2];
  
  return desired_pt;
}

std::array<double, 3> SmoothLanding::getDesiredVel(double t) const {
  std::array<double, 3> desired_vel;
  int i = 0;
  
  // desired x, y value wrt base_frame
  for (i=0; i<3; i++){
    desired_vel[i] = coeff_[i][0]*std::pow(t, 3) + coeff_[i][1]*std::pow(t, 2);
  }
  desired_vel[2] = 0.0;
  
  return desired_vel;
}

/////////////////////////////////////////////////////////////
CircularLanding::CircularLanding(double acc_time, double r, const char dir[], const std::array<std::array<double, 3>,3> p0, bool landing)
: tf_(acc_time), r_(r), dir_(dir), landing_(landing) {
  xp_ = p0[0];
  vp_ = p0[1];
  ap_ = p0[2];
  getMotionParameter();
  // if (dir[1] == 'w')
  //   std::cout << "cw" << std::endl;
  // else if (dir[1] == 'c')
  //   std::cout << "ccw" << std::endl;
}

void CircularLanding::getMotionParameter() {
  double alpha, beta;
  double th_l;
  int i = 0;

  vm_ = std::sqrt(vp_[0]*vp_[0] + vp_[1]*vp_[1]);
  alpha = std::atan2(vp_[1], vp_[0]);

  // calculate alpha, beta
  if (landing_) {
    if (alpha < -M_PI_2) {
      if (dir_[1] == 'w'){
        beta = M_PI_2*5 + alpha;
      }
      else if (dir_[1] == 'c'){
        beta = M_PI_2*3 + alpha;
      }
    }
    else if (alpha < 0) {
      if (dir_[1] == 'w'){
        beta = M_PI_2 + alpha;
      }
      else if (dir_[1] == 'c'){
        beta = M_PI_2*3 + alpha;
      } 
    }
    else if (alpha < M_PI_2) {
      if (dir_[1] == 'w'){
        beta = M_PI_2 + alpha;
      }
      else if (dir_[1] == 'c'){
        beta = M_PI_2*3 + alpha;
      } 
    }
    else {
      if (dir_[1] == 'w'){
        beta = M_PI_2 + alpha;
      }
      else if (dir_[1] == 'c'){
        beta = -M_PI_2 + alpha;
      } 
    }

    // get xc_(center of circle)
    xc_[0] = xp_[0] - r_*std::cos(beta);  
    xc_[1] = xp_[1] - r_*std::sin(beta);  
    xc_[2] = xp_[2];

    // get th0_
    th_l = vm_*tf_/(2*r_);
    if (dir_[1] == 'w'){
      th0_ = beta + th_l;
    }
    else if (dir_[1] == 'c'){
      th0_ = beta - th_l; 
    }
  }

  // taking off motion
  else {
    // get th0_
    if (dir_[1] == 'w') {
      th0_ = M_PI_2 + alpha;
    }
    else if (dir_[1] == 'c') {
      th0_ = -M_PI_2 + alpha;
    }

    // get xc_(center of circle)
    xc_[0] = xp_[0] - r_*std::cos(th0_);  
    xc_[1] = xp_[1] - r_*std::sin(th0_);  
    xc_[2] = xp_[2];
  }
}

void CircularLanding::printMotionParameter() {
  std::cout << std::endl << "vm_:" << vm_;

  std::cout << std::endl << "xc_:";
  for (const auto& a:xc_)
    std::cout << a << ",";

  std::cout << std::endl << "th0_:" << th0_;

  std::cout << std::endl << "==========================";

  std::cout << std::endl << "xp_:";
  for (const auto& a:xp_)
    std::cout << a << ",";

  std::cout << std::endl << "vp_:";
  for (const auto& a:vp_)
    std::cout << a << ",";

  std::cout << std::endl << "ap_:";
  for (const auto& a:ap_)
    std::cout << a << ",";
  std::cout << std::endl;
}

std::array<double, 3> CircularLanding::getDesiredPose(double t) const {
  std::array<double, 3> desired_pt;
  double d_th, th;
  int i = 0;
  
  if (landing_) {
    d_th = vm_/(r_*tf_)*(t*t/2.0);
  }
  else {
    d_th = -vm_/(r_*tf_)*(std::pow((t-tf_), 2)/2.0) + vm_*tf_/(r_*2.0);
  }
  
  if (dir_[1] == 'w'){
    th = th0_ - d_th;
  }
  else if (dir_[1] == 'c'){
    th = th0_ + d_th; 
  }

  // desired x, y value wrt base_frame
  desired_pt[0] = xc_[0] + r_*std::cos(th);
  desired_pt[1] = xc_[1] + r_*std::sin(th);
  desired_pt[2] = xc_[2];
  
  return desired_pt;
}

std::array<double, 3> CircularLanding::getDesiredVel(double t) const {
  std::array<double, 3> desired_vel;
  double d_th, d_thd, th;
  double thd;
  int i = 0;
  
  if (landing_) {
    d_th = vm_/(r_*tf_)*(t*t/2.0);
    d_thd = vm_/(r_*tf_)*t;
  }
  else {
    d_th = -vm_/(r_*tf_)*(std::pow((t-tf_), 2)/2.0) + vm_*tf_/(r_*2.0);
    d_thd = -vm_/(r_*tf_)*(t-tf_);
  }

  if (dir_[1] == 'w'){
    th = th0_ - d_th;
    d_thd = -d_thd;
  }
  else if (dir_[1] == 'c'){
    th = th0_ + d_th;
  }

  // desired x, y value wrt base_frame
  desired_vel[0] = -r_*std::sin(th)*d_thd;
  desired_vel[1] = r_*std::cos(th)*d_thd;
  desired_vel[2] = 0.0;
  
  return desired_vel;
}

void CircularLanding::applyTransform(const std::array<double, 3> dref) {
  xc_[0] = xc_[0] + dref[0];
}

void CircularLanding::applyRefTransform(const std::array<double, 3> ref) {
  for (int i=0; i<3; i++) xp_[i] = ref[i];
  getMotionParameter();
}