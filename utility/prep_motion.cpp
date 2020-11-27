#include <iostream>
#include <fstream>
#include "prep_motion.h"

#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <cmath>

#include <franka/exception.h>
#include <franka/robot.h>


/////////////////////////////////////////////////////////////////////////////////
CircularPrep::CircularPrep(double r, double w, double th0, double thf, const std::array<double, 3> ref)
    : _r(r), _w(w), _th0(th0), _thf(thf) {
      _runtime = (thf - th0)/w;
      _ref = {ref[0]-r, ref[1], ref[2]}; // O -> P
    }

double CircularPrep::getRuntime() {return _runtime;}

std::array<std::array<double, 3>, 3> CircularPrep::getMotionParameter() {
  std::array<std::array<double, 3>, 3> prep_start;
  std::array<double, 3> x0;
  std::array<double, 3> v0;
  std::array<double, 3> a0;
  double dth = _w;

  x0[0] = _r*(1-std::cos(_th0)) + _ref[0];
  x0[1] = _r*std::sin(_th0) + _ref[1];
  x0[2] = 0.0 + _ref[2];
  
  v0[0] = _r*std::sin(_th0)*dth;
  v0[1] = _r*std::cos(_th0)*dth;
  v0[2] = 0.0;
  
  a0[0] = _r*std::cos(_th0)*dth*dth;
  a0[1] = -_r*std::sin(_th0)*dth*dth;
  a0[2] = 0.0;

  prep_start[0] = x0;
  prep_start[1] = v0;
  prep_start[2] = a0;

  return prep_start;
}

std::array<std::array<double, 3>, 3> CircularPrep::getPrepStart() {
  std::array<std::array<double, 3>, 3> prep_start;
  std::array<double, 3> x0;
  std::array<double, 3> v0;
  std::array<double, 3> a0;
  double dth = _w;

  x0[0] = _r*(1-std::cos(_th0)) + _ref[0];
  x0[1] = _r*std::sin(_th0) + _ref[1];
  x0[2] = 0.0 + _ref[2];
  
  v0[0] = _r*std::sin(_th0)*dth;
  v0[1] = _r*std::cos(_th0)*dth;
  v0[2] = 0.0;
  
  a0[0] = _r*std::cos(_th0)*dth*dth;
  a0[1] = -_r*std::sin(_th0)*dth*dth;
  a0[2] = 0.0;

  prep_start[0] = x0;
  prep_start[1] = v0;
  prep_start[2] = a0;

  return prep_start;
}

std::array<std::array<double, 3>, 3> CircularPrep::getPrepEnd() {
  std::array<std::array<double, 3>, 3> prep_end;
  std::array<double, 3> xf;
  std::array<double, 3> vf;
  std::array<double, 3> af;
  double dth = _w;

  xf[0] = _r*(1-std::cos(_thf)) + _ref[0];
  xf[1] = _r*std::sin(_thf) + _ref[1];
  xf[2] = 0.0 + _ref[2];
  
  vf[0] = _r*std::sin(_thf)*dth;
  vf[1] = _r*std::cos(_thf)*dth;
  vf[2] = 0.0;
  
  af[0] = _r*std::cos(_thf)*dth*dth;
  af[1] = -_r*std::sin(_thf)*dth*dth;
  af[2] = 0.0;

  prep_end[0] = xf;
  prep_end[1] = vf;
  prep_end[2] = af;

  return prep_end;
}

std::array<double, 3> CircularPrep::getDesiredPose(double t) const {
  std::array<double, 3> desired_pt;
  double th;

  th = _th0 + _w*t;
  desired_pt[0] = _r*(1-std::cos(th)) + _ref[0];
  desired_pt[1] = _r*std::sin(th) + _ref[1];
  desired_pt[2] = _ref[2];

  return desired_pt;
}

std::array<double, 3> CircularPrep::getDesiredVel(double t) const {
  std::array<double, 3> desired_vel;
  double th, dth;

  th = _th0 + _w*t;
  dth = _w;

  desired_vel[0] = _r*std::sin(th)*dth;
  desired_vel[1] = _r*std::cos(th)*dth;
  desired_vel[2] = 0.0;

  return desired_vel;
}

std::array<double, 3> CircularPrep::getDesiredAcc(double t) const {
  std::array<double, 3> desired_acc;
  double th, dth, ddth;

  th = _th0 + _w*t;
  dth = _w;
  ddth = 0;

  desired_acc[0] = _r*std::cos(th)*dth + _r*std::cos(th)*ddth;
  desired_acc[1] = -_r*std::sin(th)*dth + _r*std::cos(th)*ddth;
  desired_acc[2] = 0.0;

  return desired_acc;
}

void CircularPrep::applyRefTransform(const std::array<double, 3> ref){
  _ref = {ref[0]-_r, ref[1], ref[2]};
}

/////////////////////////////////////////////////////////////////////////////////

SplinePrep::SplinePrep(std::string file_path, double vel, const std::array<double, 3> ref)
{
  _ref = {ref[0], ref[1], ref[2]};

  prepareData(file_path, vel, &_xs, &_dts, &_ts);
  mySpline(_dts, _ts, _xs.col(0), 0.0, 0.0, &_ax);
  mySpline(_dts, _ts, _xs.col(1), 0.0, 0.0, &_ay);

  _runtime = std::round(_ts.tail(1)[0]*1000.0)/1000.0;
}

double SplinePrep::getRuntime() {return _runtime;}

std::array<std::array<double, 3>, 3> SplinePrep::getPrepStart() {
  std::array<std::array<double, 3>, 3> prep_start;
  std::array<double, 3> x0;
  std::array<double, 3> v0;
  std::array<double, 3> a0;
  
  double t0 = _ts.head(1)[0];

  x0 = getDesiredPose(t0);
  v0 = getDesiredVel(t0);
  a0 = getDesiredAcc(t0);
  
  std::cout << "t0=" << t0 << std::endl;
  std::cout << "x0=" << x0[0] << x0[1] << x0[2] << std::endl;
  std::cout << "v0=" << v0[0] << v0[1] << v0[2] << std::endl;
  std::cout << "a0=" << a0[0] << a0[1] << a0[2] << std::endl;

  prep_start[0] = x0;
  prep_start[1] = v0;
  prep_start[2] = a0;

  return prep_start;
}

std::array<std::array<double, 3>, 3> SplinePrep::getPrepEnd() {
  std::array<std::array<double, 3>, 3> prep_end;
  std::array<double, 3> xN;
  std::array<double, 3> vN;
  std::array<double, 3> aN;
  
  double tN = _runtime;

  xN = getDesiredPose(tN);
  vN = getDesiredVel(tN);
  aN = getDesiredAcc(tN);
  
  std::cout << "tN=" << tN << std::endl;
  std::cout << "tN=" << xN[0] << xN[1] << xN[2] << std::endl;
  std::cout << "vN=" << vN[0] << vN[1] << vN[2] << std::endl;
  std::cout << "aN=" << aN[0] << aN[1] << aN[2] << std::endl;

  prep_end[0] = xN;
  prep_end[1] = vN;
  prep_end[2] = aN;

  return prep_end;
}

std::array<double, 3> SplinePrep::getDesiredPose(double t) const {
  std::array<double, 3> desired_pt;
  double tk;  
  int ik;  

  checkIdx(t, &tk, &ik);
  
  desired_pt[0] = _ax(ik, 0)*std::pow((t-tk), 3) + _ax(ik, 1)*std::pow((t-tk), 2) 
                  + _ax(ik, 2)*(t-tk) + _ax(ik, 3) + _ref[0];  
  desired_pt[1] = _ay(ik, 0)*std::pow((t-tk), 3) + _ay(ik, 1)*std::pow((t-tk), 2)  
                  + _ay(ik, 2)*(t-tk) + _ay(ik, 3) + _ref[1];
  desired_pt[2] = _ref[2];
  
  // std::cout << "tk=" << tk << ", ik=" << ik << std::endl;
  // std::cout << "_ax=" << _ax(ik, 1) << std::endl;
  // std::cout << "desired_pt=" << Eigen::Vector3d::Map(&desired_pt[0]) << std::endl;

  return desired_pt;
}

std::array<double, 3> SplinePrep::getDesiredVel(double t) const {
  std::array<double, 3> desired_vel;
  double tk;  
  int ik;  

  checkIdx(t, &tk, &ik);
  desired_vel[0] = 3*_ax(ik, 0)*std::pow((t-tk), 2) + 2*_ax(ik, 1)*(t-tk) + _ax(ik, 2);
  desired_vel[1] = 3*_ay(ik, 0)*std::pow((t-tk), 2) + 2*_ay(ik, 1)*(t-tk) + _ay(ik, 2);
  desired_vel[2] = 0.0;

  return desired_vel;
}

std::array<double, 3> SplinePrep::getDesiredAcc(double t) const {
  std::array<double, 3> desired_acc;
  double tk;  
  int ik;  

  checkIdx(t, &tk, &ik);
  desired_acc[0] = 6*_ax(ik, 0)*(t-tk) + 2*_ax(ik, 1);
  desired_acc[1] = 6*_ax(ik, 0)*(t-tk) + 2*_ay(ik, 1);
  desired_acc[2] = 0.0;

  return desired_acc;
}

void SplinePrep::checkIdx(double t, double *tk, int *ik) const{
  int N = _xs.rows();

  for (int i=1; i<= N-1; i++) {
    if (t < _ts(i)) {
      *tk = _ts(i-1);
      *ik = i-1;
      break;
    }
    else {
      *tk = _ts(i);
      *ik = i-1;
    }
  }
}

void SplinePrep::applyTransform(const std::array<double, 3> dref){
  for (int i=0; i<3; i++)
    _ref[i] = _ref[i] + dref[i];
}

void SplinePrep::applyRefTransform(const std::array<double, 3> ref){
  for (int i=0; i<3; i++)
    _ref[i] = ref[i];
}

// void SplinePrep::printMotionParameter() {
//   std::cout << prep_runtime << "s" << std::endl;
//   std::cout << "prep_start:" << std::endl;
//   for (int i=0; i<3; i++) {
//     for (int j=0; j<3; j++) {
//       std::cout << prep_start[i][j] << ",";    
//     }
//      std::cout << std::endl;
//   }
//   std::cout << "prep_end:" << std::endl;
//   for (int i=0; i<3; i++) {
//     for (int j=0; j<3; j++) {
//       std::cout << prep_end[i][j] << ",";    
//     }
//      std::cout << std::endl;
//   }
// }


/////////////////////////////////////////////////////////////////////////////////
MultipleSplinePrep::MultipleSplinePrep(std::array<std::string, NLayer> file_paths, double vel, 
                                      int vpts, double s_time, const std::array<double, 3> ref)
  :_ref(ref){

    for (int k=0; k<3; k++){
      prepareData(file_paths[k], vel, &(_xs_arr[k]), &(_dts_arr[k]), &(_ts_arr[k]));  
    }

    combineData(vpts, s_time, vel, _xs_arr, _dts_arr, _ts_arr, &_xs, &_dts, &_ts);

    mySpline(_dts, _ts, _xs.col(0), 0.0, 0.0, &_ax);
    mySpline(_dts, _ts, _xs.col(1), 0.0, 0.0, &_ay);
    mySpline(_dts, _ts, _xs.col(2), 0.0, 0.0, &_az);

    // std::cout << _az << std::endl;

    _runtime = std::round(_ts.tail(1)[0]*1000.0)/1000.0;
}

double MultipleSplinePrep::getRuntime() {return _runtime;}

std::array<std::array<double, 3>, 3> MultipleSplinePrep::getPrepStart() {
  std::array<std::array<double, 3>, 3> prep_start;
  std::array<double, 3> x0;
  std::array<double, 3> v0;
  std::array<double, 3> a0;
  
  double t0 = _ts.head(1)[0];

  x0 = getDesiredPose(t0);
  v0 = getDesiredVel(t0);
  a0 = getDesiredAcc(t0);
  
  std::cout << "t0=" << t0 << std::endl;
  std::cout << "x0=" << x0[0] << x0[1] << x0[2] << std::endl;
  std::cout << "v0=" << v0[0] << v0[1] << v0[2] << std::endl;
  std::cout << "a0=" << a0[0] << a0[1] << a0[2] << std::endl;

  prep_start[0] = x0;
  prep_start[1] = v0;
  prep_start[2] = a0;

  return prep_start;
}

std::array<std::array<double, 3>, 3> MultipleSplinePrep::getPrepEnd() {
  std::array<std::array<double, 3>, 3> prep_end;
  std::array<double, 3> xN;
  std::array<double, 3> vN;
  std::array<double, 3> aN;
  
  double tN = _runtime;

  xN = getDesiredPose(tN);
  vN = getDesiredVel(tN);
  aN = getDesiredAcc(tN);
  
  std::cout << "tN=" << tN << std::endl;
  std::cout << "tN=" << xN[0] << xN[1] << xN[2] << std::endl;
  std::cout << "vN=" << vN[0] << vN[1] << vN[2] << std::endl;
  std::cout << "aN=" << aN[0] << aN[1] << aN[2] << std::endl;

  prep_end[0] = xN;
  prep_end[1] = vN;
  prep_end[2] = aN;

  return prep_end;
}

std::array<double, 3> MultipleSplinePrep::getDesiredPose(double t) const {
  std::array<double, 3> desired_pt;
  double tk;  
  int ik;  

  checkIdx(t, &tk, &ik);
  
  desired_pt[0] = _ax(ik, 0)*std::pow((t-tk), 3) + _ax(ik, 1)*std::pow((t-tk), 2) 
                  + _ax(ik, 2)*(t-tk) + _ax(ik, 3) + _ref[0];  
  desired_pt[1] = _ay(ik, 0)*std::pow((t-tk), 3) + _ay(ik, 1)*std::pow((t-tk), 2)  
                  + _ay(ik, 2)*(t-tk) + _ay(ik, 3) + _ref[1];
  desired_pt[2] = _az(ik, 0)*std::pow((t-tk), 3) + _az(ik, 1)*std::pow((t-tk), 2)  
                  + _az(ik, 2)*(t-tk) + _az(ik, 3) + _ref[2];
  
  // std::cout << "tk=" << tk << ", ik=" << ik << std::endl;
  // std::cout << "_ax=" << _ax(ik, 1) << std::endl;
  // std::cout << "desired_pt=" << Eigen::Vector3d::Map(&desired_pt[0]) << std::endl;

  return desired_pt;
}

std::array<double, 3> MultipleSplinePrep::getDesiredVel(double t) const {
  std::array<double, 3> desired_vel;
  double tk;  
  int ik;  

  checkIdx(t, &tk, &ik);
  desired_vel[0] = 3*_ax(ik, 0)*std::pow((t-tk), 2) + 2*_ax(ik, 1)*(t-tk) + _ax(ik, 2);
  desired_vel[1] = 3*_ay(ik, 0)*std::pow((t-tk), 2) + 2*_ay(ik, 1)*(t-tk) + _ay(ik, 2);
  desired_vel[2] = 3*_az(ik, 0)*std::pow((t-tk), 2) + 2*_az(ik, 1)*(t-tk) + _az(ik, 2);

  return desired_vel;
}

std::array<double, 3> MultipleSplinePrep::getDesiredAcc(double t) const {
  std::array<double, 3> desired_acc;
  double tk;  
  int ik;  

  checkIdx(t, &tk, &ik);
  desired_acc[0] = 6*_ax(ik, 0)*(t-tk) + 2*_ax(ik, 1);
  desired_acc[1] = 6*_ay(ik, 0)*(t-tk) + 2*_ay(ik, 1);
  desired_acc[2] = 6*_az(ik, 0)*(t-tk) + 2*_az(ik, 1);

  return desired_acc;
}

void MultipleSplinePrep::checkIdx(double t, double *tk, int *ik) const{
  int N = _xs.rows();

  for (int i=1; i<= N-1; i++) {
    if (t < _ts(i)) {
      *tk = _ts(i-1);
      *ik = i-1;
      break;
    }
    else {
      *tk = _ts(i);
      *ik = i-1;
    }
  }
}


/////////////////////////////////////////////////////////////////////////////////
//////////////////////////////    function     //////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void prepareData(std::string file_path, double vel, Eigen::MatrixX3d* xs, \
                 Eigen::VectorXd* dts, Eigen::VectorXd* ts) {
  
  // read xs from csv
  std::ifstream ap(file_path);
  std::vector<std::array<double, 3>> via_points;
  int num_of_vpt;
  double dist;

  if (!ap.is_open()){
    std::cout << "ERROR: Failed to Open File" << std::endl; 
  }
  else{
    std::string x;
    std::string y;
    std::string z;
    while(ap.good()){
      getline(ap, x, ',');
      getline(ap, y, ',');
      getline(ap, z, '\n');
      via_points.push_back({std::stod(x)/1000, std::stod(y)/1000, std::stod(z)/1000});
    }
  }
  
  // put via points vector into matrix  
  num_of_vpt = via_points.size();
  (*xs).resize(num_of_vpt, 3);
  for (int i=0; i<num_of_vpt; i++){
    (*xs).row(i) << Eigen::RowVector3d::Map(&via_points[i][0]);
  }

  // calculate ts
  (*ts).setZero(num_of_vpt);
  (*dts).setZero(num_of_vpt-1);
  for (int i=1; i<=num_of_vpt-1; i++){
    dist = ((*xs).row(i)-(*xs).row(i-1)).norm();
    (*dts)(i-1) = dist/vel;
    (*ts)(i) = ((*dts).segment(0, i)).sum();
  }
  // std::cout << "#" << num_of_vpt << std::endl;
}

// combineData(num_of_vpt, sliding_time, _xs1, _ts1, &xss, &dtss, &tss);
void combineData(int sliding_vpts, double sliding_time, double sliding_vel, std::array<Eigen::MatrixX3d, NLayer> &xs_arr, 
              std::array<Eigen::VectorXd, NLayer> &dts_arr, std::array<Eigen::VectorXd, NLayer> &ts_arr,
              Eigen::MatrixX3d* xss, Eigen::VectorXd* dtss, Eigen::VectorXd* tss) {

  Eigen::VectorXi ns(NLayer);
  std::array<Eigen::Vector3d, NLayer-1> vel_end;
  std::array<Eigen::Vector3d, NLayer-1> vel_start;

  // 
  ns << xs_arr[0].rows(), xs_arr[1].rows(), xs_arr[2].rows();
  std::cout << "N1=" << ns[0] << ", N2=" << ns[1] << ", N3=" << ns[2] << std::endl;

  for (int k=0; k<NLayer-1; k++) {
    vel_end[k] = (xs_arr[k].row(ns[k]-1)-xs_arr[k].row(ns[k]-2))*(1/dts_arr[k](ns[k]-2));
    vel_start[k] = (xs_arr[k+1].row(1)-xs_arr[k+1].row(0))*(1/dts_arr[k](0));
  }
  // std::cout << "vel_end" << k << ":" <<  vel_end[k] << std::endl;
  // std::cout << "vel_start" << k << ":" << vel_start[k] << std::endl;

  // Calculate coefs for CubicPath
  std::array<Eigen::MatrixX4d, NLayer-1> coefs;
  for (int k=0; k<NLayer-1; k++) {
    coefs[k].resize(3, 4);
    for (int j=0; j<3; j++){
      coefs[k].row(j) = solveCubic(0.0, sliding_time, xs_arr[k](ns[k]-1, j), xs_arr[k+1](0, j), vel_end[k](j), vel_start[k](j));
    }
    // std::cout << coefs[k] << std::endl;
  }

  // sampling
  double dr = sliding_time/sliding_vpts;
  double r = 0.0;
  double dist = 0.0;
  std::array<Eigen::MatrixX3d, NLayer-1> xs_sliding;
  std::array<Eigen::VectorXd, NLayer-1> dts_sliding;
  std::array<Eigen::VectorXd, NLayer-1> ts_sliding;
  for (int k=0; k<NLayer-1; k++) {
    xs_sliding[k].resize(sliding_vpts+1, 3);
    ts_sliding[k].setZero(sliding_vpts+1);
    dts_sliding[k].setZero(sliding_vpts);
    xs_sliding[k].row(0) = xs_arr[k].row(ns[k]-1);
    for (int i=1; i<sliding_vpts+1; i++) {
      r = r + dr;
      for (int j=0; j<3; j++) {
        xs_sliding[k](i, j) = coefs[k](j, 0)*std::pow(r, 3) + coefs[k](j, 1)*std::pow(r, 2) + coefs[k](j, 2)*r + coefs[k](j, 3);
      }
      dist = (xs_sliding[k].row(i)-xs_sliding[k].row(i-1)).norm();
      dts_sliding[k](i-1) = dist/sliding_vel; 
      ts_sliding[k](i) = (dts_sliding[k].segment(0, i)).sum();
    }
    r = 0.0;
  }

  for (int k=0; k<NLayer-1; k++) {
    for (int i=0; i<sliding_vpts+1; i++)
      ts_sliding[k](i) = ts_sliding[k](i) + ts_arr[k](ns[k]-1);
    for (int i=0; i<ns[k]; i++)
      ts_arr[k+1](i) = ts_arr[k+1](i) + (ts_arr[k](ns[k]-1) + dts_sliding[k].sum());
  }

  // combine
  int N = (NLayer-1)*(sliding_vpts-1);
  N += ns.sum();
  std::cout << "N=" << N << std::endl;  

  (*tss).resize(N);
  (*dtss).resize(N-1);
  (*xss).resize(N, 3);
  
  // for (int k=0; k<NLayer; k++) {
    // (*tss).segment(ns.segment(0, k).sum()+k*(sliding_vpts-1), ns[k]) = ts_arr[k];
    // (*dtss).segment(ns.segment(0, k).sum()-k+k*sliding_vpts, ns[k]-1) = dts_arr[k];
    // if (k < NLayer-1)
      // (*tss).segment(ns.segment(0, k+1).sum()+(k+1)*(sliding_vpts-1), sliding_vpts-1) = ts_sliding[k].segment(1, sliding_vpts-1);
      // (*dtss).segment(ns.segment(0, k).sum()-k+(k-1)*sliding_vpts, sliding_vpts) = dts_sliding[k];
  // }

  // std::cout << (xs_arr[0]).row(0) << std::endl;
  (*xss).block(0, 0, ns[0], 3) = xs_arr[0];
  (*xss).block(ns[0], 0, sliding_vpts-1, 3) = xs_sliding[0].block(1, 0, sliding_vpts-1, 3);
  (*xss).block(ns[0]+sliding_vpts-1, 0, ns[1], 3) = xs_arr[1];
  (*xss).block(ns[0]+ns[1]+sliding_vpts-1, 0, sliding_vpts-1, 3) = xs_sliding[1].block(1, 0, sliding_vpts-1, 3);
  (*xss).block(ns[0]+ns[1]+2*(sliding_vpts-1), 0, ns[2], 3) = xs_arr[2];
  
  (*dtss).segment(0, ns[0]-1) = dts_arr[0];
  (*dtss).segment(ns[0]-1, sliding_vpts) = dts_sliding[0];
  (*dtss).segment(ns[0]-1+sliding_vpts, ns[1]-1) = dts_arr[1];
  (*dtss).segment(ns[0]-1+ns[1]-1+sliding_vpts, sliding_vpts) = dts_sliding[1];
  (*dtss).segment(ns[0]-1+ns[1]-1+2*sliding_vpts, ns[2]-1) = dts_arr[2];

  (*tss).segment(0, ns[0]) = ts_arr[0];
  (*tss).segment(ns[0], sliding_vpts-1) = ts_sliding[0].segment(1, sliding_vpts-1);
  (*tss).segment(ns[0]+sliding_vpts-1, ns[1]) = ts_arr[1];
  (*tss).segment(ns[0]+ns[1]+sliding_vpts-1, sliding_vpts-1) = ts_sliding[1].segment(1, sliding_vpts-1);  
  (*tss).segment(ns[0]+ns[1]+2*(sliding_vpts-1), ns[2]) = ts_arr[2];  

  std::cout << "===================" << std::endl;
  // std::cout << (*xss) << std::endl;

  
}

Eigen::Vector4d solveCubic(double t0, double tf, double x0, double xf, double v0, double vf) {
  Eigen::Matrix4d A(4, 4);
  Eigen::Vector4d b(4, 1);
  
  A << std::pow(tf, 3), std::pow(tf, 2), tf, 1.0,
       std::pow(t0, 3), std::pow(t0, 2), t0, 1.0,
       3*std::pow(tf, 2), 2*tf, 1.0, 0.0,
       3*std::pow(t0, 2), 2*t0, 1.0, 0.0;
  b << xf, x0, vf, v0;

  // std::cout << "A=" << std::endl << A << std::endl;
  // std::cout << "b=" << std::endl << b << std::endl;
  // std::cout << "x=" << Eigen::RowVector4d(A.colPivHouseholderQr().solve(b)) << std::endl;

  return Eigen::RowVector4d(A.colPivHouseholderQr().solve(b));
}

void mySpline(Eigen::VectorXd dts, Eigen::VectorXd ts, Eigen::VectorXd xs, 
              double xddot1, double xddotN, Eigen::MatrixX4d* coefs) {
  int N = xs.rows();
  // std::cout << "N=" << N << std::endl;

  //////////////////////////////////////////////////////////////
  // calculate A, b
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(N-2, N-2);
  Eigen::VectorXd b = Eigen::VectorXd::Zero(N-2);
  A(0, 0) = (dts(0) + dts(1))/3.0;
  A(0, 1) = dts(1)/6.0;
  b(0) = xs(0)/dts(0) - (1/dts(1) + 1/dts(0))*xs(1) + xs(2)/dts(1) - (dts(0)/6.0)*xddot1;

  A(N-3, N-4) = dts(N-3)/6.0;
  A(N-3, N-3) = (dts(N-3) + dts(N-2))/3.0;
  b(N-3) = xs(N-3)/dts(N-3) - (1/dts(N-2) + 1/dts(N-3))*xs(N-2) + xs(N-1)/dts(N-2) - (dts(N-2)/6.0)*xddotN;


  for (int k=1; k<=N-4; k++) {
    A(k, k-1) = dts(k)/6.0;
    A(k, k) = (dts(k) + dts(k+1))/3.0;
    A(k, k+1) = dts(k+1)/6.0;
    b(k) = xs(k)/dts(k) - (1/dts(k+1) + 1/dts(k))*xs(k+1) + xs(k+2)/dts(k+1);
  }
  // std::cout << "A=" << A << std::endl;
  // std::cout << "b=" << b << std::endl;

  //////////////////////////////////////////////////////////////
  // get TriDiagM
  int TN = (N-2) + 2; // N
  Eigen::VectorXd Ta = Eigen::VectorXd::Zero(TN-2); // N-2
  Eigen::VectorXd Tb = Eigen::VectorXd::Zero(TN-2); // N-2
  Eigen::VectorXd Tc = Eigen::VectorXd::Zero(TN-3); // N-3
  Eigen::VectorXd Td = b;                           // N-2
  for (int k=0; k<=TN-3; k++) {
    if (k > 0)
      Ta(k) = A(k, k-1);
    Tb(k) = A(k, k);
    if (k < TN-3)
      Tc(k) = A(k, k+1);
  }
  // std::cout << "Ta=" << Ta << std::endl;
  // std::cout << "Tb=" << Tb << std::endl;
  // std::cout << "Tc=" << Tc << std::endl;
  // std::cout << "Td=" << Td << std::endl;

  //////////////////////////////////////////////////////////////
  // solve TriDiagM
  int TN_ = TN-2; // N-2
  Eigen::VectorXd Tc_ = Eigen::VectorXd::Zero(TN_-1);    // N-3
  Eigen::VectorXd Td_ = Eigen::VectorXd::Zero(TN_);      // N-2
  Eigen::VectorXd Tx = Eigen::VectorXd::Zero(TN_);       // N-2
  Eigen::VectorXd xddots = Eigen::VectorXd::Zero(TN_+2); // N
  // Forward Computation
  Tc_(0) = Tc(0)/Tb(0);
  for (int i=1; i<=TN_-2; i++) {
    Tc_(i) = Tc(i) / (Tb(i)-Ta(i)*Tc_(i-1));
  }
  Td_(0) = Td(0)/Tb(0);
  for (int i=1; i<=TN_-1; i++) {
    Td_(i) = (Td(i)-Ta(i)*Td_(i-1))/(Tb(i)-Ta(i)*Tc_(i-1));
  }
  // Backward Computation
  Tx(TN_-1) = Td_(TN_-1);
  for (int i=TN_-2; i>=0; i--) {
    Tx(i) = Td_(i) - Tc_(i)*Tx(i+1);
  }
  // xddots
  xddots(0) = xddot1;
  for (int i=1; i<= N-2; i++){
    xddots(i) = Tx(i-1);
  }
  xddots(N-1) = xddotN;
  // std::cout << "Tc_=" << Tc_ << std::endl;
  // std::cout << "Td_=" << Td_ << std::endl;
  // std::cout << "Tx=" << Tx << std::endl;
  // std::cout << "xddots=" << xddots << std::endl;

  //////////////////////////////////////////////////////////////
  // calculate a1, a2
  int CN = TN_+2; // N
  (*coefs).setZero(CN-1, 4); // N-1, 4
  for (int k=0; k<=CN-2; k++) {
    (*coefs)(k, 0) = (xddots(k+1)-xddots(k))/(6.0*dts(k));
    (*coefs)(k, 1) = xddots(k)/2.0;
    (*coefs)(k, 2) = (xs(k+1)-xs(k))/dts(k) - (xddots(k+1)+2*xddots(k))*dts(k)/6.0;
    (*coefs)(k, 3) = xs(k);
  }

}