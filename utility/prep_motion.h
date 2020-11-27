// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>

#include <Eigen/Dense>

#include <franka/control_types.h>
#include <franka/duration.h>
#include <franka/robot.h>
#include <franka/robot_state.h>

#define DT 0.001
#define NLayer 3

class CircularPrep {

 public:
  CircularPrep(double r, double w, double th0, double thf, const std::array<double, 3> ref);
  double getRuntime();
  std::array<std::array<double, 3>, 3> getMotionParameter();
  std::array<std::array<double, 3>, 3> getPrepStart();
  std::array<std::array<double, 3>, 3> getPrepEnd();
  std::array<double, 3> getDesiredPose(double t) const;
  std::array<double, 3> getDesiredVel(double t) const;
  std::array<double, 3> getDesiredAcc(double t) const;
  void printMotionParameter();
  void applyRefTransform(std::array<double, 3> ref);

 private:
  std::array<double, 3> _ref;
  const double _r;
  const double _w;
  const double _th0;
  const double _thf;

  double _runtime;

};

class SplinePrep {

 public:
  SplinePrep(std::string file_path, double vel, const std::array<double, 3> ref);
  double getRuntime();
  std::array<std::array<double, 3>, 3> getPrepStart();
  std::array<std::array<double, 3>, 3> getPrepEnd();
  std::array<double, 3> getDesiredPose(double t) const;
  std::array<double, 3> getDesiredVel(double t) const;
  std::array<double, 3> getDesiredAcc(double t) const;
  void checkIdx(double t, double *tk, int *ik) const;
  void printMotionParameter();
  void applyTransform(std::array<double, 3> dref);
  void applyRefTransform(std::array<double, 3> ref);

 private:
  std::array<double, 3> _ref;
  
  Eigen::VectorXd _dts;
  Eigen::VectorXd _ts;
  Eigen::MatrixX3d _xs;
  Eigen::MatrixX4d _ax;
  Eigen::MatrixX4d _ay;

  double _runtime;

};

class MultipleSplinePrep {

 public:
  MultipleSplinePrep(std::array<std::string, NLayer> file_paths, double vel, 
                      int vpts, double s_time, const std::array<double, 3> ref);
  double getRuntime();
  std::array<std::array<double, 3>, 3> getPrepStart();
  std::array<std::array<double, 3>, 3> getPrepEnd();
  std::array<double, 3> getDesiredPose(double t) const;
  std::array<double, 3> getDesiredVel(double t) const;
  std::array<double, 3> getDesiredAcc(double t) const;
  void checkIdx(double t, double *tk, int *ik) const;
  void printMotionParameter();

 private:
  const std::array<double, 3> _ref;
  
  std::array<Eigen::VectorXd, NLayer> _dts_arr;
  std::array<Eigen::VectorXd, NLayer> _ts_arr;
  std::array<Eigen::MatrixX3d, NLayer> _xs_arr;

  Eigen::VectorXd _dts;
  Eigen::VectorXd _ts;
  Eigen::MatrixX3d _xs;
  Eigen::MatrixX4d _ax;
  Eigen::MatrixX4d _ay;
  Eigen::MatrixX4d _az;

  double _runtime;

};

void prepareData(std::string file_path, double vel, Eigen::MatrixX3d* xs, 
                 Eigen::VectorXd* dts, Eigen::VectorXd* ts);

void combineData(int num, double sliding_time, double sliding_vel, std::array<Eigen::MatrixX3d, 3> &xs_arr, 
                std::array<Eigen::VectorXd, 3> &dts_arr, std::array<Eigen::VectorXd, 3> &ts_arr,
                  Eigen::MatrixX3d* xss, Eigen::VectorXd* dtss, Eigen::VectorXd* tss);

Eigen::Vector4d solveCubic(double t0, double tf, double x0, double xf, double v0, double vf);

void mySpline(Eigen::VectorXd dts, Eigen::VectorXd ts, Eigen::VectorXd xs, 
              double xddot1, double xddotN, Eigen::MatrixX4d* coefs);