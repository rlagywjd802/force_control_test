#pragma once

#include <array>

#define DT 0.001

std::array<double, 2> solveCubicZero(double vf, double af, double tf);

/////////////////////////////////////////////////////////////
class LinearLanding {
 public:
  LinearLanding(double acc_time, const std::array<std::array<double, 3>,3> p0);

  void getMotionParameter();
  std::array<double, 3> getDesiredPose(double t) const;
  std::array<double, 3> getDesiredVel(double t) const;
  void printMotionParameter();

 private:
  const double tf_;
  std::array<double, 3> xf_;
  std::array<double, 3> vf_;
  std::array<double, 3> af_;

  std::array<double, 3> x0_;
  std::array<double, 2> coeff_;  
};

/////////////////////////////////////////////////////////////
class SmoothLanding {
 public:
  SmoothLanding(double acc_time, const std::array<std::array<double, 3>,3> p0);

  void getMotionParameter();  
  std::array<double, 3> getDesiredPose(double t) const;
  std::array<double, 3> getDesiredVel(double t) const;
  void printMotionParameter();

 private:
  const double tf_;
  std::array<double, 3> xf_;
  std::array<double, 3> vf_;
  std::array<double, 3> af_;

  std::array<double, 3> x0_;
  std::array<std::array<double, 2>, 2> coeff_;  
};

/////////////////////////////////////////////////////////////
class CircularLanding {
 public:
  CircularLanding(double acc_time, double r, const char dir[], const std::array<std::array<double, 3>,3> p0, bool landing);

  void getMotionParameter();
  std::array<double, 3> getDesiredPose(double t) const;
  std::array<double, 3> getDesiredVel(double t) const;
  void printMotionParameter();
  void applyTransform(const std::array<double, 3> dref);
  void applyRefTransform(const std::array<double, 3> ref);

 private:
  const double tf_;
  const double r_;
  const char* dir_;
  const bool landing_;
  std::array<double, 3> xp_;
  std::array<double, 3> vp_;
  std::array<double, 3> ap_;

  double th0_;
  double vm_;
  std::array<double, 3> xc_;
};