// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include "examples_common.h"

#include <algorithm>
#include <array>
#include <cmath>

#include <franka/exception.h>
#include <franka/robot.h>

void setDefaultBehavior(franka::Robot& robot) {
  robot.setCollisionBehavior(
      {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
      {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
      {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
      {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}});
  robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
  robot.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
}

MotionGenerator::MotionGenerator(double speed_factor, const std::array<double, 7> q_goal)
    : q_goal_(q_goal.data()) {
  dq_max_ *= speed_factor;
  ddq_max_start_ *= speed_factor;
  ddq_max_goal_ *= speed_factor;
  dq_max_sync_.setZero();
  q_start_.setZero();
  delta_q_.setZero();
  t_1_sync_.setZero();
  t_2_sync_.setZero();
  t_f_sync_.setZero();
  q_1_.setZero();
}

bool MotionGenerator::calculateDesiredValues(double t, Vector7d* delta_q_d) const {
  Vector7i sign_delta_q;
  sign_delta_q << delta_q_.cwiseSign().cast<int>();
  Vector7d t_d = t_2_sync_ - t_1_sync_;
  Vector7d delta_t_2_sync = t_f_sync_ - t_2_sync_;
  std::array<bool, 7> joint_motion_finished{};

  for (size_t i = 0; i < 7; i++) {
    if (std::abs(delta_q_[i]) < kDeltaQMotionFinished) {
      (*delta_q_d)[i] = 0;
      joint_motion_finished[i] = true;
    } else {
      if (t < t_1_sync_[i]) {
        (*delta_q_d)[i] = -1.0 / std::pow(t_1_sync_[i], 3.0) * dq_max_sync_[i] * sign_delta_q[i] *
                          (0.5 * t - t_1_sync_[i]) * std::pow(t, 3.0);
      } else if (t >= t_1_sync_[i] && t < t_2_sync_[i]) {
        (*delta_q_d)[i] = q_1_[i] + (t - t_1_sync_[i]) * dq_max_sync_[i] * sign_delta_q[i];
      } else if (t >= t_2_sync_[i] && t < t_f_sync_[i]) {
        (*delta_q_d)[i] =
            delta_q_[i] + 0.5 *
                              (1.0 / std::pow(delta_t_2_sync[i], 3.0) *
                                   (t - t_1_sync_[i] - 2.0 * delta_t_2_sync[i] - t_d[i]) *
                                   std::pow((t - t_1_sync_[i] - t_d[i]), 3.0) +
                               (2.0 * t - 2.0 * t_1_sync_[i] - delta_t_2_sync[i] - 2.0 * t_d[i])) *
                              dq_max_sync_[i] * sign_delta_q[i];
      } else {
        (*delta_q_d)[i] = delta_q_[i];
        joint_motion_finished[i] = true;
      }
    }
  }
  return std::all_of(joint_motion_finished.cbegin(), joint_motion_finished.cend(),
                     [](bool x) { return x; });
}

void MotionGenerator::calculateSynchronizedValues() {
  Vector7d dq_max_reach(dq_max_);
  Vector7d t_f = Vector7d::Zero();
  Vector7d delta_t_2 = Vector7d::Zero();
  Vector7d t_1 = Vector7d::Zero();
  Vector7d delta_t_2_sync = Vector7d::Zero();
  Vector7i sign_delta_q;
  sign_delta_q << delta_q_.cwiseSign().cast<int>();

  for (size_t i = 0; i < 7; i++) {
    if (std::abs(delta_q_[i]) > kDeltaQMotionFinished) {
      if (std::abs(delta_q_[i]) < (3.0 / 4.0 * (std::pow(dq_max_[i], 2.0) / ddq_max_start_[i]) +
                                   3.0 / 4.0 * (std::pow(dq_max_[i], 2.0) / ddq_max_goal_[i]))) {
        dq_max_reach[i] = std::sqrt(4.0 / 3.0 * delta_q_[i] * sign_delta_q[i] *
                                    (ddq_max_start_[i] * ddq_max_goal_[i]) /
                                    (ddq_max_start_[i] + ddq_max_goal_[i]));
      }
      t_1[i] = 1.5 * dq_max_reach[i] / ddq_max_start_[i];
      delta_t_2[i] = 1.5 * dq_max_reach[i] / ddq_max_goal_[i];
      t_f[i] = t_1[i] / 2.0 + delta_t_2[i] / 2.0 + std::abs(delta_q_[i]) / dq_max_reach[i];
    }
  }
  double max_t_f = t_f.maxCoeff();
  for (size_t i = 0; i < 7; i++) {
    if (std::abs(delta_q_[i]) > kDeltaQMotionFinished) {
      double a = 1.5 / 2.0 * (ddq_max_goal_[i] + ddq_max_start_[i]);
      double b = -1.0 * max_t_f * ddq_max_goal_[i] * ddq_max_start_[i];
      double c = std::abs(delta_q_[i]) * ddq_max_goal_[i] * ddq_max_start_[i];
      double delta = b * b - 4.0 * a * c;
      if (delta < 0.0) {
        delta = 0.0;
      }
      dq_max_sync_[i] = (-1.0 * b - std::sqrt(delta)) / (2.0 * a);
      t_1_sync_[i] = 1.5 * dq_max_sync_[i] / ddq_max_start_[i];
      delta_t_2_sync[i] = 1.5 * dq_max_sync_[i] / ddq_max_goal_[i];
      t_f_sync_[i] =
          (t_1_sync_)[i] / 2.0 + delta_t_2_sync[i] / 2.0 + std::abs(delta_q_[i] / dq_max_sync_[i]);
      t_2_sync_[i] = (t_f_sync_)[i] - delta_t_2_sync[i];
      q_1_[i] = (dq_max_sync_)[i] * sign_delta_q[i] * (0.5 * (t_1_sync_)[i]);
    }
  }
}

franka::JointPositions MotionGenerator::operator()(const franka::RobotState& robot_state,
                                                   franka::Duration period) {
  time_ += period.toSec();

  if (time_ == 0.0) {
    q_start_ = Vector7d(robot_state.q_d.data());
    delta_q_ = q_goal_ - q_start_;
    calculateSynchronizedValues();
  }

  Vector7d delta_q_d;
  bool motion_finished = calculateDesiredValues(time_, &delta_q_d);

  std::array<double, 7> joint_positions;
  Eigen::VectorXd::Map(&joint_positions[0], 7) = (q_start_ + delta_q_d);
  franka::JointPositions output(joint_positions);
  output.motion_finished = motion_finished;
  return output;
}

/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
CMotionGenerator::CMotionGenerator(double speed_factor, const std::array<double, 3> x_goal)
    : x_goal_(x_goal.data()) {
  dx_max_ *= speed_factor;
  ddx_max_start_ *= speed_factor;
  ddx_max_goal_ *= speed_factor;
  dx_max_sync_.setZero();
  x_start_.setZero();
  delta_x_.setZero();
  t_1_sync_.setZero();
  t_2_sync_.setZero();
  t_f_sync_.setZero();
  x_1_.setZero();
}

bool CMotionGenerator::calculateDesiredValues(double t, Vector3d* delta_x_d) const {
  Vector3i sign_delta_x;
  sign_delta_x << delta_x_.cwiseSign().cast<int>();
  Vector3d t_d = t_2_sync_ - t_1_sync_;
  Vector3d delta_t_2_sync = t_f_sync_ - t_2_sync_;
  std::array<bool, 3> cartesian_motion_finished{};

  for (size_t i = 0; i < 3; i++) {
    if (std::abs(delta_x_[i]) < kDeltaXMotionFinished) {
      (*delta_x_d)[i] = 0;
      cartesian_motion_finished[i] = true;
    } else {
      if (t < t_1_sync_[i]) {
        (*delta_x_d)[i] = -1.0 / std::pow(t_1_sync_[i], 3.0) * dx_max_sync_[i] * sign_delta_x[i] *
                          (0.5 * t - t_1_sync_[i]) * std::pow(t, 3.0);
      } else if (t >= t_1_sync_[i] && t < t_2_sync_[i]) {
        (*delta_x_d)[i] = x_1_[i] + (t - t_1_sync_[i]) * dx_max_sync_[i] * sign_delta_x[i];
      } else if (t >= t_2_sync_[i] && t < t_f_sync_[i]) {
        (*delta_x_d)[i] =
            delta_x_[i] + 0.5 *
                              (1.0 / std::pow(delta_t_2_sync[i], 3.0) *
                                   (t - t_1_sync_[i] - 2.0 * delta_t_2_sync[i] - t_d[i]) *
                                   std::pow((t - t_1_sync_[i] - t_d[i]), 3.0) +
                               (2.0 * t - 2.0 * t_1_sync_[i] - delta_t_2_sync[i] - 2.0 * t_d[i])) *
                              dx_max_sync_[i] * sign_delta_x[i];
      } else {
        (*delta_x_d)[i] = delta_x_[i];
        cartesian_motion_finished[i] = true;
      }
    }
  }
  return std::all_of(cartesian_motion_finished.cbegin(), cartesian_motion_finished.cend(),
                     [](bool x) { return x; });
}

void CMotionGenerator::calculateSynchronizedValues() {
  Vector3d dx_max_reach(dx_max_);
  Vector3d t_f = Vector3d::Zero();
  Vector3d delta_t_2 = Vector3d::Zero();
  Vector3d t_1 = Vector3d::Zero();
  Vector3d delta_t_2_sync = Vector3d::Zero();
  Vector3i sign_delta_x;
  sign_delta_x << delta_x_.cwiseSign().cast<int>();

  for (size_t i = 0; i < 3; i++) {
    if (std::abs(delta_x_[i]) > kDeltaXMotionFinished) {
      if (std::abs(delta_x_[i]) < (3.0 / 4.0 * (std::pow(dx_max_[i], 2.0) / ddx_max_start_[i]) +
                                   3.0 / 4.0 * (std::pow(dx_max_[i], 2.0) / ddx_max_goal_[i]))) {
        dx_max_reach[i] = std::sqrt(4.0 / 3.0 * delta_x_[i] * sign_delta_x[i] *
                                    (ddx_max_start_[i] * ddx_max_goal_[i]) /
                                    (ddx_max_start_[i] + ddx_max_goal_[i]));
      }
      t_1[i] = 1.5 * dx_max_reach[i] / ddx_max_start_[i];
      delta_t_2[i] = 1.5 * dx_max_reach[i] / ddx_max_goal_[i];
      t_f[i] = t_1[i] / 2.0 + delta_t_2[i] / 2.0 + std::abs(delta_x_[i]) / dx_max_reach[i];
    }
  }
  double max_t_f = t_f.maxCoeff();
  for (size_t i = 0; i < 3; i++) {
    if (std::abs(delta_x_[i]) > kDeltaXMotionFinished) {
      double a = 1.5 / 2.0 * (ddx_max_goal_[i] + ddx_max_start_[i]);
      double b = -1.0 * max_t_f * ddx_max_goal_[i] * ddx_max_start_[i];
      double c = std::abs(delta_x_[i]) * ddx_max_goal_[i] * ddx_max_start_[i];
      double delta = b * b - 4.0 * a * c;
      if (delta < 0.0) {
        delta = 0.0;
      }
      dx_max_sync_[i] = (-1.0 * b - std::sqrt(delta)) / (2.0 * a);
      t_1_sync_[i] = 1.5 * dx_max_sync_[i] / ddx_max_start_[i];
      delta_t_2_sync[i] = 1.5 * dx_max_sync_[i] / ddx_max_goal_[i];
      t_f_sync_[i] =
          (t_1_sync_)[i] / 2.0 + delta_t_2_sync[i] / 2.0 + std::abs(delta_x_[i] / dx_max_sync_[i]);
      t_2_sync_[i] = (t_f_sync_)[i] - delta_t_2_sync[i];
      x_1_[i] = (dx_max_sync_)[i] * sign_delta_x[i] * (0.5 * (t_1_sync_)[i]);
    }
  }
}

franka::CartesianPose CMotionGenerator::operator()(const franka::RobotState& robot_state,
                                                   franka::Duration period) {
  time_ += period.toSec();

  if (time_ == 0.0) {
    initial_pose = robot_state.O_T_EE_d;
    x_start_ = (Vector3d() << initial_pose[12], initial_pose[13], initial_pose[14]).finished();
    delta_x_ = x_goal_ - x_start_;
    calculateSynchronizedValues();
  }

  Vector3d delta_x_d;
  bool motion_finished = calculateDesiredValues(time_, &delta_x_d);

  std::array<double, 16> pose_desired = initial_pose;
  pose_desired[12] += delta_x_d[0];
  pose_desired[13] += delta_x_d[1];
  pose_desired[14] += delta_x_d[2];
  
  franka::CartesianPose output(pose_desired);
  output.motion_finished = motion_finished;
  return output;
}
/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
// CMotionGenerator::CMotionGenerator(double speed_factor, const std::array<double, 3> x_goal)
//     : x_goal_(x_goal.data()) {
//   dx_max_ *= speed_factor;
//   ddx_max_start_ *= speed_factor;
//   ddx_max_goal_ *= speed_factor;
//   dx_max_sync_.setZero();
//   x_start_.setZero();
//   delta_x_.setZero();
//   t_1_sync_.setZero();
//   t_2_sync_.setZero();
//   t_f_sync_.setZero();
//   x_1_.setZero();
// }

// bool CMotionGenerator::calculateDesiredValues(double t, Vector3d* delta_x_d) const {
//   Vector3i sign_delta_x;
//   sign_delta_x << delta_x_.cwiseSign().cast<int>();
//   Vector3d t_d = t_2_sync_ - t_1_sync_;
//   Vector3d delta_t_2_sync = t_f_sync_ - t_2_sync_;
//   std::array<bool, 3> cartesian_motion_finished{};

//   for (size_t i = 0; i < 3; i++) {
//     if (std::abs(delta_x_[i]) < kDeltaXMotionFinished) {
//       (*delta_x_d)[i] = 0;
//       cartesian_motion_finished[i] = true;
//     } else {
//       if (t < t_1_sync_[i]) {
//         (*delta_x_d)[i] = -1.0 / std::pow(t_1_sync_[i], 3.0) * dx_max_sync_[i] * sign_delta_x[i] *
//                           (0.5 * t - t_1_sync_[i]) * std::pow(t, 3.0);
//       } else if (t >= t_1_sync_[i] && t < t_2_sync_[i]) {
//         (*delta_x_d)[i] = x_1_[i] + (t - t_1_sync_[i]) * dx_max_sync_[i] * sign_delta_x[i];
//       } else if (t >= t_2_sync_[i] && t < t_f_sync_[i]) {
//         (*delta_x_d)[i] =
//             delta_x_[i] + 0.5 *
//                               (1.0 / std::pow(delta_t_2_sync[i], 3.0) *
//                                    (t - t_1_sync_[i] - 2.0 * delta_t_2_sync[i] - t_d[i]) *
//                                    std::pow((t - t_1_sync_[i] - t_d[i]), 3.0) +
//                                (2.0 * t - 2.0 * t_1_sync_[i] - delta_t_2_sync[i] - 2.0 * t_d[i])) *
//                               dx_max_sync_[i] * sign_delta_x[i];
//       } else {
//         (*delta_x_d)[i] = delta_x_[i];
//         cartesian_motion_finished[i] = true;
//       }
//     }
//   }
//   return std::all_of(cartesian_motion_finished.cbegin(), cartesian_motion_finished.cend(),
//                      [](bool x) { return x; });
// }

// void CMotionGenerator::calculateSynchronizedValues() {
//   Vector3d dx_max_reach(dx_max_);
//   Vector3d t_f = Vector3d::Zero();
//   Vector3d delta_t_2 = Vector3d::Zero();
//   Vector3d t_1 = Vector3d::Zero();
//   Vector3d delta_t_2_sync = Vector3d::Zero();
//   Vector3i sign_delta_x;
//   sign_delta_x << delta_x_.cwiseSign().cast<int>();

//   for (size_t i = 0; i < 3; i++) {
//     if (std::abs(delta_x_[i]) > kDeltaXMotionFinished) {
//       if (std::abs(delta_x_[i]) < (3.0 / 4.0 * (std::pow(dx_max_[i], 2.0) / ddx_max_start_[i]) +
//                                    3.0 / 4.0 * (std::pow(dx_max_[i], 2.0) / ddx_max_goal_[i]))) {
//         dx_max_reach[i] = std::sqrt(4.0 / 3.0 * delta_x_[i] * sign_delta_x[i] *
//                                     (ddx_max_start_[i] * ddx_max_goal_[i]) /
//                                     (ddx_max_start_[i] + ddx_max_goal_[i]));
//       }
//       t_1[i] = 1.5 * dx_max_reach[i] / ddx_max_start_[i];
//       delta_t_2[i] = 1.5 * dx_max_reach[i] / ddx_max_goal_[i];
//       t_f[i] = t_1[i] / 2.0 + delta_t_2[i] / 2.0 + std::abs(delta_x_[i]) / dx_max_reach[i];
//     }
//   }
//   double max_t_f = t_f.maxCoeff();
//   for (size_t i = 0; i < 3; i++) {
//     if (std::abs(delta_x_[i]) > kDeltaXMotionFinished) {
//       double a = 1.5 / 2.0 * (ddx_max_goal_[i] + ddx_max_start_[i]);
//       double b = -1.0 * max_t_f * ddx_max_goal_[i] * ddx_max_start_[i];
//       double c = std::abs(delta_x_[i]) * ddx_max_goal_[i] * ddx_max_start_[i];
//       double delta = b * b - 4.0 * a * c;
//       if (delta < 0.0) {
//         delta = 0.0;
//       }
//       dx_max_sync_[i] = (-1.0 * b - std::sqrt(delta)) / (2.0 * a);
//       t_1_sync_[i] = 1.5 * dx_max_sync_[i] / ddx_max_start_[i];
//       delta_t_2_sync[i] = 1.5 * dx_max_sync_[i] / ddx_max_goal_[i];
//       t_f_sync_[i] =
//           (t_1_sync_)[i] / 2.0 + delta_t_2_sync[i] / 2.0 + std::abs(delta_x_[i] / dx_max_sync_[i]);
//       t_2_sync_[i] = (t_f_sync_)[i] - delta_t_2_sync[i];
//       x_1_[i] = (dx_max_sync_)[i] * sign_delta_x[i] * (0.5 * (t_1_sync_)[i]);
//     }
//   }
// }

// franka::CartesianPose CMotionGenerator::operator()(const franka::RobotState& robot_state,
//                                                    franka::Duration period) {
//   time_ += period.toSec();

//   if (time_ == 0.0) {
//     initial_pose = robot_state.O_T_EE_d;
//     Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
//     Eigen::Quaterniond orientation(transform.linear());
    
//     // orientation error
//     // "difference" quaternion
//     Eigen::Matrix<double, 3, 1> error;
//     if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0) {
//       orientation.coeffs() << -orientation.coeffs();
//     }

//     // "difference" quaternion
//     Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d);
//     error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
    
//     // Transform to base frame
//     error.tail(3) << -transform.linear() * error.tail(3);
    
//     delta_x_ = x_goal_ - x_start_;
//     calculateSynchronizedValues();
//   }

//   Vector3d delta_x_d;
//   bool motion_finished = calculateDesiredValues(time_, &delta_x_d);

//   std::array<double, 16> pose_desired = initial_pose;
//   pose_desired[12] += delta_x_d[0];
//   pose_desired[13] += delta_x_d[1];
//   pose_desired[14] += delta_x_d[2];
  
//   franka::CartesianPose output(pose_desired);
//   output.motion_finished = motion_finished;
//   return output;
// }
