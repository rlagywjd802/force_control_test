#include <array>
#include <atomic>
#include <cmath>
#include <functional>
#include <iostream>
#include <iterator>
#include <mutex>
#include <thread>

#include <Eigen/Dense>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>

#include <ros/ros.h>
#include <icra18/Etc.h>

#include "examples_common.h"

#define FCI_IP "192.168.30.151"

namespace {
template <class T, size_t N>
std::ostream& operator<<(std::ostream& ostream, const std::array<T, N>& array) {
  ostream << "[";
  std::copy(array.cbegin(), array.cend() - 1, std::ostream_iterator<T>(ostream, ","));
  std::copy(array.cend() - 1, array.cend(), std::ostream_iterator<T>(ostream));
  ostream << "]";
  return ostream;
}
}  // anonymous namespace

int main(int argc, char** argv) {

  // Set and initialize trajectory parameters.
  const double radius = 0.02;
  const double vel_max = 0.01;
  const double acceleration_time = 2.0;
  const double run_time = 20.0;
  const double start_time = 5.0;

  // PI Gains for Torque Control
  const double k_p_ = 0.2;
  const double k_i_ = 1.0;
  const double desired_force_{1.0};

  // Set print rate for comparing commanded vs. measured torques.
  const double print_rate = 100.0;
  double vel_current = 0.0;
  double angle = 0.0;
  double time = 0.0;
  double time1 = 0.0;

  // Publisher
  ros::init(argc, argv, "force_control_test");
  ros::NodeHandle nh;
  ros::Publisher etc_pub = nh.advertise<icra18::Etc>("etc", 1);;


  // Initialize data fields for the print thread.
  struct {
    std::mutex mutex;
    bool has_data;
    std::array<double, 6> force_error;
    franka::RobotState robot_state;
  } print_data{};
  std::atomic_bool running{true};

  // Start print thread.
  std::thread print_thread([print_rate, &print_data, &running, etc_pub]() {
    while (running) {
      // Sleep to achieve the desired print rate.
      std::this_thread::sleep_for(
          std::chrono::milliseconds(static_cast<int>((1.0 / print_rate * 1000.0))));
      
      // Try to lock data to avoid read write collisions.
      if (print_data.mutex.try_lock()) {
        if (print_data.has_data) {
          // Publish
          icra18::Etc msg;
          for (size_t i=0; i<6; i++) {
            msg.force_error.push_back(print_data.force_error[i]);
          }
          etc_pub.publish(msg);

          print_data.has_data = false;
        }
        print_data.mutex.unlock();
      }
    }
  });

  try {
    // Connect to robot.
    franka::Robot robot(FCI_IP);
    setDefaultBehavior(robot);
    
    // First move the robot to a suitable joint configuration
    // std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    // MotionGenerator motion_generator(0.5, q_goal);
    // std::cout << "WARNING: This example will move the robot! "
    //           << "Please make sure to have the user stop button at hand!" << std::endl
    //           << "Press Enter to continue..." << std::endl;
    // std::cin.ignore();
    // robot.control(motion_generator);
    // std::cout << "Finished moving to initial joint configuration." << std::endl;

    // Set additional parameters always before the control loop, NEVER in the control loop!
    // Set collision behavior.
    robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});

    robot.setCartesianImpedance({1000, 1000, 1000, 100, 100, 100});
    
    // Load the kinematics and dynamics model.
    franka::Model model = robot.loadModel();
    franka::RobotState initial_state = robot.readOnce();

    // for Cartesian Pose Motion Generator
    std::array<double, 16> initial_pose = initial_state.O_T_EE_c;
    
    // for Torque Control
    Eigen::Matrix<double, 6, 1> force_ext_initial_(initial_state.O_F_ext_hat_K.data());
    Eigen::Matrix<double, 6, 1> force_error_;
    Eigen::Matrix<double, 6, 1> force_error_int_;


    // Define callback function to send Cartesian pose goals to get inverse kinematics solved.
    auto cartesian_pose_callback = [=, &time, &vel_current, &running, &angle, &initial_pose](
                                       const franka::RobotState& robot_state,
                                       franka::Duration period) -> franka::CartesianPose 
    {
      // Update time.
      time += period.toSec();

      // Initial Setting
      if (time == 0.0) {
        // initial_pose = robot_state.O_T_EE_c;
        vel_current = 0.0;
        angle = 0.0;
      }

      // Compute Cartesian velocity.
      if (time > start_time) {

        if (vel_current < vel_max && time < start_time + run_time) {
        vel_current += period.toSec() * std::fabs(vel_max / acceleration_time);
        }
        if (vel_current > 0.0 && time > start_time + run_time) {
          vel_current -= period.toSec() * std::fabs(vel_max / acceleration_time);
        }
        vel_current = std::fmax(vel_current, 0.0);
        vel_current = std::fmin(vel_current, vel_max);

        // Compute new angle for our circular trajectory.
        angle += period.toSec() * vel_current / std::fabs(radius);
        if (angle > 2 * M_PI) {
          angle -= 2 * M_PI;
        }
      }
      
      // Compute relative y and z positions of desired pose.
      double delta_x = radius * (1 - std::cos(angle));
      double delta_y = radius * std::sin(angle);
      
      franka::CartesianPose pose_desired = initial_pose;
      pose_desired.O_T_EE[12] += delta_x;
      pose_desired.O_T_EE[13] += delta_y;

      std::cout << "delta_x:" << delta_x << std::endl;

      // Send desired pose.
      if (time >= start_time + run_time + acceleration_time) {
        running = false;
        return franka::MotionFinished(pose_desired);
      }
      return pose_desired;
    };

    // Define callback for the joint torque control loop.
    std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
        force_control_callback =
            [&time1, &print_data, &model, &force_ext_initial_, &force_error_, &force_error_int_,
              k_p_, k_i_, desired_force_](
                const franka::RobotState& robot_state, franka::Duration period) -> franka::Torques 
    {
      // Update time.
      time1 += period.toSec();

      // Initial Setting
      if (time1 == 0.0) {
        // force_ext_initial_ = Eigen::Matrix<double, 6, 1>::Map(robot_state.O_F_ext_hat_K.data());
        force_error_int_.setZero();
      }

      std::array<double, 42> jacobian_array = model.zeroJacobian(franka::Frame::kEndEffector, robot_state);
      std::array<double, 7> coriolis_array = model.coriolis(robot_state);
      Eigen::Map<Eigen::Matrix<double, 7, 1> > coriolis(coriolis_array.data());
      Eigen::Map<Eigen::Matrix<double, 6, 7> > jacobian(jacobian_array.data());
      // Eigen::Map<Eigen::Matrix<double, 6, 1> > force_ext(robot_state.O_F_ext_hat_K.data());
      Eigen::Map<const Eigen::Matrix<double, 6, 1>> force_ext(robot_state.O_F_ext_hat_K.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
      Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
      Eigen::Vector3d position(transform.translation());
      Eigen::Quaterniond orientation(transform.linear());

      Eigen::VectorXd desired_force_torque(6), tau_force(7), force_control(6);

      // FORCE CONTROL
      desired_force_torque.setZero();
      desired_force_torque(2) = -desired_force_;
      force_error_ = desired_force_torque - force_ext + force_ext_initial_;
      force_error_int_ = force_error_int_ + period.toSec() * force_error_;
      force_control = (desired_force_torque + k_p_ * force_error_ + k_i_ * force_error_int_);
      force_control << 0, 0, force_control(2), 0, 0, 0;
      // FF + PI control
      tau_force = jacobian.transpose() * force_control;

      std::array<double, 7> tau_force_array;
      std::array<double, 6> force_error_array;
      // for (int i=0; i<7; i++) tau_force_desired[i] = tau_force[i];
      Eigen::VectorXd::Map(&tau_force_array[0], 7) = tau_force;
      Eigen::VectorXd::Map(&force_error_array[0], 6) = force_error_;
      
      // Update data to print.
      if (print_data.mutex.try_lock()) {
        print_data.has_data = true;
        print_data.robot_state = robot_state;
        print_data.force_error = force_error_array;
        print_data.mutex.unlock();
      }

      // Send torque command.
      return tau_force_array;
    };
    // Start real-time control loop.
    // robot.control(force_control_callback, cartesian_pose_callback, false, franka::kMaxCutoffFrequency);
    // robot.control(force_control_callback);
    robot.control(cartesian_pose_callback, franka::ControllerMode::kCartesianImpedance);
  } catch (const franka::Exception& ex) {
    running = false;
    std::cerr << ex.what() << std::endl;
  }

  if (print_thread.joinable()) {
    print_thread.join();
  }
  return 0;
}