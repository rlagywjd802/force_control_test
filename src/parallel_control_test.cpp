#include <array>
#include <cmath>
#include <functional>
#include <iostream>

#include <atomic>
#include <mutex>
#include <thread>

#include <Eigen/Dense>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>

#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>

#include "examples_common.h"

#define FCI_IP "192.168.30.151"

int main(int argc, char** argv) {

  // ros
  ros::init(argc, argv, "cartesian_impedance_control_test");
  ros::NodeHandle nh;
  ros::Publisher ft_pub = nh.advertise<geometry_msgs::WrenchStamped>("ft_data", 1);

  // Initialize data fields for the print thread.
  struct {
    std::mutex mutex;
    bool has_data;
    franka::RobotState robot_state;
  } print_data{};
  std::atomic_bool running{true};

  // Set print rate and thread related parameters
  const double print_rate = 100.0;
  int ft_seq = 0;

  // Start print thread.
  std::thread print_thread([print_rate, &print_data, &running, &ft_seq, &ft_pub]() {
    while (running) {
      // Sleep to achieve the desired print rate.
      std::this_thread::sleep_for(
          std::chrono::milliseconds(static_cast<int>((1.0 / print_rate * 1000.0))));
      
      // Try to lock data to avoid read write collisions.
      if (print_data.mutex.try_lock()) {
        // if (print_data.has_data) {
          // Print data to console
          // std::cout << print_data.has_data << ":" << print_data.robot_state.K_F_ext_hat_K[0] << "," << print_data.robot_state.K_F_ext_hat_K[1] << "," << print_data.robot_state.K_F_ext_hat_K[2] << std::endl;
          
          // Publish
          geometry_msgs::WrenchStamped ft_data;
          ft_data.header.seq = ft_seq++;
          ft_data.header.stamp = ros::Time::now();
          ft_data.header.frame_id = "world";
          ft_data.wrench.force.x = print_data.robot_state.K_F_ext_hat_K[0];
          ft_data.wrench.force.y = print_data.robot_state.K_F_ext_hat_K[1];
          ft_data.wrench.force.z = print_data.robot_state.K_F_ext_hat_K[2];
          ft_data.wrench.torque.x = print_data.robot_state.K_F_ext_hat_K[3];
          ft_data.wrench.torque.y = print_data.robot_state.K_F_ext_hat_K[4];
          ft_data.wrench.torque.z = print_data.robot_state.K_F_ext_hat_K[5];
          ft_pub.publish(ft_data);

          // 
          print_data.has_data = false;
        // }
        print_data.mutex.unlock();
      }
    }
  });


  // Compliance parameters
  const double translational_stiffness{150.0};
  const double rotational_stiffness{10.0};
  Eigen::MatrixXd stiffness(6, 6), damping(6, 6);
  stiffness.setZero();
  stiffness.topLeftCorner(3, 3) << translational_stiffness * Eigen::MatrixXd::Identity(3, 3);
  stiffness.bottomRightCorner(3, 3) << rotational_stiffness * Eigen::MatrixXd::Identity(3, 3);
  damping.setZero();
  damping.topLeftCorner(3, 3) << 2.0 * sqrt(translational_stiffness) *
                                     Eigen::MatrixXd::Identity(3, 3);
  damping.bottomRightCorner(3, 3) << 2.0 * sqrt(rotational_stiffness) *
                                         Eigen::MatrixXd::Identity(3, 3);

  // Force control PI
  double desired_force_{1.0};
  double target_force_{0.0};
  double k_p_{10.0};
  double k_i_{0.0};
  Eigen::Matrix<double, 6, 1> force_ext_initial_;
  Eigen::Matrix<double, 6, 1> force_error_;
  static constexpr double kDeltaTauMax{1.0};
  bool started = true;

  // Wiggle motions
  double time_{0.0};
  double wiggle_frequency_x_{0.5};
  double wiggle_frequency_y_{0.5};
  double amplitude_wiggle_x_{0.1};
  double amplitude_wiggle_y_{0.1};
  
  try {
    // connect to robot
    franka::Robot robot(FCI_IP);
    setDefaultBehavior(robot);
    
    // load the kinematics and dynamics model
    franka::Model model = robot.loadModel();
    franka::RobotState initial_state = robot.readOnce();
    
    // equilibrium point is the initial position
    Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
    Eigen::Vector3d position_d(initial_transform.translation());
    Eigen::Quaterniond orientation_d(initial_transform.linear());
    
    // set collision behavior
    robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});
    
    // define callback for the torque control loop
    std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
        impedance_control_callback = [=, &running, &print_data, &model, &force_ext_initial_, &force_error_, &started](const franka::RobotState& robot_state,
                                         franka::Duration period) -> franka::Torques {      
      // get state variables
      std::array<double, 7> coriolis_array = model.coriolis(robot_state);
      std::array<double, 42> jacobian_array =
          model.zeroJacobian(franka::Frame::kEndEffector, robot_state);
      
      // convert to Eigen
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
      Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
      Eigen::Map<const Eigen::Matrix<double, 6, 1> > force_ext(robot_state.O_F_ext_hat_K.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
      Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
      Eigen::Vector3d position(transform.translation());
      Eigen::Quaterniond orientation(transform.linear());

      Eigen::VectorXd desired_force_torque(6), tau_force(7), tau_cartesian_impedance(7), tau_cmd(7), force_control(6);

      // FORCE CONTROL
      if (started) {
        force_ext_initial_ = force_ext;
        force_error_.setZero();
        started = false;
      }        

      desired_force_torque.setZero();
      desired_force_torque(2) = -desired_force_;
      force_error_ = force_error_ + period.toSec() * (desired_force_torque - force_ext + force_ext_initial_);
      force_control = (desired_force_torque + k_p_ * (desired_force_torque - force_ext + force_ext_initial_) +
                                            k_i_ * force_error_);
      force_control << 0, 0, force_control(2), 0, 0, 0;
      // FF + PI control
      tau_force = jacobian.transpose() * force_control;

      // WIGGLE MOTION
      // Eigen::AngleAxisd angle_axis_wiggle_x;
      // angle_axis_wiggle_x.axis() << 1, 0, 0;
      // angle_axis_wiggle_x.angle() = sin(2.0 * M_PI * time_ * wiggle_frequency_x_) * amplitude_wiggle_x_;
      // Eigen::AngleAxisd angle_axis_wiggle_y;
      // angle_axis_wiggle_y.axis() << 0, 1, 0;
      // angle_axis_wiggle_y.angle() = sin(2.0 * M_PI * time_ * wiggle_frequency_y_) * amplitude_wiggle_y_;

      // Eigen::Quaterniond wiggle_x(angle_axis_wiggle_x);
      // Eigen::Quaterniond wiggle_y(angle_axis_wiggle_y);
      // Eigen::Quaterniond orientation_d(wiggle_y*(wiggle_x*orientation_d));
      
      // compute error to desired equilibrium pose
      // position error
      Eigen::Matrix<double, 6, 1> error;
      error.head(3) << position - position_d;
      
      // orientation error
      // "difference" quaternion
      if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0) {
        orientation.coeffs() << -orientation.coeffs();
      }
      
      // "difference" quaternion
      Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d);
      error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
      
      // Transform to base frame
      error.tail(3) << -transform.linear() * error.tail(3);
      
      // compute control
      Eigen::VectorXd tau_task(7), tau_d(7);
      
      // Spring damper system with damping ratio=1
      // tau_task << jacobian.transpose() * (-stiffness * error - damping * (jacobian * dq));
      // tau_d << tau_task + coriolis;

      // Cartesian PD control
      tau_task << jacobian.transpose() *
                  (-stiffness * error - damping * (jacobian * dq));
      // Desired torque
      tau_cartesian_impedance << tau_task + coriolis;

      // tau_d << tau_cartesian_impedance + tau_force;
      tau_d << coriolis + tau_force;

      std::array<double, 7> tau_d_array{};
      Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;

      // Update data to print.
      if (print_data.mutex.try_lock()) {
        print_data.has_data = true;
        print_data.robot_state = robot_state;
        print_data.mutex.unlock();
      }

      return tau_d_array;
    };
    
    // start real-time control loop
    std::cout << "WARNING: Collision thresholds are set to high values. "
              << "Make sure you have the user stop at hand!" << std::endl
              << "After starting try to push the robot and see how it reacts." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    robot.control(impedance_control_callback);
  } catch (const franka::Exception& ex) {
    running = false;
    std::cerr << ex.what() << std::endl;
  }
  return 0;
}