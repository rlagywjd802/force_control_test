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
  const double rotational_stiffness{50.0};
  Eigen::MatrixXd stiffness(6, 6), damping(6, 6);
  stiffness.setZero();
  stiffness.topLeftCorner(3, 3) << translational_stiffness * Eigen::MatrixXd::Identity(3, 3);
  stiffness.bottomRightCorner(3, 3) << rotational_stiffness * Eigen::MatrixXd::Identity(3, 3);
  // stiffness(2,2) = 10;
  damping.setZero();
  damping.topLeftCorner(3, 3) << 2.0 * sqrt(translational_stiffness) *
                                     Eigen::MatrixXd::Identity(3, 3);
  damping.bottomRightCorner(3, 3) << 2.0 * sqrt(rotational_stiffness) *
                                         Eigen::MatrixXd::Identity(3, 3);
  // damping(2,2) = 200;

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

    // std::cout << "orientation_d: " << orientation_d << std::endl;    
    // set collision behavior
    robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});
    
    // define callback for the torque control loop
    std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
        impedance_control_callback = [=, &running, &print_data, &model](const franka::RobotState& robot_state,
                                         franka::Duration /*duration*/) -> franka::Torques 
    {      
      // get state variables
      std::array<double, 7> coriolis_array = model.coriolis(robot_state);
      std::array<double, 42> jacobian_array =
          model.zeroJacobian(franka::Frame::kEndEffector, robot_state);
      
      // convert to Eigen
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
      Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
      Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
      Eigen::Vector3d position(transform.translation());
      Eigen::Quaterniond orientation(transform.linear());

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
      tau_task << jacobian.transpose() * (-stiffness * error - damping * (jacobian * dq));
      tau_d << tau_task + coriolis;
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