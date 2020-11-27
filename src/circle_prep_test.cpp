#include <array>
#include <atomic>
#include <cmath>
#include <functional>
#include <iostream>
#include <iterator>
#include <mutex>
#include <thread>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>

#include <ros/ros.h>
#include <force_control_test/FrankaState.h>

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
  // const double radius = 0.02; //0.0196;
  const double vel_max = 0.003;
  const double acceleration_time = 0.2;
  const double run_time = 50.0;

  // Circle Motion
  const double kPrepOX = 0.634896;
  const double kPrepOY = -0.022188;
  const double kPrepOZ = 0.3140;
  const double kPrepR = 0.0202;

  // Set print rate for comparing commanded vs. measured torques.
  const double print_rate = 5.0;
  double vel_current = 0.0;
  double angle = 0.0;
  double time = 0.0;

  // Initialize data fields for the print thread.
  struct {
    std::mutex mutex;
    bool has_data;
    std::array<double, 6> force;
    franka::RobotState robot_state;
  } print_data{};
  std::atomic_bool running{true};

  // Publisher
  ros::init(argc, argv, "circle_prep_test");
  ros::NodeHandle nh;
  ros::Publisher etc_pub = nh.advertise<force_control_test::FrankaState>("franka", 1);

  // Start print thread.
  std::thread print_thread([print_rate, &print_data, &running, etc_pub]() {
    while (running) {
      // Sleep to achieve the desired print rate.
      std::this_thread::sleep_for(
          std::chrono::milliseconds(static_cast<int>((1.0 / print_rate * 1000.0))));
      
      // Try to lock data to avoid read write collisions.
      if (print_data.mutex.try_lock()) {
        if (print_data.has_data) {
          force_control_test::FrankaState msg;
          for (size_t i=0; i<6; i++) {
            msg.force.push_back(print_data.force[i]);
          }
          msg.force.push_back(std::sqrt(print_data.force[0]*print_data.force[0] + print_data.force[1]*print_data.force[1]));
          
          msg.position.push_back(print_data.robot_state.O_T_EE[12]);
          msg.position.push_back(print_data.robot_state.O_T_EE[13]);
          msg.position.push_back(print_data.robot_state.O_T_EE[14]);
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

    // Warning For Start
    std::cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    
    // First move the robot to a suitable joint configuration
    std::array<double, 7> q_goal = {{0, -M_PI/6.0, 0, -5 * M_PI / 6.0, 0, 2*M_PI/3.0, 0.0}};
    MotionGenerator motion_generator(0.2, q_goal);
    robot.control(motion_generator);
    std::cout << "Finished moving to initial joint configuration." << std::endl;

    // std::array<double, 3> x_goal_q = {{0.5637, -0.0347, 0.3225}};
    std::array<double, 3> x_goal_q = {{kPrepOX-kPrepR-0.05, kPrepOY, kPrepOZ}}; ///////////////////////// Q
    CMotionGenerator cartesian_motion_generator_q(0.2, x_goal_q);
    robot.control(cartesian_motion_generator_q);
    std::cout << "Finished moving to point Q" << std::endl;

    // std::array<double, 3> x_goal_p = {{0.6137, -0.0347, 0.3225}};
    std::array<double, 3> x_goal_p = {{kPrepOX-kPrepR, kPrepOY, kPrepOZ}}; ///////////////////////// P
    CMotionGenerator cartesian_motion_generator_p(0.2, x_goal_p);
    robot.control(cartesian_motion_generator_p);
    std::cout << "Finished moving to point P" << std::endl;

    std::cout << "===========================" << std::endl;
    std::cout << "radius : " << kPrepR << std::endl;
    std::cout << "vel_max : " << vel_max << std::endl;
    std::cout << "acceleration_time : " << acceleration_time << std::endl;
    std::cout << "Point P : " << x_goal_p[0] << ", " << x_goal_p[1] << ", " << x_goal_p[2] << std::endl;

    // Set additional parameters always before the control loop, NEVER in the control loop!
    // Set collision behavior.
    robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});
    robot.setCartesianImpedance({{1000, 1000, 3000, 300, 300, 300}});
    
    // robot.setCollisionBehavior(
    // {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
    // {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
    // {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
    // {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}});
    // robot.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});

    // robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});

    // Load the kinematics and dynamics model.
    franka::Model model = robot.loadModel();
    std::array<double, 16> initial_pose;
    std::array<double, 6> initial_force;
    
    // Define callback function to send Cartesian pose goals to get inverse kinematics solved.
    auto cartesian_pose_callback = [=, &print_data, &running, &time, &vel_current, 
                                    &angle, &initial_pose, &initial_force](
                                       const franka::RobotState& robot_state,
                                       franka::Duration period) -> franka::CartesianPose 
    {
      // Update time.
      time += period.toSec();
      if (time == 0.0) {
        // Read the initial pose to start the motion from in the first time step.
        initial_pose = robot_state.O_T_EE_c;
        initial_force = robot_state.K_F_ext_hat_K;
      }

      // Compute Cartesian velocity.
      if (vel_current < vel_max && time < run_time) {
        vel_current += period.toSec() * std::fabs(vel_max / acceleration_time);
      }
      if (vel_current > 0.0 && time > run_time) {
        vel_current -= period.toSec() * std::fabs(vel_max / acceleration_time);
      }
      vel_current = std::fmax(vel_current, 0.0);
      vel_current = std::fmin(vel_current, vel_max);

      // Compute new angle for our circular trajectory.
      angle += period.toSec() * vel_current / std::fabs(kPrepR);
      if (angle > 2 * M_PI) {
        angle -= 2 * M_PI;
      }
      
      // Compute relative y and z positions of desired pose.
      double delta_x = kPrepR * (1 - std::cos(angle));
      double delta_y = - kPrepR * std::sin(angle);
      franka::CartesianPose pose_desired = initial_pose;
      pose_desired.O_T_EE[12] += delta_x;
      pose_desired.O_T_EE[13] += delta_y;

      // Compute calibrated force
      std::array<double, 6> calibrated_force;
      for (int i=0; i<6; i++)
        calibrated_force[i] = robot_state.K_F_ext_hat_K[i] - initial_force[i];

      // Update data to print.
      if (print_data.mutex.try_lock()) {
        print_data.has_data = true;
        print_data.robot_state = robot_state;
        print_data.force = calibrated_force;
        print_data.mutex.unlock();
      }

      // Send desired pose.
      if (time >= run_time + acceleration_time) {
        running = false;
        return franka::MotionFinished(pose_desired);
      }
      return pose_desired;
    };


    // auto cartesian_vel_callback = [=, &print_data, &running, &time, &vel_current, 
    //                                 &angle, &initial_pose, &initial_force](
    //                                    const franka::RobotState& robot_state,
    //                                    franka::Duration period) -> franka::CartesianVelocities 
    // {
    //   // Update time.
    //   time += period.toSec();
    //   if (time == 0.0) {
    //     // Read the initial pose to start the motion from in the first time step.
    //     // initial_pose = robot_state.O_T_EE_c;
    //     initial_force = robot_state.K_F_ext_hat_K;
    //   }

    //   // Compute Cartesian velocity.
    //   if (vel_current < vel_max && time < run_time) {
    //     vel_current += period.toSec() * std::fabs(vel_max / acceleration_time);
    //   }
    //   if (vel_current > 0.0 && time > run_time) {
    //     vel_current -= period.toSec() * std::fabs(vel_max / acceleration_time);
    //   }
    //   vel_current = std::fmax(vel_current, 0.0);
    //   vel_current = std::fmin(vel_current, vel_max);

    //   // Compute new angle for our circular trajectory.
    //   double d_angle = period.toSec() * vel_current / std::fabs(radius);
    //   angle += period.toSec() * vel_current / std::fabs(radius);
    //   if (angle > 2 * M_PI) {
    //     angle -= 2 * M_PI;
    //   }    
      
    //   // Compute relative y and z positions of desired pose.
    //   double delta_x = - radius * std::sin(angle) * d_angle;
    //   double delta_y = radius * std::cos(angle) * d_angle;
    //   franka::CartesianVelocities vel_desired = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    //   vel_desired.O_dP_EE[0] = delta_x;
    //   vel_desired.O_dP_EE[1] = delta_y;

    //   // Compute calibrated force
    //   std::array<double, 6> calibrated_force;
    //   for (int i=0; i<6; i++)
    //     calibrated_force[i] = robot_state.K_F_ext_hat_K[i] - initial_force[i];

    //   // Update data to print.
    //   if (print_data.mutex.try_lock()) {
    //     print_data.has_data = true;
    //     print_data.robot_state = robot_state;
    //     print_data.force = calibrated_force;
    //     print_data.mutex.unlock();
    //   }

    //   // Send desired pose.
    //   if (time >= run_time + acceleration_time) {
    //     running = false;
    //     return franka::MotionFinished(vel_desired);
    //   }
    //   return vel_desired;
    // };



    // Start real-time control loop.
    robot.control(cartesian_pose_callback, franka::ControllerMode::kCartesianImpedance);
    // robot.control(cartesian_pose_callback);

  } catch (const franka::Exception& ex) {
    running = false;
    std::cerr << ex.what() << std::endl;
  }

  if (print_thread.joinable()) {
    print_thread.join();
  }
  return 0;
}