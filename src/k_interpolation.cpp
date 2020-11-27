#include <array>
#include <atomic>
#include <cmath>
#include <functional>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iterator>
#include <mutex>
#include <thread>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>

#include <Poco/DateTimeFormatter.h>
#include <Poco/File.h>
#include <Poco/Path.h>

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
  const double radius = 0.05;
  const double vel_max = 0.1;
  const double acceleration_time = 0.5;
  const double run_time = 3.0;

  // Set print rate for comparing commanded vs. measured torques.
  const double print_rate = 1000.0;
  double vel_current = 0.0;
  double angle = 0.0;
  double time = 0.0;

  // Set speed of tracking trajectory
  int cnt = 0;
  int i = 0;
  int len = 0;
  const float k = 20; // number of interporlation

  // Initialize data fields for the print thread.
  struct {
    std::mutex mutex;
    bool has_data;
    franka::RobotState robot_state;
    double delta_x;
    double delta_y;
    double time;
    uint64_t duration;
  } print_data{};
  std::atomic_bool running{true};

  // read csv
  std::ifstream ap("/home/hj/circle.csv");
  struct Point {
    float x; 
    float y;
  };
  std::vector<Point> data_list;

  if (!ap.is_open()){
    std::cout << "ERROR: Failed to Open File" << std::endl; 
  }
  else{
    std::string x;
    std::string y;
    Point pt;
    while(ap.good()){
      getline(ap, x, ',');
      getline(ap, y, '\n');
      pt.x = std::stof(x); // mm -> m
      pt.y = std::stof(y); // mm -> m

      // std::cout << "x:" << x << ", y:" << y << std::endl;
      data_list.push_back(pt);
      len++;
    }
  }

  for (auto it=data_list.cbegin(); it!=data_list.cend(); it++)
  {
    std::cout << "x:" << it->x << ", y:" << it->y << std::endl;
  }
  std::cout << "--------------------" << std::endl;

  // for print
  std::ofstream log_data("/home/hj/log_data.csv");
  log_data << "t" << "," << "command_success_rate" << ","
               << "dq[0]" << "," << "dq[1]" << "," << "dq[2]" << "," << "dq[3]" << "," << "dq[4]" << "," << "dq[5]" << "," << "dq[6]" << ","
               << "tau_J[0]" << "," << "tau_J[1]" << "," << "tau_J[2]" << "," << "tau_J[3]" << "," << "tau_J[4]" << "," << "tau_J[5]" << "," << "tau_J[6]" << ","
               // << "measured_x" << "," << "measured_y" << ","
               // << "commanded_x" << "," << "commanded_y" << ","
               // << "commanded_xd" << "," << "commanded_yd" << ","
               // << "commanded_xdd" << "," << "commanded_ydd"
               << std::endl;

  // Start print thread.
  std::thread print_thread([print_rate, &print_data, &running]() {
    while (running) {
      // Sleep to achieve the desired print rate.
      std::this_thread::sleep_for(
          std::chrono::milliseconds(static_cast<int>((1.0 / print_rate * 1000.0))));
      
      // Try to lock data to avoid read write collisions.
      if (print_data.mutex.try_lock()) {
        if (print_data.has_data) {
          double vel_x = print_data.robot_state.O_dP_EE_c[0];
          double vel_y = print_data.robot_state.O_dP_EE_c[1];
          double vel = std::sqrt(vel_x*vel_x + vel_y*vel_y);
          double acc_x = print_data.robot_state.O_ddP_EE_c[0];
          double acc_y = print_data.robot_state.O_ddP_EE_c[1];
          double acc = std::sqrt(acc_x*acc_x + acc_y*acc_y);
          std::cout
            << print_data.delta_x << "," << print_data.delta_y << ","
            << print_data.robot_state.O_T_EE[12] << "," << print_data.robot_state.O_T_EE[13] << ","
            << vel_x << "," << vel_y << "," << vel << ","
            << acc_x << "," << acc_y << "," << acc << ","
            << print_data.time
            << std::endl; 
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
    std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    MotionGenerator motion_generator(0.5, q_goal);
    std::cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    robot.control(motion_generator);
    std::cout << "Finished moving to initial joint configuration." << std::endl;

    // Set additional parameters always before the control loop, NEVER in the control loop!
    // Set collision behavior.
    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});
    
    // Load the kinematics and dynamics model.
    franka::Model model = robot.loadModel();
    std::array<double, 16> initial_pose;
    
    // Define callback function to send Cartesian pose goals to get inverse kinematics solved.
    auto cartesian_pose_callback = [=, &time, &vel_current, &running, &angle, &initial_pose, &cnt, &i, &log_data](
                                       const franka::RobotState& robot_state,
                                       franka::Duration period) -> franka::CartesianPose {
      // Update time.
      time += period.toSec();

      if (time == 0.0) {
        // Read the initial pose to start the motion from in the first time step.
        initial_pose = robot_state.O_T_EE_c;
      }

      if (period.toMSec() > 1){
        std::cout << "*" << std::endl;
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
      angle += period.toSec() * vel_current / std::fabs(radius);
      if (angle > 2 * M_PI) {
        angle -= 2 * M_PI;
      }
      // Compute relative y and z positions of desired pose.
      double delta_x = radius * (1 - std::cos(angle));
      double delta_y = radius * std::sin(angle);
      // double delta_x;
      // double delta_y;
      // bool is_finished = false;
      // std::cout << cnt << "/" << i << std::endl;
      // if (cnt < len) {
      //   delta_x = ((data_list[cnt].x)*(k-i) + data_list[cnt+1].x*i)/k; 
      //   delta_y = ((data_list[cnt].y)*(k-i) + data_list[cnt+1].y*i)/k;
      // }
      // else {
      //   delta_x = data_list[cnt].x;
      //   delta_y = data_list[cnt].y;
      //   is_finished = true;
      // }
      // std::cout << delta_x << "/" << delta_y << std::endl;
      // if (i < k-1) {
      //   i++;
      // }
      // else {
      //   cnt++;
      //   i = 0;
      // }

      franka::CartesianPose pose_desired = initial_pose;
      // pose_desired.O_T_EE[12] += delta_x;
      // pose_desired.O_T_EE[13] += delta_y;
      pose_desired.O_T_EE[12] += delta_x;
      pose_desired.O_T_EE[13] += delta_y;

      // std::cout << delta_x << "," << delta_y << std::endl;

      // Update data to print.
      // if (print_data.mutex.try_lock()) {
      //   print_data.has_data = true;
      //   print_data.robot_state = robot_state;
      //   print_data.delta_x = delta_x;
      //   print_data.delta_y = delta_y;
      //   print_data.time = time;
      //   print_data.duration = period.toMSec();
      //   print_data.mutex.unlock();
      // }
      log_data << robot_state.time.toMSec() << "," << robot_state.control_command_success_rate << ","
               << robot_state.dq[0] << "," << robot_state.dq[1] << "," << robot_state.dq[2] << "," << robot_state.dq[3] << ","
               << robot_state.dq[4] << "," << robot_state.dq[5] << "," << robot_state.dq[6] << ","
               << robot_state.tau_J[0] << "," << robot_state.tau_J[1] << "," << robot_state.tau_J[2] << "," << robot_state.tau_J[3] << ","
               << robot_state.tau_J[4] << "," << robot_state.tau_J[5] << "," << robot_state.tau_J[6] << ","
               // << robot_state.O_T_EE[12] << "," << robot_state.O_T_EE[13] << ","
               // << robot_state.O_T_EE_c[12] << "," << robot_state.O_T_EE_c[13] << ","
               // << robot_state.O_dP_EE_c[0] << "," << robot_state.O_dP_EE_c[1] << ","
               // << robot_state.O_ddP_EE_c[0] << "," << robot_state.O_ddP_EE_c[1]
               << std::endl;

      // Send desired pose.
      if (time >= run_time) {
        std::cout << "Finished" << std::endl;
        running = false;
        return franka::MotionFinished(pose_desired);
      }
      // if (is_finished) {
      //   return franka::MotionFinished(pose_desired);
      // }
      return pose_desired;
    };

    robot.control(cartesian_pose_callback, franka::ControllerMode::kJointImpedance, false, 1000.0);
  } catch (const franka::Exception& ex) {
    running = false;
    std::cerr << ex.what() << std::endl;
  }

  // if (print_thread.joinable()) {
  //   print_thread.join();
  // }
  // savelogToCSV(log_stream);

  return 0;
}
