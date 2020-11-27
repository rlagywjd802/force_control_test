#include <array>
#include <atomic>
#include <cmath>
#include <functional>
#include <iostream>
#include <fstream>
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
#define DT 0.001

enum MotionStage {
    LANDING_ACC = 0,
    LANDING_UV,
    PREP,
    FINISHED
};

struct Point {
  double x; 
  double y;
};

namespace {
template <class T, size_t N>
std::ostream& operator<<(std::ostream& ostream, const std::array<T, N>& array) {
  ostream << "[";
  std::copy(array.cbegin(), array.cend() - 1, std::ostream_iterator<T>(ostream, ","));
  std::copy(array.cend() - 1, array.cend(), std::ostream_iterator<T>(ostream));
  ostream << "]";
  return ostream;
}
}  // anonymous namespace -- ????

bool check_landing_velocity_reached(const franka::RobotState& robot_state, Point landing_velocity){
  bool is_reached = false;
  double vx = std::fabs(robot_state.O_dP_EE_c[0]);  // Last commanded end effector twist in base frame
  double vy = std::fabs(robot_state.O_dP_EE_c[1]);
  
  if (vx >= std::fabs(landing_velocity.x) && vy >= std::fabs(landing_velocity.y)) 
    is_reached = true;
  
  return is_reached;
}

double get_distance(const franka::RobotState& robot_state, const std::array<double, 16>& initial_pose){
  double dx = robot_state.O_T_EE[12] - initial_pose[12];
  double dy = robot_state.O_T_EE[13] - initial_pose[13];
  double distance = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));

  return distance;
}

int main(int argc, char** argv) {
  // Check whether the required arguments were passed.
  const std::string file_path = "/home/hj/circle.csv";

  // Set and initialize trajectory parameters.
  const double acceleration_time = 0.1;  // [s]
  const double landing_distance = 0.01;  // [m]

  // Set motion related parameters.
  std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
  std::vector<Point> prep_motion;
  Point landing_velocity, a, b;

  // Set print rate for measured data.
  const double print_rate = 1000.0;

  // Initialize data fields for the print thread.
  struct {
    std::mutex mutex;
    bool has_data;
    franka::RobotState robot_state;
    MotionStage stage;
    Point desired_x;
    Point desired_xdot;
    Point desired_xddot;
    double distance;
    uint64_t duration;
  } print_data{};
  std::atomic_bool running{true};

  // print basic info
  std::cout << "===================================" << std::endl;
  std::cout << "FCI_IP    : " << FCI_IP << std::endl;
  std::cout << "Read data from " << file_path << std::endl;

  // read desired point from csv file
  std::ifstream ap(file_path);

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
      pt.x = std::stof(x)/1000.0; // mm -> m
      pt.y = std::stof(y)/1000.0; // mm -> m

      // std::cout << "x:" << x << ", y:" << y << std::endl;
      prep_motion.push_back(pt);
    }
  }

  // print vector
  // for (auto it=prep_motion.cbegin(); it!=prep_motion.cend(); it++)
  // {
  //   std::cout << "x:" << it->x << ", y:" << it->y << std::endl;
  // }

  // calculate landing_velocity and parameter for cubic from prep motion
  landing_velocity.x = (prep_motion[1].x - prep_motion[0].x)/DT;
  landing_velocity.y = (prep_motion[1].y - prep_motion[0].y)/DT;
  a.x = -2*landing_velocity.x/std::pow(acceleration_time, 3);
  a.y = -2*landing_velocity.y/std::pow(acceleration_time, 3);
  b.x = 3*landing_velocity.x/std::pow(acceleration_time, 2);
  b.y = 3*landing_velocity.y/std::pow(acceleration_time, 2);

  std::cout << "Number of waypoints : " << prep_motion.size() << std::endl;
  std::cout << "Acceleration_time : " << acceleration_time << std::endl;
  std::cout << "Landing_distance : " << landing_distance << std::endl;
  std::cout << "landing_velocity : " << landing_velocity.x << ", " 
                                     << landing_velocity.y << std::endl;
  std::cout << "cubic velocity : " << a.x << ", " << b.x << "|"
                                   << a.y << ", " << b.y << std::endl;
  std::cout << "===================================" << std::endl;

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
            << print_data.desired_x.x << "," << print_data.desired_x.y << ","
            << print_data.desired_xdot.x << "," << print_data.desired_xdot.y << ","
            << print_data.desired_xddot.x << "," << print_data.desired_xddot.y << ","
            // << print_data.robot_state.O_T_EE[12] << "," << print_data.robot_state.O_T_EE[13] << ","
            // << print_data.robot_state.O_T_EE_c[12] << "," << print_data.robot_state.O_T_EE_c[13] << ","
            << vel_x << "," << vel_y << "," << vel << ","
            // << acc_x << "," << acc_y << "," << acc << ","
            << print_data.distance << "," << print_data.stage << "," << print_data.duration
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
    
    // Define initial variable for motion callback.
    std::array<double, 16> initial_pose;
    double desired_x = 0.0;
    double desired_y = 0.0;
    enum MotionStage stage = LANDING_ACC;
    double t = 0.0;
    int i = 1;
    
    // Define callback function to send Cartesian pose goals to get inverse kinematics solved.
    auto cartesian_pose_callback = [=, &t, &i, &initial_pose, &desired_x, &desired_y, &stage, 
                                       &running, &print_data](
                                       const franka::RobotState& robot_state,
                                       franka::Duration period) -> franka::CartesianPose {
      // Update time.
      t += period.toSec();

      // Read the initial pose to start the motion from in the first time step.
      if (t == 0.0) {
        initial_pose = robot_state.O_T_EE_c;
      }

      // Decide motion stage
      // if (stage == LANDING_ACC && /*&& check_landing_velocity_reached(robot_state, landing_velocity)*/){
      if (stage == LANDING_ACC && t >= acceleration_time){
        stage = LANDING_UV;
      }

      double distance_from_start = get_distance(robot_state, initial_pose);
      if (distance_from_start >= landing_distance){
        if (stage == LANDING_ACC)     stage = FINISHED;
        else if (stage == LANDING_UV) stage = PREP;
      }

      if (i >= prep_motion.size() - 1)
        stage = FINISHED;

      franka::CartesianPose pose_desired = initial_pose;
      double desired_xdot = 0.0;
      double desired_ydot = 0.0;
      double desired_xddot = 0.0;
      double desired_yddot = 0.0;
      switch (stage) {
        case LANDING_ACC:
          desired_x = a.x*std::pow(t, 4)/4.0 + b.x*std::pow(t, 3)/3.0;
          desired_y = a.y*std::pow(t, 4)/4.0 + b.y*std::pow(t, 3)/3.0;
          desired_xdot = a.x*std::pow(t, 3) + b.x*std::pow(t, 2);
          desired_ydot = a.y*std::pow(t, 3) + b.y*std::pow(t, 2);
          desired_xddot = 3*a.x*std::pow(t, 2) + 2*b.x*std::pow(t, 1);
          desired_yddot = 3*a.y*std::pow(t, 2) + 2*b.y*std::pow(t, 1);
          break;
        
        case LANDING_UV:
          desired_x += landing_velocity.x*period.toSec();
          desired_y += landing_velocity.y*period.toSec();
          break;
        
        case PREP:
          desired_x += prep_motion[i].x;
          desired_y += prep_motion[i].y;
          i += period.toMSec();  // in case of packet loss happens
          break;
        
        case FINISHED:
          pose_desired.O_T_EE[12] += desired_x;
          pose_desired.O_T_EE[13] += desired_y;
          running = false;
          return franka::MotionFinished(pose_desired);  // 꼭 pose_desired 넘겨줘야 되나????
      }

      // Update data to print.
      if (print_data.mutex.try_lock()) {
        print_data.has_data = true;
        print_data.robot_state = robot_state;
        print_data.stage = stage;
        print_data.desired_x = {desired_x, desired_y};
        print_data.desired_xdot = {desired_xdot, desired_ydot};
        print_data.desired_xddot = {desired_xddot, desired_yddot};
        print_data.distance = distance_from_start;
        print_data.duration = period.toMSec();
        print_data.mutex.unlock();
      }

      // Send desired pose.
      pose_desired.O_T_EE[12] += desired_x;
      pose_desired.O_T_EE[13] += desired_y;
      return pose_desired;
    };

    robot.control(cartesian_pose_callback);
    // robot.control(cartesian_pose_callback, franka::ControllerMode::kJointImpedance, true, 10);
  } catch (const franka::Exception& ex) {
    running = false;
    std::cerr << ex.what() << std::endl;
  }

  if (print_thread.joinable()) {
    print_thread.join();
  }
  return 0;
}
