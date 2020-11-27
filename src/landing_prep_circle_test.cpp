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

#include <ros/ros.h>
#include <force_control_test/FrankaState.h>

#include <Poco/DateTimeFormatter.h>
#include <Poco/File.h>
#include <Poco/Path.h>

#include "examples_common.h"
#include "landing_motion.h"
#include "prep_motion.h"
#include "cartesian_pose_log.h"

#define FCI_IP "192.168.30.151"
#define DT 0.001
#define D2R M_PI/180.0

enum MotionStage {
    LANDING_CIRCULAR,
    PREP_SPLINE,
    TAKINGOFF_CIRCULAR,
    FINISHED
};

double getVelocity(const std::array<double, 3> vel);
void writeLogToFile(const std::vector<franka::Record>& log);

int main(int argc, char** argv) {
  
  // prep motion parameters
  // const double kPrepOX = 0.6350;
  // const double kPrepOY = -0.0220;
  // const double kPrepOZ = 0.3140;
  const double kPrepOX = 0.634896;
  const double kPrepOY = -0.022188;
  const double kPrepOZ = 0.3130; // 0.3140;
  const double kPrepR = 0.0203; // 0.0202;
  const double kPrepW = 10*D2R;
  const double kPrepTh0 = 0*D2R;
  const double kPrepThf = 360*D2R;
  std::array<double, 3> prep_center_of_circle = {kPrepOX, kPrepOY, kPrepOZ}; // Point O


  // landing motion parameters
  const double kLandingAccTime = 5.0;
  const double kLandingR = kPrepR;
  const char kLandingDir[4] = "ccc";  // 1-cw,(www) 2-ccw(ccc)
  const enum MotionStage kLandingType = LANDING_CIRCULAR;

  // Prep Motion
  CircularPrep prep(kPrepR, kPrepW, kPrepTh0, kPrepThf, prep_center_of_circle);
  std::array<std::array<double, 3>, 3> prep_start = prep.getPrepStart();
  std::array<std::array<double, 3>, 3> prep_end = prep.getPrepEnd();
  double prep_runtime = prep.getRuntime();
  std::cout << prep_runtime << "s";

  // Landing Motion
  CircularLanding landing(kLandingAccTime, kLandingR, kLandingDir, prep_start, true);
  CircularLanding takingoff(kLandingAccTime, kLandingR, kLandingDir, prep_end, false);
  landing.printMotionParameter();
  takingoff.printMotionParameter();

  // Set print rate for comparing commanded vs. measured torques.
  const double print_rate = 5.0;

  // Initialize data fields for the print thread.
  struct {
    std::mutex mutex;
    bool has_data;
    franka::RobotState robot_state;
  } print_data{};
  std::atomic_bool running{true};

  // Publisher
  ros::init(argc, argv, "landing_circle_prep_test");
  ros::NodeHandle nh;
  ros::Publisher franka_pub = nh.advertise<force_control_test::FrankaState>("franka", 1);

  // Start print thread.
  std::thread print_thread([print_rate, &print_data, &running, franka_pub]() {
    while (running) {
      // Sleep to achieve the desired print rate.
      std::this_thread::sleep_for(
          std::chrono::milliseconds(static_cast<int>((1.0 / print_rate * 1000.0))));
      
      // Try to lock data to avoid read write collisions.
      if (print_data.mutex.try_lock()) {
        if (print_data.has_data) {
          force_control_test::FrankaState msg;
          for (size_t i=0; i<6; i++) {
            msg.force.push_back(print_data.robot_state.O_F_ext_hat_K[i]);
          }
          // msg.force.push_back(std::sqrt(print_data.force[0]*print_data.force[0] + print_data.force[1]*print_data.force[1]));
          
          msg.position.push_back(print_data.robot_state.O_T_EE[12]);
          msg.position.push_back(print_data.robot_state.O_T_EE[13]);
          msg.position.push_back(print_data.robot_state.O_T_EE[14]);
          franka_pub.publish(msg);
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

    std::cout << "WARNING: This example will move the robot! "
          << "Please make sure to have the user stop button at hand!" << std::endl
          << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    // First move the robot to a suitable joint configuration
    std::array<double, 7> q_goal = {{0, -M_PI/6.0, 0, -5 * M_PI / 6.0, 0, 2*M_PI/3.0, 0.0}};
    MotionGenerator motion_generator(0.2, q_goal);
    robot.control(motion_generator);
    std::cout << "Finished moving to initial joint configuration." << std::endl;

    // Then, move the robot to start point of landing motion
    std::array<double, 3> x_goal_q = landing.getDesiredPose(0.0);
    CMotionGenerator cartesian_motion_generator_q(0.2, x_goal_q);
    robot.control(cartesian_motion_generator_q);
    std::cout << "Finished moving to point Q" << std::endl;

    // Next, ..
    std::cout << "LANDING START---------------" << std::endl;    
    
    // Set additional parameters always before the control loop, NEVER in the control loop!
    // Set collision behavior.
    // robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
    //                            {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
    //                            {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
    //                            {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});
    robot.setCollisionBehavior(
    {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
    {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
    {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
    {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}});
    robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
    // robot.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
    // robot.setCartesianImpedance({{1000, 1000, 3000, 300, 300, 300}});
    
    // Load the kinematics and dynamics model.
    std::array<double, 16> initial_pose;

    // Define initial variable for motion callback.
    std::array<double, 3> desired_vel;
    enum MotionStage stage = kLandingType;
    double t = 0.0;    

    // prep circle
    bool is_prep = false;
    double prep_t0 = 0.0;
    double prep_t1 = 0.0;
    double diff_x = 0.0;
    double diff_y = 0.0;
    
    // Define callback function to send Cartesian pose goals to get inverse kinematics solved.
    auto cartesian_vel_callback = [=, &running, &print_data, &t, &stage, &desired_vel, &diff_x, &diff_y, &prep_t0, &prep_t1](
                                       const franka::RobotState& robot_state,
                                       franka::Duration period) -> franka::CartesianVelocities 
    {
      // Update time.
      t += period.toSec();
     
      // Print out when packet loss happens
      // if (period.toMSec() > 1){
      //   std::cout << "*" << period.toMSec() << std::endl;
      // }

      // Decide motion stage
      if (t >= kLandingAccTime && stage == LANDING_CIRCULAR){
        stage = PREP_SPLINE;
        std::cout << "switch to prep" << std::endl;
      }
      if (t >= kLandingAccTime+prep_runtime && stage == PREP_SPLINE){
        stage = TAKINGOFF_CIRCULAR;
        std::cout << "switch to taking off" << std::endl;
      }
      if (t >= 2*kLandingAccTime+prep_runtime){
        stage = FINISHED;
        std::cout << "time to finish" << std::endl;
      }

      // get desired value
      franka::CartesianVelocities vel_desired = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
      switch (stage) {
        case LANDING_CIRCULAR:
          desired_vel = landing.getDesiredVel(t);
          break;
        
        case PREP_SPLINE:
          if (prep_t0 == 0) prep_t0 = t;
          desired_vel = prep.getDesiredVel(t-prep_t0);
          break;

        case TAKINGOFF_CIRCULAR:
          if (prep_t1 == 0) prep_t1 = t;
          desired_vel = takingoff.getDesiredVel(t-prep_t1);
          break;

        case FINISHED:
          std::cout << "(f)" << std::endl;
          running = false;
          franka::CartesianVelocities output = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
          return franka::MotionFinished(output);
      }

      // log data
       if (print_data.mutex.try_lock()) {
        print_data.has_data = true;
        print_data.robot_state = robot_state;        
        print_data.mutex.unlock();
      }

      // Send desired vel
      vel_desired.O_dP_EE[0] = desired_vel[0];
      vel_desired.O_dP_EE[1] = desired_vel[1];
      return vel_desired;
    };

    robot.control(cartesian_vel_callback);
    // robot.control(cartesian_vel_callback, franka::ControllerMode::kCartesianImpedance);
    // robot.control(cartesian_vel_callback, franka::ControllerMode::kCartesianImpedance, false, 1000.0);
    // robot.control(cartesian_vel_callback, franka::ControllerMode::kJointImpedance, true, 10);
  } catch (const franka::Exception& ex) {
    std::cerr << ex.what() << std::endl;
  }
  if (print_thread.joinable()) {
    print_thread.join();
  }
  return 0;
}

double getVelocity(const std::array<double, 3> vel){
  return std::sqrt(vel[0]*vel[0] + vel[1]*vel[1] + vel[2]*vel[2]);
}