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

#include <Eigen/Dense>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/WrenchStamped.h>
#include <visualization_msgs/Marker.h>

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
  const double kPrepRefX = 0.3;
  const double kPrepRefY = 0.0;
  const double kPrepRefZ = 0.45;
  const double kPrepVel = 0.001; //0.005;
  std::array<double, 3> prep_ref = {kPrepRefX, kPrepRefY, kPrepRefZ};
  std::string prep_file_path1 = "/home/hj/Documents/MATLAB/data/prep_lower_layer_1.csv";
  std::string prep_file_path2 = "/home/hj/Documents/MATLAB/data/prep_lower_layer_2.csv";
  std::string prep_file_path3 = "/home/hj/Documents/MATLAB/data/prep_lower_layer_3.csv";
  std::array<std::string, NLayer> prep_file_paths = {prep_file_path1, prep_file_path2, prep_file_path3};

  // landing motion parameters
  const double kLandingAccTime = 5.0*(0.001/kPrepVel); //1;
  const double kLandingR = 0.002;
  const char kLandingDir[4] = "www";  // 1-cw(www), 2-ccw(ccc)
  const enum MotionStage kLandingType = LANDING_CIRCULAR;

  // sliding motion paramters
  const int kSlidingVpts = 50; //50;
  const double kSlidingTime = 1.5*(0.001/kPrepVel); //0.2;

  // Prep Motion
  MultipleSplinePrep prep(prep_file_paths, kPrepVel, kSlidingVpts, kSlidingTime, prep_ref);
  std::array<std::array<double, 3>, 3> prep_start = prep.getPrepStart();
  std::array<std::array<double, 3>, 3> prep_end = prep.getPrepEnd();
  double prep_runtime = prep.getRuntime();
  // prep.printMotionParameter();

  // Landing Motion
  CircularLanding landing(kLandingAccTime, kLandingR, kLandingDir, prep_start, true); 
  CircularLanding takingoff(kLandingAccTime, kLandingR, kLandingDir, prep_end, false); 
  landing.printMotionParameter();
  takingoff.printMotionParameter();

  // ros
  ros::init(argc, argv, "cartesian_impedance_control_test");
  ros::NodeHandle nh;
  ros::Publisher ft_pub = nh.advertise<geometry_msgs::WrenchStamped>("ft_data", 1);
  ros::Publisher vis_pub = nh.advertise<visualization_msgs::Marker>("pose_marker", 0);

  // Initialize data fields for the print thread.
  struct {
    std::mutex mutex;
    bool has_data;
    double des_x;
    double des_y;
    double des_z;
    franka::RobotState robot_state;
  } print_data{};
  std::atomic_bool running{true};

  // Set print rate and thread related parameters
  const double print_rate = 100.0;
  int ft_seq = 0;

  // Start print thread.
  std::thread print_thread([print_rate, &print_data, &running, &ft_seq, &ft_pub, &vis_pub]() {
    while (running) {
      // Sleep to achieve the desired print rate.
      std::this_thread::sleep_for(
          std::chrono::milliseconds(static_cast<int>((1.0 / print_rate * 1000.0))));
      
      // Try to lock data to avoid read write collisions.
      if (print_data.mutex.try_lock()) {
        if (print_data.has_data) {
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

          visualization_msgs::Marker marker;
          marker.header.frame_id = "world";
          marker.header.stamp = ros::Time();
          marker.ns = "my_namespace";
          marker.id = 0;
          marker.type = visualization_msgs::Marker::SPHERE;
          marker.action = visualization_msgs::Marker::ADD;
          marker.pose.position.x = print_data.robot_state.O_T_EE[12];
          marker.pose.position.y = print_data.robot_state.O_T_EE[13];
          marker.pose.position.z = print_data.robot_state.O_T_EE[14]; //print_data.des_z;
          marker.pose.orientation.x = 0.0;
          marker.pose.orientation.y = 0.0;
          marker.pose.orientation.z = 0.0;
          marker.pose.orientation.w = 1.0;
          marker.scale.x = 0.001;
          marker.scale.y = 0.001;
          marker.scale.z = 0.001;
          marker.color.a = 1.0;
          marker.color.r = 0.0;
          marker.color.g = 1.0;
          marker.color.b = 0.0;
          vis_pub.publish(marker);

          // 
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

    // First move the robot to start point of landing motion
    std::array<double, 3> x_goal = landing.getDesiredPose(0.0);
    CMotionGenerator motion_generator(0.2, x_goal);
    std::cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    robot.control(motion_generator);
    std::cout << "Finished moving to initial joint configuration." << std::endl;

    // Next, ..
    std::cout << "LANDING START---------------" << std::endl;    
    
    // Set additional parameters always before the control loop, NEVER in the control loop!
    // Set collision behavior.
    // robot.setCollisionBehavior(
    //     {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
    //     {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
    //     {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
    //     {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});
    
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
    auto cartesian_vel_callback = [=, &running, &print_data, &t, &stage, &desired_vel, &diff_x, &diff_y, &is_prep, &prep_t0, &prep_t1](
                                       const franka::RobotState& robot_state,
                                       franka::Duration period) -> franka::CartesianVelocities {
      // Update time.
      t += period.toSec();

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
          if (!is_prep){
            is_prep = true;
            prep_t0 = t;
          }
            
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
          return franka::MotionFinished(output);  // 꼭 vel_desired 넘겨줘야 되나????
      }

      // log data
       if (print_data.mutex.try_lock()) {
        print_data.has_data = true;
        print_data.robot_state = robot_state;
        if (stage == LANDING_CIRCULAR) {
          std::array<double, 3> landing_x = landing.getDesiredPose(t);
          print_data.des_x = landing_x[0];
          print_data.des_y = landing_x[1];
          print_data.des_z = landing_x[2];
        }
        else if (stage == PREP_SPLINE){
          std::array<double, 3> prep_x = prep.getDesiredPose(t-prep_t0);
          print_data.des_x = prep_x[0];
          print_data.des_y = prep_x[1];
          print_data.des_z = prep_x[2];
        }
        else if (stage == TAKINGOFF_CIRCULAR){
          std::array<double, 3> takingoff_x = takingoff.getDesiredPose(t-prep_t1);
          print_data.des_x = takingoff_x[0];
          print_data.des_y = takingoff_x[1]; 
          print_data.des_z = takingoff_x[2]; 
        }
        
        print_data.mutex.unlock();
      }

      // Send desired vel
      vel_desired.O_dP_EE[0] = desired_vel[0];
      vel_desired.O_dP_EE[1] = desired_vel[1];
      vel_desired.O_dP_EE[2] = desired_vel[2];
      return vel_desired;
    };

    // robot.control(cartesian_vel_callback);
    robot.control(cartesian_vel_callback, franka::ControllerMode::kJointImpedance, false, 1000.0);
    // robot.control(cartesian_vel_callback, franka::ControllerMode::kJointImpedance, true, 10);
  } catch (const franka::ControlException& e) {
    std::cout << e.what() << std::endl;
    running = false;
    writeLogToFile(e.log);
    return -1;

  } catch (const franka::Exception& ex) {
    std::cerr << ex.what() << std::endl;
  }
  if (print_thread.joinable()) {
    print_thread.join();
  }
  return 0;
}

void writeLogToFile(const std::vector<franka::Record>& log) {
  if (log.empty()) {
    return;
  }
  try {
    Poco::Path temp_dir_path(Poco::Path::temp());
    temp_dir_path.pushDirectory("libfranka-logs");
    Poco::File temp_dir(temp_dir_path);
    temp_dir.createDirectories();
    std::string now_string =
        Poco::DateTimeFormatter::format(Poco::Timestamp{}, "%Y-%m-%d-%h-%m-%S-%i");
    std::string filename = std::string{"log-" + now_string + ".csv"};
    Poco::File log_file(Poco::Path(temp_dir_path, filename));
    if (!log_file.createFile()) {
      std::cout << "Failed to write log file." << std::endl;
      return;
    }
    std::ofstream log_stream(log_file.path().c_str());
    log_stream << CartesianPose::logToCSV(log);
    std::cout << "Log file written to: " << log_file.path() << std::endl;
  } catch (...) {
    std::cout << "Failed to write log file." << std::endl;
  }
}

double getVelocity(const std::array<double, 3> vel){
  return std::sqrt(vel[0]*vel[0] + vel[1]*vel[1] + vel[2]*vel[2]);
}