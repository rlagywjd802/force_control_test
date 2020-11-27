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

#include <ReflexxesAPI.h>
#include <RMLPositionFlags.h>
#include <RMLPositionInputParameters.h>
#include <RMLPositionOutputParameters.h>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <force_control_test/FrankaState.h>

#include "examples_common.h"
#include "landing_motion.h"
#include "prep_motion.h"
#include "cartesian_pose_log.h"
#include "target_update.h"

#define FCI_IP "192.168.30.151"
#define DT 0.001
#define D2R M_PI/180.0
#define NUMBER_OF_DOFS 3

enum MotionStage {
    LANDING=0,
    PREP_SPLINE,
    OTG,
    TAKINGOFF,
    FINISHED
};

double getVelocity(const std::array<double, 3> vel);
void writeLogToFile(const std::vector<franka::Record>& log);

int main(int argc, char** argv) {
  
  // prep motion parameters
  const double kPrepOX = 0.634896;
  const double kPrepOY = -0.022188;
  const double kPrepOZ = 0.3130; // 0.3140;
  const double kPrepR = 0.0203; // 0.0202;
  const double kPrepW = 10*D2R;
  const double kPrepTh0 = 0*D2R;
  const double kPrepThf = 360*D2R;
  std::array<double, 3> prep_center_of_circle = {kPrepOX, kPrepOY, kPrepOZ}; // Point O

  // landing motion parameters
  const double kLandingAccTime = 1.5;
  const double kLandingR = 0.002;
  const char kLandingDir[4] = "www";  // 1-cw(www), 2-ccw(ccc)

  // ros
  ros::init(argc, argv, "otg_target_update");
  ros::NodeHandle nh;
  ros::Publisher etc_pub = nh.advertise<force_control_test::FrankaState>("franka", 1);
  ros::Publisher joint_states_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
  sensor_msgs::JointState msg;
  msg.name = {"panda_joint1", "panda_joint2", "panda_joint3", 
              "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"};

  // Prep Motion
  CircularPrep prep(kPrepR, kPrepW, kPrepTh0, kPrepThf, prep_center_of_circle);
  std::array<std::array<double, 3>, 3> prep_start = prep.getPrepStart();
  std::array<std::array<double, 3>, 3> prep_end = prep.getPrepEnd();
  double prep_runtime = prep.getRuntime();
  std::cout << prep_runtime << "s";

  // Landing Motion
  CircularLanding landing(kLandingAccTime, kLandingR, kLandingDir, prep_start, true); 
  landing.printMotionParameter();

  // OTG
  const int kOTGDuration = 1000;  
  double max_vel[3] = {0.1, 0.1, 0.1}; 
  double max_acc[3] = {0.1, 0.1, 0.1}; 
  double max_jerk[3] = {0.01, 0.01, 0.01};
  // double max_jerk[] = {1, 1, 1};
  bool selection_vec[3] = {true, true, false};
  
  int ResultValue = 0;
  ReflexxesAPI *RML = NULL;
  RMLPositionInputParameters *IP = NULL;
  RMLPositionOutputParameters *OP = NULL;
  RMLPositionFlags Flags;
  
  RML = new ReflexxesAPI(NUMBER_OF_DOFS, DT);
  IP = new RMLPositionInputParameters(NUMBER_OF_DOFS);
  OP = new RMLPositionOutputParameters(NUMBER_OF_DOFS);

  // IP->MaxVelocityVector->VecData = max_vel;
  // IP->MaxAccelerationVector->VecData = max_acc;
  // IP->MaxJerkVector->VecData = max_jerk;
  // IP->SelectionVector->VecData = selection_vec;

  IP->MaxVelocityVector->VecData[0] = max_vel[0];
  IP->MaxVelocityVector->VecData[1] = max_vel[1];
  IP->MaxVelocityVector->VecData[2] = max_vel[2];
  IP->MaxAccelerationVector->VecData[0] = max_acc[0];
  IP->MaxAccelerationVector->VecData[1] = max_acc[1];
  IP->MaxAccelerationVector->VecData[2] = max_acc[2];
  IP->MaxJerkVector->VecData[0] = max_jerk[0];
  IP->MaxJerkVector->VecData[1] = max_jerk[1];
  IP->MaxJerkVector->VecData[2] = max_jerk[2];
  IP->SelectionVector->VecData[0] = selection_vec[0];
  IP->SelectionVector->VecData[1] = selection_vec[1];
  IP->SelectionVector->VecData[2] = selection_vec[2];

  // taking off  
  CircularLanding takingoff(kLandingAccTime, kLandingR, kLandingDir, prep_end, false); 
  takingoff.printMotionParameter();

  // Target
  TargetUpdate target(prep_center_of_circle);
  ros::spinOnce(); // wait for subscriber

  // Set print rate
  const double print_rate = 5.0;

  // Initialize data fields for the print thread.
  struct {
    std::mutex mutex;
    bool has_data;
    franka::RobotState robot_state;
    double des_x;
    double des_y;
    bool is_otg;
    std::array<double, 3> desired_pose;
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
          force_control_test::FrankaState msg;

          msg.position.push_back(print_data.robot_state.O_T_EE[12]);
          msg.position.push_back(print_data.robot_state.O_T_EE[13]);
          msg.position.push_back(print_data.robot_state.O_T_EE[14]);
          
          if (print_data.is_otg == true){
            msg.position.push_back(print_data.desired_pose[0]);
            msg.position.push_back(print_data.desired_pose[1]);
            msg.position.push_back(print_data.desired_pose[2]);  
          }
          else {
            msg.position.push_back(print_data.robot_state.O_T_EE[12]);
            msg.position.push_back(print_data.robot_state.O_T_EE[13]);
            msg.position.push_back(print_data.robot_state.O_T_EE[14]);
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
       
    // Define initial variable for motion callback.
    std::array<double, 3> desired_vel;
    int stage = LANDING;
    double t = 0.0;
    double prep_t = 0.0;
    double takingoff_t0 = 0.0;

    bool is_otg_started = false;
    double* new_ref;
    
    // Define callback function to send Cartesian pose goals to get inverse kinematics solved.
    auto cartesian_vel_callback = [=, &running, &print_data, &target, &stage, &t, &prep_t, &takingoff_t0, 
                                    &prep, &takingoff, &is_otg_started, &RML, &IP, &OP, &ResultValue, &msg,  
                                    &desired_vel, &new_ref](const franka::RobotState& robot_state, 
                                    franka::Duration period) -> franka::CartesianVelocities 
    {
      // Update time.
      t += period.toSec();

      // Decide motion stage
      if (stage == LANDING && t >= kLandingAccTime) {
        stage = PREP_SPLINE;
        std::cout << "switch to prep" << std::endl;
      }
      ///////////////////////////////////////////////////////////////////
      if (stage == PREP_SPLINE) {
        if (prep_t > 0.5 && target.getUpdated()) {
          stage = OTG;
          std::cout << "switch to OTG" << std::endl;
        }
        else {
          target.setUpdated(false);
        }
        if (prep_t >= prep_runtime){
        stage = TAKINGOFF;
        std::cout << "switch to taking off" << std::endl;
        }
      }
      ///////////////////////////////////////////////////////////////////
      if (stage == OTG) {
        if (!is_otg_started) {
          new_ref = target.getTargetValue();
          std::array<double, 3> current_pose; 
          std::array<double, 3> current_vel; 
          std::array<double, 3> current_acc;
          std::array<double, 3> target_pose; 
          std::array<double, 3> target_vel; 

          current_pose = prep.getDesiredPose(prep_t);
          current_vel = prep.getDesiredVel(prep_t);
          current_acc = prep.getDesiredAcc(prep_t);
          
          prep.applyRefTransform({new_ref[0], new_ref[1], kPrepOZ});
          takingoff.applyRefTransform({new_ref[0], new_ref[1], kPrepOZ});
          target_pose = prep.getDesiredPose(prep_t);
          target_vel = prep.getDesiredVel(prep_t);

          // Set-up the input parameters
          IP->CurrentPositionVector->VecData[0] = current_pose[0];
          IP->CurrentPositionVector->VecData[1] = current_pose[1];
          IP->CurrentPositionVector->VecData[2] = current_pose[2];
          IP->CurrentVelocityVector->VecData[0] = current_vel[0];
          IP->CurrentVelocityVector->VecData[1] = current_vel[1];
          IP->CurrentVelocityVector->VecData[2] = current_vel[2];
          IP->CurrentAccelerationVector->VecData[0] = current_acc[0];
          IP->CurrentAccelerationVector->VecData[1] = current_acc[1];
          IP->CurrentAccelerationVector->VecData[2] = current_acc[2];
          IP->TargetPositionVector->VecData[0] = target_pose[0];
          IP->TargetPositionVector->VecData[1] = target_pose[1];
          IP->TargetPositionVector->VecData[2] = target_pose[2];
          IP->TargetVelocityVector->VecData[0] = target_vel[0];
          IP->TargetVelocityVector->VecData[1] = target_vel[1];
          IP->TargetVelocityVector->VecData[2] = target_vel[2];

          is_otg_started = true;
          std::cout << "---------------------------------------------------------" << std::endl;
          // std::cout << "current_pose : " << current_pose[0] << "," << current_pose[1] << "," << current_pose[2] << std::endl;
          // std::cout << "current_vel : " << current_vel[0] << "," << current_vel[1] << "," << current_vel[2] << std::endl;
          // std::cout << "current_acc : " << current_acc[0] << "," << current_acc[1] << "," << current_acc[2] << std::endl;
          // std::cout << "target_pose : " << target_pose[0] << "," << target_pose[1] << "," << target_pose[2] << std::endl;
          // std::cout << "target_vel : " << target_vel[0] << "," << target_vel[1] << "," << target_vel[2] << std::endl;
        }
        else {
          *IP->CurrentPositionVector = *OP->NewPositionVector;
          *IP->CurrentVelocityVector = *OP->NewVelocityVector;
          *IP->CurrentAccelerationVector = *OP->NewAccelerationVector;
        }
        
        ResultValue = RML->RMLPosition(*IP, OP, Flags);
        if (ResultValue < 0)  {
          std::cout << "An error occurred (" << ResultValue << ")." << std::endl;
          stage = FINISHED;
        }
        else if (ResultValue == ReflexxesAPI::RML_FINAL_STATE_REACHED) {
          std::cout << "Final State Reached." << std::endl;
          target.setUpdated(false);
          is_otg_started = false;
          stage = PREP_SPLINE;
        }
        // else {
        //   printf("The execution time of the current trajectory is %.3lf seconds.\n", OP->GetSynchronizationTime());
        // }
      }
      ///////////////////////////////////////////////////////////////////
      if (stage == TAKINGOFF && (t-takingoff_t0) >= kLandingAccTime){
        stage = FINISHED;
        std::cout << "time to finish" << std::endl;
      }     

      // get desired value
      franka::CartesianVelocities vel_desired = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
      switch (stage) {
        case LANDING:
          desired_vel = landing.getDesiredVel(t);
          break;
        
        case PREP_SPLINE:
          prep_t += period.toSec();
          desired_vel = prep.getDesiredVel(prep_t);
          break;

        case OTG:
          desired_vel[0] = OP->NewVelocityVector->VecData[0];
          desired_vel[1] = OP->NewVelocityVector->VecData[1];
          break;

        case TAKINGOFF:
          if (takingoff_t0 == 0) takingoff_t0 = t;
          desired_vel = takingoff.getDesiredVel(t-takingoff_t0);
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
        if (stage == LANDING) {
          std::array<double, 3> landing_x = landing.getDesiredPose(t);
          print_data.des_x = landing_x[0];
          print_data.des_y = landing_x[1];
          print_data.is_otg = false;
        }
        else if (stage == PREP_SPLINE){
          std::array<double, 3> prep_x = prep.getDesiredPose(prep_t);
          print_data.des_x = prep_x[0];
          print_data.des_y = prep_x[1];
          print_data.is_otg = false;
        }
        else if (stage == OTG){
          print_data.des_x = OP->NewPositionVector->VecData[0];
          print_data.des_y = OP->NewPositionVector->VecData[1];
          std::array<double, 3> desired_pose;
          for (int i=0; i<3; i++)
            desired_pose[i] = new_ref[i];

          print_data.desired_pose = desired_pose;
          print_data.is_otg = true;
        }
        else if (stage == TAKINGOFF){
          std::array<double, 3> takingoff_x = takingoff.getDesiredPose(t-takingoff_t0);
          print_data.des_x = takingoff_x[0];
          print_data.des_y = takingoff_x[1];
          print_data.is_otg = false; 
        }
        print_data.mutex.unlock();
      }

      // wait for subscriber
      ros::spinOnce();

      // Publish joint state
      std::array< double, 7 > measured_q = robot_state.q;
      msg.header.stamp = ros::Time::now();
      msg.position = {measured_q[0], measured_q[1], measured_q[2], measured_q[3], measured_q[4], measured_q[5], measured_q[6]};
      // msg.velocity = {};
      // msg.effort = {};
      joint_states_pub.publish(msg);


      // Send desired vel
      vel_desired.O_dP_EE[0] = desired_vel[0];
      vel_desired.O_dP_EE[1] = desired_vel[1];
      vel_desired.O_dP_EE[2] = desired_vel[2];
      return vel_desired;
    };

    robot.control(cartesian_vel_callback);
  
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
  
  // ********************************************************************
  // Deleting the objects of the Reflexxes Motion Library end terminating
  // the process
  delete  RML         ;
  delete  IP          ;
  delete  OP          ;

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