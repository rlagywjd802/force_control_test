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

#include "examples_common.h"
#include "landing_motion.h"
#include "prep_motion.h"
#include "cartesian_pose_log.h"

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
  const double kPrepRefX = 0.3;
  const double kPrepRefY = 0.0;
  const double kPrepRefZ = 0.45;
  const double kPrepVel = 0.005;
  std::array<double, 3> prep_ref = {kPrepRefX, kPrepRefY, kPrepRefZ};
  std::string prep_file_path = "/home/hj/Documents/MATLAB/data/prep_loop.csv";

  // landing motion parameters
  const double kLandingAccTime = 1.5;
  const double kLandingR = 0.002;
  const char kLandingDir[4] = "www";  // 1-cw(www), 2-ccw(ccc)
  const int kLandingType = LANDING;

  // Prep Motion
  SplinePrep prep(prep_file_path, kPrepVel, prep_ref);
  std::array<std::array<double, 3>, 3> prep_start = prep.getPrepStart();
  std::array<std::array<double, 3>, 3> prep_end = prep.getPrepEnd();
  double prep_runtime = prep.getRuntime();

  // Landing Motion
  CircularLanding landing(kLandingAccTime, kLandingR, kLandingDir, prep_start, true); 
  landing.printMotionParameter();

  // OTG
  const int kOTGDuration = 1000;  
  double max_vel[] = {0.5, 0.5, 0.5}; 
  double max_acc[] = {0.5, 0.5, 0.5}; 
  double max_jerk[] = {0.1, 0.1, 0.1};
  // double max_jerk[] = {1, 1, 1};
  double selection_vec[] = {true, true, false};
  
  // taking off  
  CircularLanding takingoff(kLandingAccTime, kLandingR, kLandingDir, prep_end, false); 
  takingoff.printMotionParameter();

  // OTG
  int ResultValue = 0;
  ReflexxesAPI *RML = NULL;
  RMLPositionInputParameters *IP = NULL;
  RMLPositionOutputParameters *OP = NULL;
  RMLPositionFlags Flags;
  
  RML = new ReflexxesAPI(NUMBER_OF_DOFS, DT);
  IP = new RMLPositionInputParameters(NUMBER_OF_DOFS);
  OP = new RMLPositionOutputParameters(NUMBER_OF_DOFS);

  // Set print rate for comparing commanded vs. measured torques.
  const double print_rate = 100.0;

  // Initialize data fields for the print thread.
  struct {
    std::mutex mutex;
    bool has_data;
    franka::RobotState robot_state;
    double des_x;
    double des_y;
  } print_data{};
  std::atomic_bool running{true};

  // Start print thread.
  std::thread print_thread([print_rate, &print_data, &running]() {
    while (running) {
      // Sleep to achieve the desired print rate.
      std::this_thread::sleep_for(
          std::chrono::milliseconds(static_cast<int>((1.0 / print_rate * 1000.0))));
      
      // Try to lock data to avoid read write collisions.
      if (print_data.mutex.try_lock()) {
        if (print_data.has_data) {
          // Print data to console
          std::cout << print_data.robot_state.O_T_EE[12] << "," << print_data.robot_state.O_T_EE[13] << ","
                    << print_data.robot_state.O_dP_EE_c[0] << "," << print_data.robot_state.O_dP_EE_c[1] << ","
                    << print_data.robot_state.O_ddP_EE_c[0] << "," << print_data.robot_state.O_ddP_EE_c[1] << std::endl;
                    // << print_data.des_x << "," << print_data.des_y << std::endl;
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
    int stage = kLandingType;
    double t = 0.0;
    double prep_t = 0.0;
    double takingoff_t0 = 0.0;

    bool otg_flag = false;
    
    // Define callback function to send Cartesian pose goals to get inverse kinematics solved.
    auto cartesian_vel_callback = [=, &running, &print_data, &stage, &t, &prep_t, &takingoff_t0, 
                                    &prep, &takingoff, &otg_flag, &RML, &IP, &OP, &ResultValue, &desired_vel](
                                       const franka::RobotState& robot_state,
                                       franka::Duration period) -> franka::CartesianVelocities {
      // Update time.
      t += period.toSec();

      // Decide motion stage
      if (stage == LANDING && t >= kLandingAccTime) {
        stage = PREP_SPLINE;
        std::cout << "switch to prep" << std::endl;
      }
      else if (stage == PREP_SPLINE) {
        if (int(prep_t*1000) % kOTGDuration == 0.0) {
          stage = OTG;
          std::cout << "switch to OTG" << std::endl;
        }
      }
      else if (stage == OTG && ResultValue == ReflexxesAPI::RML_FINAL_STATE_REACHED) {
        stage = PREP_SPLINE;
        std::cout << "switch back to Prep" << std::endl;
      }
      else if (stage == PREP_SPLINE && prep_t >= prep_runtime){
        stage = TAKINGOFF;
        std::cout << "switch to taking off" << std::endl;
      }
      else if (stage == TAKINGOFF && (t-takingoff_t0) >= kLandingAccTime){
        stage = FINISHED;
        std::cout << "time to finish" << std::endl;
      }

      if (stage == OTG) {
        double random_dx = -0.002;
        double random_dy = 0.002;
        double random_dz = 0.0;
        std::array<double, 3> dref = {random_dx, random_dy, random_dz};
        if (otg_flag == false) {
          // calculate random target value
          std::array<double, 3> current_pose; 
          std::array<double, 3> current_vel; 
          std::array<double, 3> current_acc;
          double target_pose[3];
          double target_vel[3];

          current_pose = prep.getDesiredPose(prep_t);
          current_vel = prep.getDesiredVel(prep_t);
          current_acc = prep.getDesiredAcc(prep_t);
          
          // std::array<double, 3> target_pose;
          // std::array<double, 3> target_vel;
          target_pose[0] = current_pose[0] + random_dx;
          target_pose[1] = current_pose[1] + random_dy;
          target_pose[2] = current_pose[2] + random_dz;
          target_vel[0] = current_vel[0];
          target_vel[1] = current_vel[1];
          target_vel[2] = current_vel[2];
  
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
          
          // IP->MaxVelocityVector->VecData = max_vel;
          IP->MaxVelocityVector->VecData[0] = max_vel[0];
          IP->MaxVelocityVector->VecData[1] = max_vel[1];
          IP->MaxVelocityVector->VecData[2] = max_vel[2];
          IP->MaxAccelerationVector->VecData[0] = max_acc[0];
          IP->MaxAccelerationVector->VecData[1] = max_acc[1];
          IP->MaxAccelerationVector->VecData[2] = max_acc[2];
          IP->MaxJerkVector->VecData[0] = max_jerk[0];
          IP->MaxJerkVector->VecData[1] = max_jerk[1];
          IP->MaxJerkVector->VecData[2] = max_jerk[2];

          // IP->TargetPositionVector->VecData = target_pose;
          // IP->TargetVelocityVector->VecData = target_vel;
          IP->TargetPositionVector->VecData[0] = target_pose[0];
          IP->TargetPositionVector->VecData[1] = target_pose[1];
          IP->TargetPositionVector->VecData[2] = target_pose[2];
          IP->TargetVelocityVector->VecData[0] = target_vel[0];
          IP->TargetVelocityVector->VecData[1] = target_vel[1];
          IP->TargetVelocityVector->VecData[2] = target_vel[2];
          IP->SelectionVector->VecData[0] = selection_vec[0];
          IP->SelectionVector->VecData[1] = selection_vec[1];
          IP->SelectionVector->VecData[2] = selection_vec[2];

          std::cout << "---------------------------------------------------------" << std::endl;
          // std::cout << "current_pose : " << current_pose[0] << "," << current_pose[1] << "," << current_pose[2] << std::endl;
          // std::cout << "current_vel : " << current_vel[0] << "," << current_vel[1] << "," << current_vel[2] << std::endl;
          // std::cout << "current_acc : " << current_acc[0] << "," << current_acc[1] << "," << current_acc[2] << std::endl;
          // std::cout << "target_pose : " << target_pose[0] << "," << target_pose[1] << "," << target_pose[2] << std::endl;
          // std::cout << "target_vel : " << target_vel[0] << "," << target_vel[1] << "," << target_vel[2] << std::endl;
          // if (IP->CheckForValidity())
          // {
          //     printf("Input values are valid!\n");
          // }
          // else
          // {
          //     printf("Input values are INVALID!\n");
          // }
          // printf("The execution time of the current trajectory is %.3lf seconds.\n", OP->GetSynchronizationTime());

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
        else {
          if (otg_flag == false) {
            otg_flag = true;
            
            prep.applyTransform(dref);
            takingoff.applyTransform(dref);
          }
        }
      }
      else if (stage == PREP_SPLINE) {
        otg_flag = false;
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
        }
        else if (stage == PREP_SPLINE){
          std::array<double, 3> prep_x = prep.getDesiredPose(prep_t);
          print_data.des_x = prep_x[0];
          print_data.des_y = prep_x[1];
        }
        else if (stage == OTG){
          print_data.des_x = OP->NewPositionVector->VecData[0];
          print_data.des_y = OP->NewPositionVector->VecData[1];
        }
        else if (stage == TAKINGOFF){
          std::array<double, 3> takingoff_x = takingoff.getDesiredPose(t-takingoff_t0);
          print_data.des_x = takingoff_x[0];
          print_data.des_y = takingoff_x[1]; 
        }
        print_data.mutex.unlock();
      }

      // Send desired vel
      vel_desired.O_dP_EE[0] = desired_vel[0];
      vel_desired.O_dP_EE[1] = desired_vel[1];
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