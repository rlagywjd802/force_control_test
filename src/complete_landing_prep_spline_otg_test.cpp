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

#include "examples_common.h"
#include "landing_motion.h"
#include "prep_motion.h"
#include "cartesian_pose_log.h"

#define FCI_IP "192.168.30.151"
#define DT 0.001
#define D2R M_PI/180.0
#define NUMBER_OF_DOFS 3

enum MotionStage {
    LANDING_CIRCULAR,
    PREP_SPLINE,
    OTG,
    PREP_AFTER_SPLINE,
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
  const double kPrepVel = 0.005;
  std::array<double, 3> prep_ref = {kPrepRefX, kPrepRefY, kPrepRefZ};
  std::string prep_file_path = "/home/hj/Documents/MATLAB/data/prep.csv";

  // landing motion parameters
  const double kLandingAccTime = 1.5;
  const double kLandingR = 0.002;
  const char kLandingDir[4] = "www";  // 1-cw(www), 2-ccw(ccc)
  const enum MotionStage kLandingType = LANDING_CIRCULAR;

  // Prep Motion
  SplinePrep prep(prep_file_path, kPrepVel, prep_ref);
  std::array<std::array<double, 3>, 3> prep_start = prep.getPrepStart();
  std::array<std::array<double, 3>, 3> prep_end = prep.getPrepEnd();
  double prep_runtime = prep.getRuntime();
  // prep.printMotionParameter();

  // Landing Motion
  CircularLanding landing(kLandingAccTime, kLandingR, kLandingDir, prep_start, true); 
  landing.printMotionParameter();

  // OTG
  float kStartOTGTime = 1.0;
  float kDX = -0.002;
  float kDY = 0.002;
  std::array<double, 3> current_pose; 
  std::array<double, 3> current_vel; 
  std::array<double, 3> current_acc; 
  current_pose = prep.getDesiredPose(kStartOTGTime);
  current_vel = prep.getDesiredVel(kStartOTGTime);
  current_acc = prep.getDesiredAcc(kStartOTGTime);
  std::array<double, 3> target_pose = {current_pose[0]+kDX, current_pose[1]+kDY, current_pose[2]};
  std::array<double, 3> target_vel = current_vel;
  
  std::cout << "---------------------------------------------------------" << std::endl;
  std::cout << "current_pose : " << current_pose[0] << "," << current_pose[1] << "," << current_pose[2] << std::endl;
  std::cout << "current_vel : " << current_vel[0] << "," << current_vel[1] << "," << current_vel[2] << std::endl;
  std::cout << "current_acc : " << current_acc[0] << "," << current_acc[1] << "," << current_acc[2] << std::endl;
  std::cout << "target_pose : " << target_pose[0] << "," << target_pose[1] << "," << target_pose[2] << std::endl;
  std::cout << "target_vel : " << target_vel[0] << "," << target_vel[1] << "," << target_vel[2] << std::endl;
  
  std::array<double, 3> prep_after_ref = {kPrepRefX+kDX, kPrepRefY+kDY, kPrepRefZ};
  SplinePrep prep_after(prep_file_path, kPrepVel, prep_after_ref);
  std::array<std::array<double, 3>, 3> prep_after_start = prep_after.getPrepStart();
  std::array<std::array<double, 3>, 3> prep_after_end = prep_after.getPrepEnd();
  double prep_after_runtime = prep_after.getRuntime();

  // taking off  
  CircularLanding takingoff(kLandingAccTime, kLandingR, kLandingDir, prep_after_end, false); 
  takingoff.printMotionParameter();

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


    // ********************************************************************
    // Variable declarations and definitions
    int                         ResultValue                 =   0       ;
    int                         ResultValue_t               =   0       ;
    float                       t                           =   0.0     ;

    unsigned int                i                           =   0
                            ,   j                           =   0       ;

    ReflexxesAPI                *RML                        =   NULL    ;

    RMLPositionInputParameters  *IP                         =   NULL    ;

    RMLPositionOutputParameters *OP                         =   NULL    ;
    RMLPositionOutputParameters *OP_t                       =   NULL    ;

    RMLPositionFlags            Flags                                   ;

    // ********************************************************************
    // Creating all relevant objects of the Type II Reflexxes Motion Library

    RML = new ReflexxesAPI(NUMBER_OF_DOFS, DT);
    IP = new RMLPositionInputParameters(NUMBER_OF_DOFS);
    OP = new RMLPositionOutputParameters(NUMBER_OF_DOFS);
    OP_t= new RMLPositionOutputParameters(NUMBER_OF_DOFS);

    // ********************************************************************
    // Set-up the input parameters

    IP->CurrentPositionVector->VecData      [0] = current_pose[0];
    IP->CurrentPositionVector->VecData      [1] = current_pose[1];
    IP->CurrentPositionVector->VecData      [2] = current_pose[2];

    IP->CurrentVelocityVector->VecData      [0] = current_vel[0];
    IP->CurrentVelocityVector->VecData      [1] = current_vel[1];
    IP->CurrentVelocityVector->VecData      [2] = current_vel[2];

    IP->CurrentAccelerationVector->VecData  [0] = current_acc[0];
    IP->CurrentAccelerationVector->VecData  [1] = current_acc[1];
    IP->CurrentAccelerationVector->VecData  [2] = current_acc[2];

    IP->MaxVelocityVector->VecData          [0] =    0.5      ;
    IP->MaxVelocityVector->VecData          [1] =    0.5      ;
    IP->MaxVelocityVector->VecData          [2] =    0.5      ;

    IP->MaxAccelerationVector->VecData      [0] =    0.5      ;
    IP->MaxAccelerationVector->VecData      [1] =    0.5      ;
    IP->MaxAccelerationVector->VecData      [2] =    0.5      ;

    IP->MaxJerkVector->VecData              [0] =    1.0      ;
    IP->MaxJerkVector->VecData              [1] =    1.0      ;
    IP->MaxJerkVector->VecData              [2] =    1.0      ;

    IP->TargetPositionVector->VecData       [0] = target_pose[0];
    IP->TargetPositionVector->VecData       [1] = target_pose[1];
    IP->TargetPositionVector->VecData       [2] = target_pose[2];

    IP->TargetVelocityVector->VecData       [0] = target_vel[0];
    IP->TargetVelocityVector->VecData       [1] = target_vel[1];
    IP->TargetVelocityVector->VecData       [2] = target_vel[2];

    IP->SelectionVector->VecData            [0] =   true        ;
    IP->SelectionVector->VecData            [1] =   true        ;
    IP->SelectionVector->VecData            [2] =   false        ;

    // ********************************************************************
    // Checking the input parameters (optional)

    if (IP->CheckForValidity())
    {
        printf("Input values are valid!\n");
    }
    else
    {
        printf("Input values are INVALID!\n");
    }

    // Calling the Reflexxes OTG algorithm
    ResultValue = RML->RMLPosition(*IP, OP, Flags);

    if (ResultValue < 0)
    {
      printf("An error occurred (%d).\n", ResultValue);
      return 0;
    }

    // ****************************************************************
    // The following part completely describes all output values
    // of the Reflexxes Type II Online Trajectory Generation
    // algorithm.
    printf("-------------------------------------------------------\n");
    printf("General information:\n\n");
    printf("The execution time of the current trajectory is %.3lf seconds.\n", OP->GetSynchronizationTime());

    float otg_runtime = OP->GetSynchronizationTime();

    if (OP->IsTrajectoryPhaseSynchronized())
    {
        printf("The current trajectory is phase-synchronized.\n");
    }  
    else
    {
        printf("The current trajectory is time-synchronized.\n");
    }
    if (OP->WasACompleteComputationPerformedDuringTheLastCycle())
    {
        printf("The trajectory was computed during the last computation cycle.\n");
    }
    else
    {
        printf("The input values did not change, and a new computation of the trajectory parameters was not required.\n");
    }

    printf("-------------------------------------------------------\n");
    printf("New state of motion:\n\n");
    printf("New position/pose vector                  : ");
    for ( j = 0; j < NUMBER_OF_DOFS; j++)
    {
        printf("%10.3lf ", OP->NewPositionVector->VecData[j]);
    }
    printf("\n");
    printf("New velocity vector                       : ");
    for ( j = 0; j < NUMBER_OF_DOFS; j++)
    {
        printf("%10.3lf ", OP->NewVelocityVector->VecData[j]);
    }
    printf("\n");
    printf("New acceleration vector                   : ");
    for ( j = 0; j < NUMBER_OF_DOFS; j++)
    {
        printf("%10.3lf ", OP->NewAccelerationVector->VecData[j]);
    }
    printf("\n");
    printf("-------------------------------------------------------\n");
    printf("Extremes of the current trajectory:\n");
    for ( i = 0; i < NUMBER_OF_DOFS; i++)
    {
      printf("\n");
      printf("Degree of freedom                         : %d\n", i);
      printf("Minimum position                          : %10.3lf\n", OP->MinPosExtremaPositionVectorOnly->VecData[i]);
      printf("Time, at which the minimum will be reached: %10.3lf\n", OP->MinExtremaTimesVector->VecData[i]);
      printf("Position/pose vector at this time         : ");
      for ( j = 0; j < NUMBER_OF_DOFS; j++)
      {
          printf("%10.3lf ", OP->MinPosExtremaPositionVectorArray[i]->VecData[j]);
      }
      printf("\n");
      printf("Velocity vector at this time              : ");
      for ( j = 0; j < NUMBER_OF_DOFS; j++)
      {
          printf("%10.3lf ", OP->MinPosExtremaVelocityVectorArray[i]->VecData[j]);
      }
      printf("\n");
      printf("Acceleration vector at this time          : ");
      for ( j = 0; j < NUMBER_OF_DOFS; j++)
      {
          printf("%10.3lf ", OP->MinPosExtremaAccelerationVectorArray[i]->VecData[j]);
      }
      printf("\n");
      printf("Maximum position                          : %10.3lf\n", OP->MaxPosExtremaPositionVectorOnly->VecData[i]);
      printf("Time, at which the maximum will be reached: %10.3lf\n", OP->MaxExtremaTimesVector->VecData[i]);
      printf("Position/pose vector at this time         : ");
      for ( j = 0; j < NUMBER_OF_DOFS; j++)
      {
          printf("%10.3lf ", OP->MaxPosExtremaPositionVectorArray[i]->VecData[j]);
      }
      printf("\n");
      printf("Velocity vector at this time              : ");
      for ( j = 0; j < NUMBER_OF_DOFS; j++)
      {
          printf("%10.3lf ", OP->MaxPosExtremaVelocityVectorArray[i]->VecData[j]);
      }
      printf("\n");
      printf("Acceleration vector at this time          : ");
      for ( j = 0; j < NUMBER_OF_DOFS; j++)
      {
          printf("%10.3lf ", OP->MaxPosExtremaAccelerationVectorArray[i]->VecData[j]);
      }
      printf("\n");
    }
    printf("-------------------------------------------------------\n");

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
    enum MotionStage stage = kLandingType;
    double t = 0.0;    

    // prep circle
    double prep_t0 = 0.0;
    double prep_t1 = 0.0;
    double prep_t2 = 0.0;
    double otg_t0 = 0.0;
    double diff_x = 0.0;
    double diff_y = 0.0;
    
    // Define callback function to send Cartesian pose goals to get inverse kinematics solved.
    auto cartesian_vel_callback = [=, &running, &print_data, &t, &stage, &desired_vel, &diff_x, &diff_y, &prep_t0, &prep_t1, &prep_t2, &otg_t0](
                                       const franka::RobotState& robot_state,
                                       franka::Duration period) -> franka::CartesianVelocities {
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
      if (t >= kLandingAccTime+kStartOTGTime && stage == PREP_SPLINE){
        stage = OTG;
        std::cout << "switch to OTG" << std::endl;
      }
      if (t >= kLandingAccTime+kStartOTGTime+otg_runtime && stage == OTG){
        stage = PREP_AFTER_SPLINE;
        std::cout << "switch back to Prep" << std::endl;
      }
      if (t >= kLandingAccTime+otg_runtime+prep_runtime && stage == PREP_AFTER_SPLINE){
        stage = TAKINGOFF_CIRCULAR;
        std::cout << "switch to taking off" << std::endl;
      }
      if (t >= 2*kLandingAccTime+prep_runtime+otg_runtime){
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

        case OTG:
          if (otg_t0 == 0) otg_t0 = t;
          RML->RMLPositionAtAGivenSampleTime(t-otg_t0, OP_t);
          desired_vel[0] = OP_t->NewVelocityVector->VecData[0];
          desired_vel[1] = OP_t->NewVelocityVector->VecData[1];
          break;

        case PREP_AFTER_SPLINE:
          if (prep_t1 == 0) prep_t1 = t;
          desired_vel = prep_after.getDesiredVel(t-prep_t1 + (otg_t0-prep_t0));
          break;

        case TAKINGOFF_CIRCULAR:
          if (prep_t2 == 0) prep_t2 = t;
          desired_vel = takingoff.getDesiredVel(t-prep_t2);
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
        if (stage == LANDING_CIRCULAR) {
          std::array<double, 3> landing_x = landing.getDesiredPose(t);
          print_data.des_x = landing_x[0];
          print_data.des_y = landing_x[1];
        }
        else if (stage == PREP_SPLINE){
          std::array<double, 3> prep_x = prep.getDesiredPose(t-prep_t0);
          print_data.des_x = prep_x[0];
          print_data.des_y = prep_x[1];
        }
        else if (stage == OTG){
          RML->RMLPositionAtAGivenSampleTime(t-otg_t0, OP_t);
          print_data.des_x = OP_t->NewPositionVector->VecData[0];
          print_data.des_y = OP_t->NewPositionVector->VecData[1];
        }
        else if (stage == PREP_AFTER_SPLINE){
          std::array<double, 3> prep_x = prep_after.getDesiredPose(t-prep_t1 + (otg_t0-prep_t0));
          print_data.des_x = prep_x[0];
          print_data.des_y = prep_x[1];
        }
        else if (stage == TAKINGOFF_CIRCULAR){
          std::array<double, 3> takingoff_x = takingoff.getDesiredPose(t-prep_t2);
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
    // robot.control(cartesian_vel_callback, franka::ControllerMode::kJointImpedance, false, 1000.0);
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
  

  // ********************************************************************
  // Deleting the objects of the Reflexxes Motion Library end terminating
  // the process

  delete  RML         ;
  delete  IP          ;
  delete  OP          ;
  delete  OP_t        ;

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