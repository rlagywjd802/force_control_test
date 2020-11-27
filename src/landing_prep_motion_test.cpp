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
#include "landing_motion.h"
#include "prep_motion.h"
#include "cartesian_pose_log.h"

#define FCI_IP "192.168.30.151"
#define DT 0.001
#define D2R M_PI/180.0

enum MotionStage {
    LANDING_LINEAR = 0,
    LANDING_SMOOTH,
    LANDING_CIRCULAR,
    PREP_CIRCULAR,
    FINISHED
};

double getVelocity(const std::array<double, 3> vel);
void writeLogToFile(const std::vector<franka::Record>& log);

int main(int argc, char** argv) {
  
  // prep motion parameters
  const double kPrepRefX = 0.3;
  const double kPrepRefY = 0.0;
  const double kPrepRefZ = 0.45;
  const double kPrepR = 0.05;
  const double kPrepW = 60*D2R;
  const double kPrepTh0 = 60*D2R;
  const double kPrepThf = 180*D2R;
  std::array<double, 3> prep_ref = {kPrepRefX, kPrepRefY, kPrepRefZ};


  // landing motion parameters
  const double kLandingAccTime = 1.0;
  const double kLandingR = 0.05;
  const char kLandingDir[4] = "ccw";  // 1-cw, 2-ccw
  const enum MotionStage kLandingType = LANDING_CIRCULAR;

  // Prep Motion
  CircularPrep prep(kPrepR, kPrepW, kPrepTh0, kPrepThf, prep_ref);
  std::array<std::array<double, 3>, 3> prep_start = prep.getMotionParameter();
  double prep_runtime = prep.getRuntime();
  std::cout << prep_runtime << "s";

  // Landing Motion
  // if (kLandingType == LANDING_LINEAR) {
  //   LinearLanding landing(kLandingAccTime, prep_start);
  // }
  // else if (kLandingType == LANDING_SMOOTH) {
  //   SmoothLanding landing(kLandingAccTime, prep_start);
  // }
  // else if (kLandingType == LANDING_CIRCULAR) {
  //   CircularLanding landing(kLandingAccTime, kLandingR, kLandingDir, prep_start);
  // }
  // else {
  //   std::cout << "Select Correct Landing Type" << std::endl;
  //   return 0;
  // }
  // LinearLanding landing(kLandingAccTime, prep_start);
  // SmoothLanding landing(kLandingAccTime, prep_start);
  CircularLanding landing(kLandingAccTime, kLandingR, kLandingDir, prep_start, true); 
  landing.printMotionParameter();

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
    std::array<double, 16> initial_pose;
    std::array<double, 3> desired_pt;
    enum MotionStage stage = kLandingType;
    double t = 0.0;    

    // prep circle
    bool is_prep = false;
    double prep_t0 = 0.0;
    double diff_x = 0.0;
    double diff_y = 0.0;
    
    // Define callback function to send Cartesian pose goals to get inverse kinematics solved.
    auto cartesian_pose_callback = [=, &t, &initial_pose, &stage, &desired_pt, &diff_x, &diff_y, &is_prep, &prep_t0](
                                       const franka::RobotState& robot_state,
                                       franka::Duration period) -> franka::CartesianPose {
      // Update time.
      t += period.toSec();

      // Read the initial pose to start the motion from in the first time step.
      if (t == 0.0) {
        initial_pose = robot_state.O_T_EE_c;
      }
      
      // Print out when packet loss happens
      if (period.toMSec() > 1){
        std::cout << "*" << period.toMSec() << std::endl;
      }

      // // Decide motion stage
      // if (t >= kLandingAccTime && stage != PREP_CIRCULAR){
      //   stage = PREP_CIRCULAR;
      // }

      std::array<double, 6> vel_current = robot_state.O_dP_EE_c;
      double v = getVelocity({vel_current[0], vel_current[1], vel_current[2]});
      double v_max = getVelocity(prep.getDesiredVel(0.0));
      if (v >= v_max && stage != PREP_CIRCULAR){
        stage = PREP_CIRCULAR;
      }
      if (t >= kLandingAccTime + prep_runtime){
        stage = FINISHED;
        std::cout << "time to finish" << std::endl;
      }

      // get desired value
      franka::CartesianPose pose_desired = initial_pose;
      switch (stage) {
        case LANDING_LINEAR:
          desired_pt = landing.getDesiredPose(t);
          // std::cout << desired_pt[0] << "," << desired_pt[1] << std::endl;
          break;
        
        case LANDING_SMOOTH:
          desired_pt = landing.getDesiredPose(t);
          break;

        case LANDING_CIRCULAR:
          desired_pt = landing.getDesiredPose(t);
          prep_t0 = t;
          break;
        
        case PREP_CIRCULAR:
          if (!is_prep){
            std::array<double, 3> landing_xf = landing.getDesiredPose(t);
            std::array<double, 3> prep_x0 = prep.getDesiredPose(0.0);
            diff_x = robot_state.O_T_EE[12] - prep_x0[0];
            diff_y = robot_state.O_T_EE[13] - prep_x0[1];
            std::cout << "PREP START---------------" << t-prep_t0 << std::endl;
            std::cout << "[prep_x0]: " << prep_x0[0] << "," << prep_x0[1] << std::endl;
            std::cout << "[landing_xf]: " << landing_xf[0] << "," << landing_xf[1] << std::endl;
            std::cout << "[x_0] measured: " << robot_state.O_T_EE[0] << "," << robot_state.O_T_EE[1] << std::endl;
            std::cout << "[diff] : " << diff_x << "," << diff_y << std::endl;
            std::cout << "[v_l_T] measured: " << robot_state.O_dP_EE_c[0] << "," << robot_state.O_dP_EE_c[1] << std::endl;
                      // << "  desired:" << landing.get << "," << lmotion.v_p_0.y << std::endl;
            std::cout << "[a_l_T] measured: " << robot_state.O_ddP_EE_c[0] << "," << robot_state.O_ddP_EE_c[1] << std::endl;
                      // << "  desired:" << lmotion.a_p_0.x << "," << lmotion.a_p_0.y << std::endl;
            is_prep = true;
          }
            
          desired_pt = prep.getDesiredPose(t-prep_t0);
          desired_pt[0] +=  diff_x;
          desired_pt[1] +=  diff_y;
          std::cout << "(p) measured:" << robot_state.O_T_EE[12] << "," << robot_state.O_T_EE[13]
                    << "  desired:" <<  desired_pt[0] << "," << desired_pt[1]
                    << "  acc:" << robot_state.O_ddP_EE_c[0] << "," << robot_state.O_ddP_EE_c[1] << std::endl;
          break;

        case FINISHED:
          std::cout << "(f)" << std::endl;
          pose_desired.O_T_EE[12] = desired_pt[0];
          pose_desired.O_T_EE[13] = desired_pt[1];
          return franka::MotionFinished(pose_desired);  // 꼭 pose_desired 넘겨줘야 되나????
      }

      // log data

      // Send desired pose.
      pose_desired.O_T_EE[12] = desired_pt[0];
      pose_desired.O_T_EE[13] = desired_pt[1];
      return pose_desired;
    };

    robot.control(cartesian_pose_callback);
    // robot.control(cartesian_pose_callback, franka::ControllerMode::kJointImpedance, false, 1000.0);
    // robot.control(cartesian_pose_callback, franka::ControllerMode::kJointImpedance, true, 10);
  } catch (const franka::ControlException& e) {
    std::cout << e.what() << std::endl;
    writeLogToFile(e.log);
    return -1;

  } catch (const franka::Exception& ex) {
    std::cerr << ex.what() << std::endl;
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