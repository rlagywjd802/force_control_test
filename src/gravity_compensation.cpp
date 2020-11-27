#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <fstream>
// #include <iomanip>
#include <iostream>

#include <Poco/DateTimeFormatter.h>
#include <Poco/File.h>
#include <Poco/Path.h>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/robot.h>

#include "examples_common.h"
#include "custom_log.h"

void writeLogToFile(const std::vector<franka::cRecord>& log);

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }
  uint64_t counter = 0;
  double avg_success_rate = 0.0;
  double min_success_rate = 1.0;
  double max_success_rate = 0.0;
  uint64_t time = 0;
  std::vector<franka::cRecord> log;

  std::cout.precision(2);
  std::cout << std::fixed;
  
  try {
    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);

    // First move the robot to a suitable joint configuration
    std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    MotionGenerator motion_generator(0.5, q_goal);
    std::cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    robot.control(motion_generator);

    // gravity compensation
    std::cout << "Finished moving to initial joint configuration." << std::endl << std::endl;
    std::cout << "Starting communication test." << std::endl;
    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});
    franka::Torques zero_torques{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    robot.control(
        [&time, &counter, &avg_success_rate, &min_success_rate, &max_success_rate, zero_torques, &log](
            const franka::RobotState& robot_state, franka::Duration period) -> franka::Torques 
        {
          time += period.toMSec();
          if (time == 0.0) {
            std::cout << robot_state << std::endl;
            return zero_torques;
          }
          counter++;
         
          if (counter % 100 == 0) {
            // std::cout << "#" << counter
            //           << " Current success rate: " << robot_state.control_command_success_rate
            //           << std::endl;
            
            // save to log
            franka::cRobotCommand robot_command;
            robot_command.torques = zero_torques;

            franka::cRecord rec;
            rec.state = robot_state;
            rec.command = robot_command;

            log.push_back(rec);
          }

          // std::this_thread::sleep_for(std::chrono::microseconds(100));
          
          // avg_success_rate += robot_state.control_command_success_rate;
          // if (robot_state.control_command_success_rate > max_success_rate) {
          //   max_success_rate = robot_state.control_command_success_rate;
          // }
          // if (robot_state.control_command_success_rate < min_success_rate) {
          //   min_success_rate = robot_state.control_command_success_rate;
          // }
          if (time >= 5000) {
            std::cout << std::endl << "Finished test, shutting down example" << std::endl;
            return franka::MotionFinished(zero_torques);
          }

          // Sending zero torques - if EE is configured correctly, robot should not move
          return zero_torques;
        },
        false, 1000);
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  writeLogToFile(log);
  // avg_success_rate = avg_success_rate / counter;
  // std::cout << std::endl
  //           << std::endl
  //           << "#######################################################" << std::endl;
  // uint64_t lost_robot_states = time - counter;
  // if (lost_robot_states > 0) {
  //   std::cout << "The control loop did not get executed " << lost_robot_states << " times in the"
  //             << std::endl
  //             << "last " << time << " milliseconds! (lost " << lost_robot_states << " robot states)"
  //             << std::endl
  //             << std::endl;
  // }
  // std::cout << "Control command success rate of " << counter << " samples: " << std::endl;
  // std::cout << "Max: " << max_success_rate << std::endl;
  // std::cout << "Avg: " << avg_success_rate << std::endl;
  // std::cout << "Min: " << min_success_rate << std::endl;
  // if (avg_success_rate < 0.90) {
  //   std::cout << std::endl
  //             << "WARNING: THIS SETUP IS PROBABLY NOT SUFFICIENT FOR FCI!" << std::endl;
  //   std::cout << "PLEASE TRY OUT A DIFFERENT PC / NIC" << std::endl;
  // } else if (avg_success_rate < 0.95) {
  //   std::cout << std::endl << "WARNING: MANY PACKETS GOT LOST!" << std::endl;
  //   std::cout << "PLEASE INSPECT YOUR SETUP AND FOLLOW ADVICE ON" << std::endl
  //             << "https://frankaemika.github.io/docs/troubleshooting.html" << std::endl;
  // }
  // std::cout << "#######################################################" << std::endl << std::endl;


  return 0;
}

void writeLogToFile(const std::vector<franka::cRecord>& log) {
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
    log_stream << franka::customlogToCSV(log);
    std::cout << "Log file written to: " << log_file.path() << std::endl;
  } catch (...) {
    std::cout << "Failed to write log file." << std::endl;
  }
}