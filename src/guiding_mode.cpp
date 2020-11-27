#include <cmath>
#include <iostream>
#include <fstream>
#include <franka/exception.h>
#include <franka/robot.h>

#include "examples_common.h"

#define FCI_IP "192.168.30.151"

int main(int argc, char** argv) {
  try {
    franka::Robot robot(FCI_IP);
        
    std::array<bool, 6> guiding_mode = {true, true, true, false, false, false};
    bool elbow_mode = true;
    robot.setGuidingMode(guiding_mode, elbow_mode);

    std::cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    std::cout << "Motion finished" << std::endl;
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }
  return 0;
}