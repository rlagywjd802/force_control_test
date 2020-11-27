#include <iostream>
#include <franka/exception.h>
#include <franka/robot.h>

#define FCI_IP "192.168.30.151"

int main(int argc, char** argv) {
  try {
    franka::Robot robot(FCI_IP);
    size_t count = 0;
    robot.read([&count](const franka::RobotState& robot_state) {
      // Printing to std::cout adds a delay. This is acceptable for a read loop such as this, but
      // should not be done in a control loop.
      std::cout << robot_state << std::endl;
      return count++ < 0;
    });
    std::cout << "Done." << std::endl;
  } catch (franka::Exception const& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }
  return 0;
}