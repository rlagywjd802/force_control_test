#include "custom_log.h"

#include <iterator>
#include <sstream>

using namespace std::string_literals;  // NOLINT(google-build-using-namespace)

namespace franka {

namespace {

template <typename T, size_t N>
std::string csvName(const std::array<T, N>& /*unused*/, const std::string& name) {
  std::ostringstream os;
  for (size_t i = 0; i < N - 1; i++) {
    os << name << "[" << i << "], ";
  }
  os << name << "[" << N - 1 << "]";
  return os.str();
}

template <class T, size_t N>
std::ostream& operator<<(std::ostream& ostream /*unused*/, const std::array<T, N>& array) {
  std::copy(array.cbegin(), array.cend() - 1, std::ostream_iterator<T>(ostream, ","));
  std::copy(array.cend() - 1, array.cend(), std::ostream_iterator<T>(ostream));
  return ostream;
}

std::string csvRobotStateHeader() {
  RobotState robot_state;
  std::ostringstream os;
  os << "duration, success rate, " << csvName(robot_state.q, "q") << ","
     << csvName(robot_state.O_T_EE, "O_T_EE");
  return os.str();
}

std::string csvRobotCommandHeader() {
  franka::cRobotCommand command;
  std::ostringstream os;
  os << "sent commands," << csvName(command.torques.tau_J, "tau_J_d");
  return os.str();
}

std::string csvLine(const franka::RobotState& robot_state) {
  std::ostringstream os;
  os << robot_state.time.toMSec() << "," << robot_state.control_command_success_rate << ","
     << robot_state.q << "," << robot_state.O_T_EE;
  return os.str();
}

std::string csvLine(const franka::cRobotCommand& command) {
  std::ostringstream os;
  os << command.torques.tau_J;
  return os.str();
}

}  // anonymous namespace

std::string customlogToCSV(const std::vector<cRecord>& log) {
  if (log.empty()) {
    return "";
  }
  std::ostringstream os;

  os << csvRobotStateHeader() << "," << csvRobotCommandHeader() << std::endl;
  for (const cRecord& r : log) {
    os << csvLine(r.state) << ",," << csvLine(r.command) << std::endl;
  }

  return os.str();
}


}  // namespace franka