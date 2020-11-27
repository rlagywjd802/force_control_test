#pragma once

#include <vector>

#include <franka/log.h>

namespace CartesianPose {

  std::string logToCSV(const std::vector<franka::Record>& log);

}  // namespace CartesianPose