#pragma once

#include <vector>

#include <franka/control_types.h>
#include <franka/robot_state.h>

/**
 * @file log.h
 * Contains helper types for logging sent commands and received robot states.
 */

namespace franka {

/**
 * Command sent to the robot. Structure used only for logging purposes.
 */
struct cRobotCommand {
  /**
   * \f$q_d\f$ sent to the robot.
   */
  JointPositions joint_positions{0, 0, 0, 0, 0, 0, 0};
  /**
   * \f$\dot{q}_d\f$ sent to the robot.
   */
  JointVelocities joint_velocities{0, 0, 0, 0, 0, 0, 0};
  /** 
   * \f$^O{\mathbf{T}_{EE}}_{d}\f$ sent to the robot. 
   */
   
  CartesianPose cartesian_pose{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  /**
   * \f$^O\dot{P}_{EE}\f$ sent to the robot.
   */
  CartesianVelocities cartesian_velocities{0, 0, 0, 0, 0, 0};
  /**
   * \f${\tau_J}_d\f$ sent to the robot.
   */
  Torques torques{0, 0, 0, 0, 0, 0, 0};
};

/**
 * One row of the log contains a robot command of timestamp n and a
 * corresponding robot state of timestamp n+1.
 * Provided by the ControlException.
 */
struct cRecord {
  /**
   * Robot state of timestamp n+1.
   */
  RobotState state;
  /**
   * Robot command of timestamp n, after rate limiting (if activated).
   */
  cRobotCommand command;
};

/**
 * Writes the log to a string in CSV format. If the string is not empty, the first row contains the
 * header with names of columns. The following lines contain rows of values separated by commas.
 *
 * If the log is empty, the function returns an empty string.
 *
 * @param[in] log Log provided by the ControlException.
 *
 * @return a string in CSV format, or empty string.
 */
std::string customlogToCSV(const std::vector<cRecord>& log);
}  // namespace franka