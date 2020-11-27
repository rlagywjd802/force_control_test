cmake_minimum_required(VERSION 3.0.2)
project(force_control_test)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs visualization_msgs icra18 message_generation)
find_package(Franka REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Poco REQUIRED COMPONENTS Foundation)
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

include_directories(
#   include
  utility
  spline_cpp
  /usr/local/MATLAB/R2020a/extern/include
  /home/hj/fci_ws/src/ReflexxesTypeII/include
  ${catkin_INCLUDE_DIRS}
  icra18
)

add_message_files(
  FILES
  FrankaState.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(CATKIN_DEPENDS roscpp std_msgs geometry_msgs visualization_msgs icra18)

# custom libraries
add_library(examples_common utility/examples_common.cpp)
target_link_libraries(examples_common ${catkin_LIBRARIES} Franka::Franka Eigen3::Eigen)

add_library(custom_log utility/custom_log.cpp)
target_link_libraries(custom_log ${catkin_LIBRARIES} Franka::Franka Eigen3::Eigen)

add_library(landing_motion utility/landing_motion.cpp)
target_link_libraries(landing_motion ${catkin_LIBRARIES} Franka::Franka Eigen3::Eigen)

add_library(prep_motion utility/prep_motion.cpp)
target_link_libraries(prep_motion ${catkin_LIBRARIES} Franka::Franka Eigen3::Eigen)

add_library(cartesian_pose_log utility/cartesian_pose_log.cpp)
target_link_libraries(cartesian_pose_log ${catkin_LIBRARIES} Franka::Franka Eigen3::Eigen)

add_library(spline spline_cpp/Spline.cpp)
target_link_libraries(spline ${catkin_LIBRARIES})

add_library(target_update utility/target_update.cpp)
target_link_libraries(target_update ${catkin_LIBRARIES})

add_library(force_update utility/force_update.cpp)
target_link_libraries(force_update ${catkin_LIBRARIES})

# test files
add_executable(gravity_compensation src/gravity_compensation.cpp)
target_link_libraries(gravity_compensation ${catkin_LIBRARIES} Franka::Franka Poco::Foundation examples_common custom_log)

add_executable(log_test src/log_test.cpp)
target_link_libraries(log_test ${catkin_LIBRARIES} Franka::Franka Poco::Foundation examples_common)

add_executable(read_test src/read_test.cpp)
target_link_libraries(read_test ${catkin_LIBRARIES} Franka::Franka)

add_executable(joint_impedance_control_test src/joint_impedance_control_test.cpp)
target_link_libraries(joint_impedance_control_test ${catkin_LIBRARIES} Franka::Franka Threads::Threads examples_common)

add_executable(k_interpolation src/k_interpolation.cpp)
target_link_libraries(k_interpolation ${catkin_LIBRARIES} Franka::Franka Poco::Foundation Threads::Threads examples_common)

add_executable(eclipse_test src/eclipse_test.cpp)
target_link_libraries(eclipse_test ${catkin_LIBRARIES} Franka::Franka Poco::Foundation Threads::Threads examples_common)

add_executable(landing_motion_test src/landing_motion_test.cpp)
target_link_libraries(landing_motion_test ${catkin_LIBRARIES} Franka::Franka Poco::Foundation Threads::Threads examples_common custom_log)

add_executable(joint_point_to_point_motion src/joint_point_to_point_motion.cpp)
target_link_libraries(joint_point_to_point_motion ${catkin_LIBRARIES} Franka::Franka Poco::Foundation Threads::Threads examples_common)

add_executable(cartesian_point_to_point_motion src/cartesian_point_to_point_motion.cpp)
target_link_libraries(cartesian_point_to_point_motion ${catkin_LIBRARIES} Franka::Franka Poco::Foundation Threads::Threads examples_common)

add_executable(echo_robot_state src/echo_robot_state.cpp)
target_link_libraries(echo_robot_state ${catkin_LIBRARIES} Franka::Franka)

add_executable(landing_prep_motion_test src/landing_prep_motion_test.cpp)
target_link_libraries(landing_prep_motion_test ${catkin_LIBRARIES} Franka::Franka Poco::Foundation Threads::Threads examples_common landing_motion prep_motion cartesian_pose_log)

add_executable(landing_prep_motion_vel_test src/landing_prep_motion_vel_test.cpp)
target_link_libraries(landing_prep_motion_vel_test ${catkin_LIBRARIES} Franka::Franka Poco::Foundation Threads::Threads examples_common landing_motion prep_motion cartesian_pose_log)

add_executable(landing_prep_spline_vel_test src/landing_prep_spline_vel_test.cpp)
target_link_libraries(landing_prep_spline_vel_test ${catkin_LIBRARIES} Franka::Franka Poco::Foundation Threads::Threads examples_common landing_motion prep_motion cartesian_pose_log)

add_executable(complete_landing_prep_spline_vel_test src/complete_landing_prep_spline_vel_test.cpp)
target_link_libraries(complete_landing_prep_spline_vel_test ${catkin_LIBRARIES} Franka::Franka Poco::Foundation Threads::Threads examples_common landing_motion prep_motion cartesian_pose_log)

add_executable(complete_landing_multiple_layer_prep_test src/complete_landing_multiple_layer_prep_test.cpp)
target_link_libraries(complete_landing_multiple_layer_prep_test ${catkin_LIBRARIES} Franka::Franka Poco::Foundation Threads::Threads examples_common landing_motion prep_motion cartesian_pose_log)

add_executable(complete_landing_multiple_layer_prep_gui src/complete_landing_multiple_layer_prep_gui.cpp)
target_link_libraries(complete_landing_multiple_layer_prep_gui ${catkin_LIBRARIES} Franka::Franka Poco::Foundation Threads::Threads examples_common landing_motion prep_motion cartesian_pose_log)

add_executable(complete_landing_prep_spline_otg_test src/complete_landing_prep_spline_otg_test.cpp)
target_link_libraries(complete_landing_prep_spline_otg_test ${catkin_LIBRARIES} Franka::Franka Poco::Foundation Threads::Threads examples_common landing_motion prep_motion cartesian_pose_log /home/hj/fci_ws/src/ReflexxesTypeII/Linux/x64/debug/lib/shared/libReflexxesTypeII.so)
	
add_executable(complete_landing_prep_otg src/complete_landing_prep_otg.cpp)
target_link_libraries(complete_landing_prep_otg ${catkin_LIBRARIES} Franka::Franka Poco::Foundation Threads::Threads examples_common landing_motion prep_motion cartesian_pose_log /home/hj/fci_ws/src/ReflexxesTypeII/Linux/x64/debug/lib/shared/libReflexxesTypeII.so)

add_executable(complete_landing_prep_otg_update_target src/complete_landing_prep_otg_update_target.cpp)
target_link_libraries(complete_landing_prep_otg_update_target ${catkin_LIBRARIES} Franka::Franka Poco::Foundation Threads::Threads examples_common landing_motion prep_motion cartesian_pose_log target_update /home/hj/fci_ws/src/ReflexxesTypeII/Linux/x64/debug/lib/shared/libReflexxesTypeII.so)

add_executable(cartesian_impedance_control_test src/cartesian_impedance_control_test.cpp)
target_link_libraries(cartesian_impedance_control_test ${catkin_LIBRARIES} Franka::Franka Threads::Threads examples_common Eigen3::Eigen)

add_executable(impedance_multiple_layer src/impedance_multiple_layer.cpp)
target_link_libraries(impedance_multiple_layer ${catkin_LIBRARIES} Franka::Franka Threads::Threads examples_common Eigen3::Eigen examples_common landing_motion prep_motion)

add_executable(parallel_control_test src/parallel_control_test.cpp)
target_link_libraries(parallel_control_test ${catkin_LIBRARIES} Franka::Franka Threads::Threads examples_common Eigen3::Eigen)

add_executable(guiding_mode src/guiding_mode.cpp)
target_link_libraries(guiding_mode ${catkin_LIBRARIES} Franka::Franka Threads::Threads examples_common Eigen3::Eigen)

add_executable(cartesian_impedance_control_gui src/cartesian_impedance_control_gui.cpp)
target_link_libraries(cartesian_impedance_control_gui ${catkin_LIBRARIES} Franka::Franka Threads::Threads examples_common Eigen3::Eigen)

add_executable(force_control_test src/force_control_test.cpp)
target_link_libraries(force_control_test ${catkin_LIBRARIES} Franka::Franka Threads::Threads examples_common Eigen3::Eigen)

add_executable(circle_prep_test src/circle_prep_test.cpp)
target_link_libraries(circle_prep_test ${catkin_LIBRARIES} Franka::Franka Threads::Threads examples_common)

add_executable(serial_test serial/serial_test.cpp)
target_link_libraries(serial_test ${catkin_LIBRARIES})

add_executable(cartesian_impedance_circle_test src/cartesian_impedance_circle_test.cpp)
target_link_libraries(cartesian_impedance_circle_test ${catkin_LIBRARIES} Franka::Franka Threads::Threads examples_common Eigen3::Eigen)

add_executable(landing_prep_circle_test src/landing_prep_circle_test.cpp)
target_link_libraries(landing_prep_circle_test ${catkin_LIBRARIES} Franka::Franka Poco::Foundation Threads::Threads examples_common landing_motion prep_motion cartesian_pose_log)

add_executable(landing_prep_circle_force_test src/landing_prep_circle_force_test.cpp)
target_link_libraries(landing_prep_circle_force_test ${catkin_LIBRARIES} Franka::Franka Poco::Foundation Threads::Threads examples_common landing_motion prep_motion cartesian_pose_log force_update)

add_executable(subscriber_test src/subscriber_test.cpp)
target_link_libraries(subscriber_test ${catkin_LIBRARIES} Franka::Franka Poco::Foundation Threads::Threads examples_common landing_motion prep_motion cartesian_pose_log force_update)

add_executable(circle_prep_radius_change_test src/circle_prep_radius_change_test.cpp)
target_link_libraries(circle_prep_radius_change_test ${catkin_LIBRARIES} Franka::Franka Threads::Threads examples_common force_update)

add_executable(circle_prep_radius_change src/circle_prep_radius_change.cpp)
target_link_libraries(circle_prep_radius_change ${catkin_LIBRARIES} Franka::Franka Threads::Threads examples_common force_update)

add_executable(circle_prep_otg_update_target src/circle_prep_otg_update_target.cpp)
target_link_libraries(circle_prep_otg_update_target ${catkin_LIBRARIES} Franka::Franka Poco::Foundation Threads::Threads examples_common landing_motion prep_motion cartesian_pose_log target_update /home/hj/fci_ws/src/ReflexxesTypeII/Linux/x64/debug/lib/shared/libReflexxesTypeII.so)


# from matlab
add_executable(spline_main spline_cpp/examples/main.cpp)
target_link_libraries(spline_main ${catkin_LIBRARIES} spline)