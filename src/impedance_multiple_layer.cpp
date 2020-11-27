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

#include <Eigen/Dense>

#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>

#include "examples_common.h"
#include "landing_motion.h"
#include "prep_motion.h"

#define FCI_IP "192.168.30.151"
#define DT 0.001
#define D2R M_PI/180.0

enum MotionStage {
    LANDING_CIRCULAR,
    PREP_SPLINE,
    TAKINGOFF_CIRCULAR,
    FINISHED
};

double getVelocity(const std::array<double, 3> vel);

int main(int argc, char** argv) {
  
  // ros
  ros::init(argc, argv, "impedance_multiple_layer");
  ros::NodeHandle nh;
  ros::Publisher ft_pub = nh.advertise<geometry_msgs::WrenchStamped>("ft_data", 1);

  // prep motion parameters
  const double kPrepRefX = 0.3;
  const double kPrepRefY = 0.0;
  const double kPrepRefZ = 0.45;
  const double kPrepVel = 0.005;
  std::array<double, 3> prep_ref = {kPrepRefX, kPrepRefY, kPrepRefZ};
  std::string prep_file_path1 = "/home/hj/Documents/MATLAB/data/prep_lower_layer_1.csv";
  std::string prep_file_path2 = "/home/hj/Documents/MATLAB/data/prep_lower_layer_2.csv";
  std::string prep_file_path3 = "/home/hj/Documents/MATLAB/data/prep_lower_layer_3.csv";
  std::array<std::string, NLayer> prep_file_paths = {prep_file_path1, prep_file_path2, prep_file_path3};

  // landing motion parameters
  const double kLandingAccTime = 1.5;
  const double kLandingR = 0.002;
  const char kLandingDir[4] = "www";  // 1-cw(www), 2-ccw(ccc)
  const enum MotionStage kLandingType = LANDING_CIRCULAR;

  // sliding motion paramters
  const int kSlidingVpts = 50;
  const double kSlidingTime = 0.12;

  // Prep Motion
  MultipleSplinePrep prep(prep_file_paths, kPrepVel, kSlidingVpts, kSlidingTime, prep_ref);
  std::array<std::array<double, 3>, 3> prep_start = prep.getPrepStart();
  std::array<std::array<double, 3>, 3> prep_end = prep.getPrepEnd();
  double prep_runtime = prep.getRuntime();
  // prep.printMotionParameter();

  // Landing Motion
  CircularLanding landing(kLandingAccTime, kLandingR, kLandingDir, prep_start, true); 
  CircularLanding takingoff(kLandingAccTime, kLandingR, kLandingDir, prep_end, false); 
  landing.printMotionParameter();
  takingoff.printMotionParameter();

  // Initialize data fields for the print thread.
  struct {
    std::mutex mutex;
    bool has_data;
    franka::RobotState robot_state;
    double des_x;
    double des_y;
    double des_z;
  } print_data{};
  std::atomic_bool running{true};

  // Set print rate and thread related parameters
  const double print_rate = 100.0;
  int ft_seq = 0;

  // Start print thread.
  std::thread print_thread([print_rate, &print_data, &running, &ft_seq, &ft_pub]() {
    while (running) {
      // Sleep to achieve the desired print rate.
      std::this_thread::sleep_for(
          std::chrono::milliseconds(static_cast<int>((1.0 / print_rate * 1000.0))));
      
      // Try to lock data to avoid read write collisions.
      if (print_data.mutex.try_lock()) {
        if (print_data.has_data) {
          // Print data to console
          std::cout << print_data.robot_state.O_T_EE[12] << "," << print_data.robot_state.O_T_EE[13] 
                    << "," << print_data.robot_state.O_T_EE[14] << "," << print_data.des_x << "," 
                    << print_data.des_y << "," << print_data.des_z << std::endl;

          // Publish
          geometry_msgs::WrenchStamped ft_data;
          ft_data.header.seq = ft_seq++;
          ft_data.header.stamp = ros::Time::now();
          ft_data.header.frame_id = "world";
          ft_data.wrench.force.x = print_data.robot_state.K_F_ext_hat_K[0];
          ft_data.wrench.force.y = print_data.robot_state.K_F_ext_hat_K[1];
          ft_data.wrench.force.z = print_data.robot_state.K_F_ext_hat_K[2];
          ft_data.wrench.torque.x = print_data.robot_state.K_F_ext_hat_K[3];
          ft_data.wrench.torque.y = print_data.robot_state.K_F_ext_hat_K[4];
          ft_data.wrench.torque.z = print_data.robot_state.K_F_ext_hat_K[5];
          ft_pub.publish(ft_data);
          print_data.has_data = false;
        }
        print_data.mutex.unlock();
      }
    }
  });

  // Compliance parameters
  const double translational_stiffness{150.0};
  const double rotational_stiffness{10.0};
  Eigen::MatrixXd stiffness(6, 6), damping(6, 6);
  stiffness.setZero();
  stiffness.topLeftCorner(3, 3) << translational_stiffness * Eigen::MatrixXd::Identity(3, 3);
  stiffness.bottomRightCorner(3, 3) << rotational_stiffness * Eigen::MatrixXd::Identity(3, 3);
  damping.setZero();
  damping.topLeftCorner(3, 3) << 2.0 * sqrt(translational_stiffness) *
                                     Eigen::MatrixXd::Identity(3, 3);
  damping.bottomRightCorner(3, 3) << 2.0 * sqrt(rotational_stiffness) *
                                         Eigen::MatrixXd::Identity(3, 3);

  try {
    // Connect to robot.
    franka::Robot robot(FCI_IP);
    setDefaultBehavior(robot);

    // First move the robot to start point of landing motion
    std::array<double, 3> x_goal = landing.getDesiredPose(0.0);
    CMotionGenerator motion_generator(0.2, x_goal);

    // load the kinematics and dynamics model
    franka::Model model = robot.loadModel();
    franka::RobotState initial_state = robot.readOnce();
    
    // equilibrium point is the initial position
    Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
    Eigen::Vector3d position_d(initial_transform.translation());
    Eigen::Quaterniond orientation_d(initial_transform.linear());
    
    // set collision behavior
    robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});

    // start real-time control loop
    std::cout << "WARNING: Collision thresholds are set to high values. "
              << "Make sure you have the user stop at hand!" << std::endl
              << "After starting try to push the robot and see how it reacts." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    
    // Define initial variable for motion callback.
    std::array<double, 3> desired_vel;
    enum MotionStage stage = kLandingType;
    double t = 0.0;    

    // prep circle
    bool is_prep = false;
    double prep_t0 = 0.0;
    double prep_t1 = 0.0;
    double diff_x = 0.0;
    double diff_y = 0.0;
    
    // Cartesian Velocity for Motion Control
    auto cartesian_vel_callback = [=, &t, &running, &print_data, &stage, &desired_vel, &diff_x, &diff_y, &is_prep, &prep_t0, &prep_t1](
                                       const franka::RobotState& robot_state,
                                       franka::Duration period) -> franka::CartesianVelocities 
    {
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
      if (t >= kLandingAccTime+prep_runtime && stage == PREP_SPLINE){
        stage = TAKINGOFF_CIRCULAR;
        std::cout << "switch to taking off" << std::endl;
      }

      if (t >= 2*kLandingAccTime + prep_runtime){
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
          if (!is_prep){
            is_prep = true;
            prep_t0 = t;
          }           
          desired_vel = prep.getDesiredVel(t-prep_t0);
          break;

        case TAKINGOFF_CIRCULAR:
          if (prep_t1 == 0) prep_t1 = t;
          desired_vel = takingoff.getDesiredVel(t-prep_t1);
          break;

        case FINISHED:
          std::cout << "(f)" << std::endl;
          running = false;
          franka::CartesianVelocities output = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
          return franka::MotionFinished(output);  // 꼭 vel_desired 넘겨줘야 되나????
      }

      // log data
      if (print_data.mutex.try_lock()) {
        print_data.has_data = true;
        print_data.robot_state = robot_state;
        if (stage == LANDING_CIRCULAR) {
          std::array<double, 3> landing_x = landing.getDesiredPose(t);
          print_data.des_x = landing_x[0];
          print_data.des_y = landing_x[1];
          print_data.des_z = landing_x[2];
        }
        else if (stage == PREP_SPLINE){
          std::array<double, 3> prep_x = prep.getDesiredPose(t-prep_t0);
          print_data.des_x = prep_x[0];
          print_data.des_y = prep_x[1];
          print_data.des_z = prep_x[2];
        }
        else if (stage == TAKINGOFF_CIRCULAR){
          std::array<double, 3> takingoff_x = takingoff.getDesiredPose(t-prep_t1);
          print_data.des_x = takingoff_x[0];
          print_data.des_y = takingoff_x[1]; 
          print_data.des_z = takingoff_x[2]; 
        }
        
        print_data.mutex.unlock();
      }

      // Send desired vel
      vel_desired.O_dP_EE[0] = desired_vel[0];
      vel_desired.O_dP_EE[1] = desired_vel[1];
      return vel_desired;
    };

    // Impedance Control
    std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
        impedance_control_callback = [=, &print_data, &model](const franka::RobotState& robot_state,
                                         franka::Duration /*duration*/) -> franka::Torques 
    {
      // get state variables
      std::array<double, 7> coriolis_array = model.coriolis(robot_state);
      std::array<double, 42> jacobian_array =
          model.zeroJacobian(franka::Frame::kEndEffector, robot_state);
      
      // convert to Eigen
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
      Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
      Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
      Eigen::Vector3d position(transform.translation());
      Eigen::Quaterniond orientation(transform.linear());
      
      // compute error to desired equilibrium pose
      // position error
      Eigen::Matrix<double, 6, 1> error;
      error.head(3) << position - position_d;
      
      // orientation error
      // "difference" quaternion
      if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0) {
        orientation.coeffs() << -orientation.coeffs();
      }
      
      // "difference" quaternion
      Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d);
      error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
      
      // Transform to base frame
      error.tail(3) << -transform.linear() * error.tail(3);
      
      // compute control
      Eigen::VectorXd tau_task(7), tau_d(7);
      
      // Spring damper system with damping ratio=1
      tau_task << jacobian.transpose() * (-stiffness * error - damping * (jacobian * dq));
      tau_d << tau_task + coriolis;
      std::array<double, 7> tau_d_array{};
      Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;

      // Update data to print.
      if (print_data.mutex.try_lock()) {
        print_data.has_data = true;
        print_data.robot_state = robot_state;
        print_data.mutex.unlock();
      }

      return tau_d_array;
    };

    robot.control(impedance_control_callback, cartesian_vel_callback);
    // robot.control(cartesian_vel_callback);
    // robot.control(cartesian_vel_callback, franka::ControllerMode::kJointImpedance, false, 1000.0);
    // robot.control(cartesian_vel_callback, franka::ControllerMode::kJointImpedance, true, 10);
  } catch (const franka::Exception& ex) {
    running = false;
    std::cerr << ex.what() << std::endl;
  }

  if (print_thread.joinable()) {
    print_thread.join();
  }

  return 0;
}

double getVelocity(const std::array<double, 3> vel){
  return std::sqrt(vel[0]*vel[0] + vel[1]*vel[1] + vel[2]*vel[2]);
}