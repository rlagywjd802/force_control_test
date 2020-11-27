#include <array>
#include <atomic>
#include <cmath>
#include <functional>
#include <iostream>
#include <iterator>
#include <mutex>
#include <thread>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>

#include <ros/ros.h>
#include <force_control_test/FrankaState.h>

#include "examples_common.h"
#include "force_update.h"

#define FCI_IP "192.168.30.151"

namespace {
template <class T, size_t N>
std::ostream& operator<<(std::ostream& ostream, const std::array<T, N>& array) {
  ostream << "[";
  std::copy(array.cbegin(), array.cend() - 1, std::ostream_iterator<T>(ostream, ","));
  std::copy(array.cend() - 1, array.cend(), std::ostream_iterator<T>(ostream));
  ostream << "]";
  return ostream;
}
}  // anonymous namespace

int main(int argc, char** argv) {

  // Set and initialize trajectory parameters.
  // const double radius = 0.02; //0.0196;
  const double vel_max = 0.003;
  const double acceleration_time = 0.2;
  const double run_time = 175.0;

  // Circle Motion
  const double kPrepOX = 0.634896;
  const double kPrepOY = -0.0213497; //-0.022188;
  const double kPrepOZ = 0.3120; // layer1(0.3140), layer2(0.3120)
  const double kPrepR = 0.0199; // layer1(0.0204), layer2(0.0206)

  // Set print rate for comparing commanded vs. measured torques.
  const double print_rate = 50.0;
  double vel_current = 0.0;
  double angle = 0.0;
  double time = 0.0;

  // Force parameters
  double feedforce = 0.0;
  double force_max = 20.0;
  double rgamma1 = 0.0;
  double rgamma2 = 0.0;
  double rgamma3 = 0.0;
  double nrgamma1 = 0.0;
  double nrgamma2 = 0.0;
  double nrgamma3 = 0.0;
  //butterworth low pass filter - 2nd order, Wn: 0.02
  const double bA1 = -1.911197067426073;
  const double bA2 = 0.914975834801433;
  const double bB0 = 0.000944691843840;
  const double bB1 = 0.001889383687680;
  const double bB2 = 0.000944691843840;
  const double kp = 0.00020;
  const double force_threshold = 2.0;

  // Initialize data fields for the print thread.
  struct {
    std::mutex mutex;
    bool has_data;
    std::array<double, 3> force;
    franka::RobotState robot_state;
    std::array<double, 3> delta;
    std::array<double, 3> delta_ref;
    double data;
  } print_data{};
  std::atomic_bool running{true};

  // Publisher
  ros::init(argc, argv, "circle_prep_radius_change");
  ros::NodeHandle nh;
  ros::Publisher etc_pub = nh.advertise<force_control_test::FrankaState>("franka", 1);

  ForceUpdate force_update;
  ros::spinOnce(); // wait for subscriber

  // Start print thread.
  std::thread print_thread([print_rate, &print_data, &running, etc_pub]() {
    while (running) {
      // Sleep to achieve the desired print rate.
      std::this_thread::sleep_for(
          std::chrono::milliseconds(static_cast<int>((1.0 / print_rate * 1000.0))));
      
      // Try to lock data to avoid read write collisions.
      if (print_data.mutex.try_lock()) {
        if (print_data.has_data) {
          force_control_test::FrankaState msg;
          for (size_t i=0; i<3; i++) {
            msg.force.push_back(print_data.force[i]);
          }
          // msg.force.push_back(std::sqrt(print_data.force[0]*print_data.force[0] + print_data.force[1]*print_data.force[1]));
          
          msg.position.push_back(print_data.robot_state.O_T_EE[12]);
          msg.position.push_back(print_data.robot_state.O_T_EE[13]);
          msg.position.push_back(print_data.robot_state.O_T_EE[14]);
          msg.position.push_back(print_data.delta[0]);
          msg.position.push_back(print_data.delta[1]);
          msg.position.push_back(print_data.delta[2]);
          msg.position.push_back(print_data.delta_ref[0]);
          msg.position.push_back(print_data.delta_ref[1]);
          msg.position.push_back(print_data.delta_ref[2]);

          msg.position.push_back(print_data.data);
          etc_pub.publish(msg);
          print_data.has_data = false;
        }
        print_data.mutex.unlock();
      }
    }
  });

  try {
    // Connect to robot.
    franka::Robot robot(FCI_IP);
    setDefaultBehavior(robot);

    // Warning For Start
    std::cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    std::cout << "===========================" << std::endl;
    std::cout << "radius : " << kPrepR << std::endl;
    std::cout << "vel_max : " << vel_max << std::endl;
    std::cout << "acceleration_time : " << acceleration_time << std::endl;
    std::cout << "===========================" << std::endl;
    std::cout << "bA1 : " << bA1 << std::endl;
    std::cout << "bA2 : " << bA2 << std::endl;
    std::cout << "bB0 : " << bB0 << std::endl;
    std::cout << "bB1 : " << bB1 << std::endl;
    std::cout << "bB2 : " << bB2 << std::endl;
    std::cout << "kp : " << kp << std::endl;
    std::cout << "force_threshold : " << force_threshold << std::endl;
    std::cout << "===========================" << std::endl;
    
    // First move the robot to a suitable joint configuration
    std::array<double, 7> q_goal = {{0, -M_PI/6.0, 0, -5 * M_PI / 6.0, 0, 2*M_PI/3.0, 0.0}};
    MotionGenerator motion_generator(0.2, q_goal);
    robot.control(motion_generator);
    std::cout << "Finished moving to initial joint configuration." << std::endl;

    // std::array<double, 3> x_goal_q = {{0.5637, -0.0347, 0.3225}};
    std::array<double, 3> x_goal_q = {{kPrepOX-kPrepR-0.05, kPrepOY, kPrepOZ}}; ///////////////////////// Q
    CMotionGenerator cartesian_motion_generator_q(0.2, x_goal_q);
    robot.control(cartesian_motion_generator_q);
    std::cout << "Finished moving to point Q" << std::endl;

    // std::array<double, 3> x_goal_p = {{0.6137, -0.0347, 0.3225}};
    std::array<double, 3> x_goal_p = {{kPrepOX-kPrepR, kPrepOY, kPrepOZ}}; ///////////////////////// P
    CMotionGenerator cartesian_motion_generator_p(0.2, x_goal_p);
    robot.control(cartesian_motion_generator_p);
    std::cout << "Finished moving to point P" << std::endl;
    std::cout << "Point P : " << x_goal_p[0] << ", " << x_goal_p[1] << ", " << x_goal_p[2] << std::endl;

    // Set additional parameters always before the control loop, NEVER in the control loop!
    // Set collision behavior.
    robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});
    robot.setJointImpedance({{3000, 3000, 3000, 3000, 3000, 3000, 3000}});
    // robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
    // robot.setJointImpedance({{2000, 2000, 2000, 2000, 2000, 2000, 2000}});
    // robot.setJointImpedance({{1500, 1500, 1500, 1500, 1500, 1500, 1500}});

    // robot.setCollisionBehavior(
    // {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
    // {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
    // {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
    // {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}});
    // robot.setCartesianImpedance({{1000, 1000, 3000, 300, 300, 300}});
    // robot.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
    // robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});

    // Load the kinematics and dynamics model.
    franka::Model model = robot.loadModel();
    std::array<double, 16> initial_pose;
    std::array<double, 6> initial_force;
    
    // Define callback function to send Cartesian pose goals to get inverse kinematics solved.
    auto cartesian_pose_callback = [=, &print_data, &running, &time, &vel_current, 
                                    &angle, &initial_pose, &initial_force, &force_update, &feedforce, 
                                    &rgamma1, &rgamma2, &rgamma3, &nrgamma1, &nrgamma2, &nrgamma3](
                                       const franka::RobotState& robot_state,
                                       franka::Duration period) -> franka::CartesianPose 
    {
      // Update time.
      time += period.toSec();
      if (time == 0.0) {
        // Read the initial pose to start the motion from in the first time step.
        initial_pose = robot_state.O_T_EE_c;
        initial_force = robot_state.K_F_ext_hat_K;
      }

      // Compute Cartesian velocity.
      if (vel_current < vel_max && time < run_time) {
        vel_current += period.toSec() * std::fabs(vel_max / acceleration_time);
      }
      if (vel_current > 0.0 && time > run_time) {
        vel_current -= period.toSec() * std::fabs(vel_max / acceleration_time);
      }
      vel_current = std::fmax(vel_current, 0.0);
      vel_current = std::fmin(vel_current, vel_max);

      // Compute new angle for our circular trajectory.
      angle += period.toSec() * vel_current / std::fabs(kPrepR);
      if (angle > 2 * M_PI) {
        angle -= 2 * M_PI;
      }
      
      // Compute radius   
      //=====================================================   
      double* force_value = force_update.getForceValue();
      std::array<double, 3> subscribed_force = {{force_value[0], force_value[1], force_value[2]}};
      if(feedforce < force_max) feedforce = force_value[2];
      // if(feedforce > force_threshold) 
      // {
      //   rgamma1 = kp * feedforce;
      // }
      // else rgamma1 = 0.0;
      rgamma1 = kp * feedforce;

      nrgamma1 = bB0*rgamma1 + bB1*rgamma2 + bB2*rgamma3;
      nrgamma1 = nrgamma1 - bA1*nrgamma2 - bA2*nrgamma3;

      //if(rgamma > 0.001) rgamma = 0.001;
      //=====================================================

      // Compute relative x and y positions of desired pose.
      double delta_x = (kPrepR + nrgamma1) * (1 - std::cos(angle));
      double delta_y = (kPrepR + nrgamma1) * std::sin(angle);
      std::array<double, 3> delta_ref;
      delta_ref[0] = kPrepR * (1 - std::cos(angle)) + initial_pose[12];
      delta_ref[1] = kPrepR * std::sin(angle)+ initial_pose[13];
      delta_ref[2] = initial_pose[14];
      std::array<double, 3> delta;
      delta[0] = delta_x + initial_pose[12];
      delta[1] = delta_y + initial_pose[13];
      delta[2] = initial_pose[14];
      franka::CartesianPose pose_desired = initial_pose;
      pose_desired.O_T_EE[12] += delta_x;
      pose_desired.O_T_EE[13] += delta_y;
      //-----------------------------------------------------
      rgamma3 = rgamma2;
      rgamma2 = rgamma1;
      nrgamma3 = nrgamma2;
      nrgamma2 = nrgamma1;
      //-----------------------------------------------------

      // Compute calibrated force
      std::array<double, 6> calibrated_force;
      for (int i=0; i<6; i++)
        calibrated_force[i] = robot_state.K_F_ext_hat_K[i] - initial_force[i];

      // Update data to print.
      if (print_data.mutex.try_lock()) {
        print_data.has_data = true;
        print_data.robot_state = robot_state;
        print_data.force = subscribed_force;
        print_data.data = nrgamma1;
        print_data.delta_ref = delta_ref;
        print_data.delta = delta;
        print_data.mutex.unlock();
      }

      ros::spinOnce();

      // Send desired pose.
      if (time >= run_time + acceleration_time) {
        running = false;
        return franka::MotionFinished(pose_desired);
      }
      return pose_desired;
    };

    // Start real-time control loop.
    robot.control(cartesian_pose_callback);
    // robot.control(cartesian_pose_callback, franka::ControllerMode::kCartesianImpedance);

  } catch (const franka::Exception& ex) {
    running = false;
    std::cerr << ex.what() << std::endl;
  }

  if (print_thread.joinable()) {
    print_thread.join();
  }
  return 0;
}