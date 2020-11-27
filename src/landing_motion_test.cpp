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

#define FCI_IP "192.168.30.151"
#define DT 0.001

enum MotionStage {
    LANDING_LINEAR = 0,
    LANDING_CIRCLE,
    LANDING_SMOOTH,
    LANDING_READY,
    PREP,
    PREP_CIRCLE,
    FINISHED
};

struct Point {
  double x; 
  double y;
};

// struct LinearMotion {
//   Point x_l_0;
//   Point a;
//   Point b;

//   Point v_p_0;
//   Point a_p_0;
// };

struct LinearMotion {
  Point x_l_0;
  Point v_p_0;
  Point a_p_0;

  double coeff_4;
  double coeff_3;
  double th_n;
};

namespace {
template <class T, size_t N>
std::ostream& operator<<(std::ostream& ostream, const std::array<T, N>& array) {
  ostream << "[";
  std::copy(array.cbegin(), array.cend() - 1, std::ostream_iterator<T>(ostream, ","));
  std::copy(array.cend() - 1, array.cend(), std::ostream_iterator<T>(ostream));
  ostream << "]";
  return ostream;
}
}  // anonymous namespace -- ????

bool check_landing_velocity_reached(const franka::RobotState& robot_state, Point landing_velocity){
  bool is_reached = false;
  double vx = std::fabs(robot_state.O_dP_EE_c[0]);  // Last commanded end effector twist in base frame
  double vy = std::fabs(robot_state.O_dP_EE_c[1]);
  
  if (vx >= std::fabs(landing_velocity.x) && vy >= std::fabs(landing_velocity.y)) 
    is_reached = true;
  
  return is_reached;
}

double get_distance(const franka::RobotState& robot_state, const std::array<double, 16>& initial_pose){
  double dx = robot_state.O_T_EE[12] - initial_pose[12];
  double dy = robot_state.O_T_EE[13] - initial_pose[13];
  double distance = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));

  return distance;
}

// LinearMotion linear_landing_motion(const std::vector<Point>& prep_motion, double acc_time){
//   LinearMotion lm;
//   Point x_p_0;
//   Point v_p_0, v_p_1;
//   Point a_p_0;

//   // 1. get x_p_0, v_p_0, a_p_0
//   x_p_0 = prep_motion[0];
//   // v_p_0.x = (prep_motion[1].x - prep_motion[0].x)/DT;
//   // v_p_0.y = (prep_motion[1].y - prep_motion[0].y)/DT;
//   // v_p_1.x = (prep_motion[2].x - prep_motion[1].x)/DT;
//   // v_p_1.y = (prep_motion[2].y - prep_motion[1].y)/DT;
//   // a_p_0.x = (v_p_1.x - v_p_0.x)/DT;
//   // a_p_0.y = (v_p_1.y - v_p_0.y)/DT;
//   v_p_0.x = 0.0;
//   v_p_0.y = 0.05;
//   a_p_0.x = -0.05;
//   a_p_0.y = 0.0;

//   std::cout << "-------- From Prep Motion --------" << std::endl;
//   std::cout<< "x_p_0: " << x_p_0.x << "," << x_p_0.y << std::endl;
//   std::cout<< "v_p_0: " << v_p_0.x << "," << v_p_0.y << std::endl;
//   std::cout<< "v_p_1: " << v_p_1.x << "," << v_p_1.y << std::endl;
//   std::cout<< "a_p_0: " << a_p_0.x << "," << a_p_0.y << std::endl;

//   // 2. calculate coefficient for velocity profile
//   lm.a.x = -2*v_p_0.x/std::pow(acc_time, 3) + a_p_0.x/std::pow(acc_time, 2);
//   lm.a.y = -2*v_p_0.y/std::pow(acc_time, 3) + a_p_0.y/std::pow(acc_time, 2);
//   lm.b.x = 3*v_p_0.x/std::pow(acc_time, 2) - a_p_0.x/acc_time;
//   lm.b.y = 3*v_p_0.y/std::pow(acc_time, 2) - a_p_0.y/acc_time;

//   // 3. find x_l_0
//   double l_x = lm.a.x/4*std::pow(acc_time, 4) + lm.b.x/3*std::pow(acc_time, 3);
//   double l_y = lm.a.y/4*std::pow(acc_time, 4) + lm.b.y/3*std::pow(acc_time, 3);
//   lm.x_l_0.x = x_p_0.x - l_x;
//   lm.x_l_0.y = x_p_0.y - l_y;

//   lm.v_p_0 = v_p_0;
//   lm.a_p_0 = a_p_0;

//   return lm;
// }
// LinearMotion linear_landing_motion(Point x_p_0, Point v_p_0, Point a_p_0, double acc_time){
//   LinearMotion lm;

//   std::cout << "-------- From Prep Motion --------" << std::endl;
//   std::cout<< "x_p_0: " << x_p_0.x << "," << x_p_0.y << std::endl;
//   std::cout<< "v_p_0: " << v_p_0.x << "," << v_p_0.y << std::endl;
//   // std::cout<< "v_p_1: " << v_p_1.x << "," << v_p_1.y << std::endl;
//   std::cout<< "a_p_0: " << a_p_0.x << "," << a_p_0.y << std::endl;

//   // 2. calculate coefficient for velocity profile
//   lm.a.x = -2*v_p_0.x/std::pow(acc_time, 3) + a_p_0.x/std::pow(acc_time, 2);
//   lm.a.y = -2*v_p_0.y/std::pow(acc_time, 3) + a_p_0.y/std::pow(acc_time, 2);
//   lm.b.x = 3*v_p_0.x/std::pow(acc_time, 2) - a_p_0.x/acc_time;
//   lm.b.y = 3*v_p_0.y/std::pow(acc_time, 2) - a_p_0.y/acc_time;

//   // 3. find x_l_0
//   double l_x = lm.a.x/4*std::pow(acc_time, 4) + lm.b.x/3*std::pow(acc_time, 3);
//   double l_y = lm.a.y/4*std::pow(acc_time, 4) + lm.b.y/3*std::pow(acc_time, 3);
//   lm.x_l_0.x = x_p_0.x - l_x;
//   lm.x_l_0.y = x_p_0.y - l_y;

//   lm.v_p_0 = v_p_0;
//   lm.a_p_0 = a_p_0;

//   return lm;
// }

// LinearMotion linear_landing_motion(const std::vector<Point>& prep_motion, double acc_time){
//   LinearMotion lm;
//   Point x_p_0;
//   Point v_p_0, v_p_1;
//   Point a_p_0;

//   // 1. get x_p_0, v_p_0, a_p_0
//   x_p_0 = prep_motion[0];
//   v_p_0.x = (prep_motion[1].x - prep_motion[0].x)/DT;
//   v_p_0.y = (prep_motion[1].y - prep_motion[0].y)/DT;
//   v_p_1.x = (prep_motion[2].x - prep_motion[1].x)/DT;
//   v_p_1.y = (prep_motion[2].y - prep_motion[1].y)/DT;
//   a_p_0.x = (v_p_1.x - v_p_0.x)/DT;
//   a_p_0.y = (v_p_1.y - v_p_0.y)/DT;
//   double v_n = std::sqrt(std::pow(v_p_0.x, 2) + std::pow(v_p_0.y, 2));
//   double a_n = std::sqrt(std::pow(a_p_0.x, 2) + std::pow(a_p_0.y, 2));
//   double th_n = std::atan2(v_p_0.y, v_p_0.x);

//   std::cout << "-------- From Prep Motion --------" << std::endl;
//   std::cout<< "x_p_0: " << x_p_0.x << "," << x_p_0.y << std::endl;
//   std::cout<< "v_p_0: " << v_p_0.x << "," << v_p_0.y << std::endl;
//   std::cout<< "v_p_1: " << v_p_1.x << "," << v_p_1.y << std::endl;
//   std::cout<< "a_p_0: " << a_p_0.x << "," << a_p_0.y << std::endl;
//   std::cout<< "v_n: " << v_n << std::endl;
//   std::cout<< "th_n: " << th_n << std::endl;
//   std::cout<< "a_n: " << a_n << std::endl;
  
//   // 2. calculate coefficient for velocity profile
//   lm.coeff_4 = -2*v_n/std::pow(acc_time, 3);
//   lm.coeff_3 = 3*v_n/std::pow(acc_time, 2);

//   // 3. find x_l_0
//   double x_n = lm.coeff_4/4*std::pow(acc_time, 4) + lm.coeff_3/3*std::pow(acc_time, 3);
//   double l_x = x_n*std::cos(th_n);
//   double l_y = x_n*std::sin(th_n);
  
//   lm.x_l_0.x = x_p_0.x - l_x;
//   lm.x_l_0.y = x_p_0.y - l_y;
//   lm.v_p_0 = v_p_0;
//   lm.th_n = th_n;

//   return lm;
// }

LinearMotion linear_landing_motion(Point x_p_0, Point v_p_0, Point a_p_0, double acc_time){
  LinearMotion lm;

  // 1. get x_p_0, v_p_0, a_p_0
  double v_n = std::sqrt(std::pow(v_p_0.x, 2) + std::pow(v_p_0.y, 2));
  double a_n = std::sqrt(std::pow(a_p_0.x, 2) + std::pow(a_p_0.y, 2));
  double th_n = std::atan2(v_p_0.y, v_p_0.x);

  std::cout << "-------- From Prep Motion --------" << std::endl;
  std::cout<< "x_p_0: " << x_p_0.x << "," << x_p_0.y << std::endl;
  std::cout<< "v_p_0: " << v_p_0.x << "," << v_p_0.y << std::endl;
  // std::cout<< "v_p_1: " << v_p_1.x << "," << v_p_1.y << std::endl;
  std::cout<< "a_p_0: " << a_p_0.x << "," << a_p_0.y << std::endl;
  std::cout<< "v_n: " << v_n << std::endl;
  std::cout<< "th_n: " << th_n << std::endl;
  std::cout<< "a_n: " << a_n << std::endl;
  
  // 2. calculate coefficient for velocity profile
  lm.coeff_4 = -2*v_n/std::pow(acc_time, 3);
  lm.coeff_3 = 3*v_n/std::pow(acc_time, 2);

  // 3. find x_l_0
  double x_n = lm.coeff_4/4*std::pow(acc_time, 4) + lm.coeff_3/3*std::pow(acc_time, 3);
  double l_x = x_n*std::cos(th_n);
  double l_y = x_n*std::sin(th_n);
  
  lm.x_l_0.x = x_p_0.x - l_x;
  lm.x_l_0.y = x_p_0.y - l_y;
  lm.v_p_0 = v_p_0;
  lm.a_p_0 = a_p_0;
  lm.th_n = th_n;

  return lm;
}

void writeLogToFile(const std::vector<franka::Record>& log);

int main(int argc, char** argv) {
  // prep motion file path
  // const std::string file_path = "/home/hj/landing_prep_motion_matlab.csv";
  const std::string file_path = "/home/hj/csv/prep_motion_r0.05_v0.1_t1.csv";
  const int jump_idx = 0;

  // Set and initialize trajectory parameters.
  const double acceleration_time = 0.5;  // [s]
  const double radius = 0.05;
  const int direction = 1; // 1-cw, 2-ccw
  const double ref_x = 0.3;
  const double ref_y = 0.0;

  // Choose landing motion
  const int landing = 1; // 1-linear, 2-circle

  // Set motion related parameters.
  std::vector<Point> prep_motion;
  Point landing_velocity, a, b;

  // Prep motion
  const double prep_dth = 0.0005;
  const double prep_r = 0.05;
  const double prep_w = prep_dth/DT;
  const double prep_v = prep_r*prep_w;
  const double prep_a = prep_r*prep_w*prep_w;
  const Point prep_x_0 = {ref_x, ref_y};
  const Point prep_v_0 = {0.0, prep_v};
  const Point prep_a_0 = {-prep_a, 0.0};

  // Set print rate for measured data.
  const double print_rate = 1000.0;

  // print basic info
  std::cout << "===================================" << std::endl;
  std::cout << "FCI_IP    : " << FCI_IP << std::endl;
  std::cout << "Read data from " << file_path << std::endl;

  // read desired point from csv file
  std::ifstream ap(file_path);
  int tmp_jump = jump_idx;  
  if (!ap.is_open()){
    std::cout << "ERROR: Failed to Open File" << std::endl; 
  }
  else{
    std::string x;
    std::string y;
    Point pt;
    while(ap.good()){
      getline(ap, x, ',');
      getline(ap, y, '\n');
      pt.x = std::stod(x) + ref_x; // mm -> m
      pt.y = std::stod(y) + ref_y; // mm -> m

      // std::cout << "x:" << x << ", y:" << y << std::endl;
      if (tmp_jump <= 0)
        prep_motion.push_back(pt);
      else
        tmp_jump -= 1;

    }
  }

  // print vector
  // for (auto it=prep_motion.cbegin(); it!=prep_motion.cend(); it++)
  // {
  //   std::cout << "x:" << it->x << ", y:" << it->y << std::endl;
  // }

  std::cout << "Number of waypoints : " << prep_motion.size() << std::endl;
  std::cout << "jump_idx : " << jump_idx << std::endl;
  std::cout << "t_acc : " << acceleration_time << std::endl;

  // prep circle
  std::cout << "prep_dth: " << prep_dth << std::endl;
  std::cout << "prep_r: " << prep_r << std::endl;
  std::cout << "prep_w: " << prep_w << std::endl;
  std::cout << "prep_v: " << prep_v << std::endl;
  std::cout << "prep_a: " << prep_a << std::endl;

  // calculate landing_velocity and parameter for cubic from prep motion
  LinearMotion lmotion;
  if (landing == 1){
    // lmotion = linear_landing_motion(prep_motion, acceleration_time);
    lmotion = linear_landing_motion(prep_x_0, prep_v_0, prep_a_0, acceleration_time);
    std::cout << "------ Linear Landing Motion ------" << std::endl;
    std::cout << "x_l_0: " << lmotion.x_l_0.x << "," << lmotion.x_l_0.y << std::endl;
    // std::cout << "coeff_x: 0,0," << lmotion.b.x << "," << lmotion.a.x << std::endl;
    // std::cout << "coeff_y: 0,0," << lmotion.b.y << "," << lmotion.a.y << std::endl;
    std::cout << "coeff: 0,0," << lmotion.coeff_3 << "," << lmotion.coeff_4 << std::endl;
    std::cout << "===================================" << std::endl;
  }
  else if (landing == 2){
    std::cout << "===================================" << std::endl;
  }
  
  // for print
  std::ofstream log_data("/home/hj/log_data.csv");
  log_data << "t" << "," << "command_success_rate" << ","
           // << "dq[0]" << "," << "dq[1]" << "," << "dq[2]" << "," << "dq[3]" << "," << "dq[4]" << "," << "dq[5]" << "," << "dq[6]" << ","
           // << "dq_d[0]" << "," << "dq_d[1]" << "," << "dq_d[2]" << "," << "dq_d[3]" << "," << "dq_d[4]" << "," << "dq_d[5]" << "," << "dq_d[6]" << ","
           // << "ddq_d[0]" << "," << "ddq_d[1]" << "," << "ddq_d[2]" << "," << "ddq_d[3]" << "," << "ddq_d[4]" << "," << "ddq_d[5]" << "," << "ddq_d[6]" << ","
           << "tau_J[0]" << "," << "tau_J[1]" << "," << "tau_J[2]" << "," << "tau_J[3]" << "," << "tau_J[4]" << "," << "tau_J[5]" << "," << "tau_J[6]"
               // << "measured_x" << "," << "measured_y" << ","
               // << "commanded_x" << "," << "commanded_y" << ","
               // << "commanded_xd" << "," << "commanded_yd" << ","
               // << "commanded_xdd" << "," << "commanded_ydd"
               << std::endl;


  try {
    // Connect to robot.
    franka::Robot robot(FCI_IP);
    setDefaultBehavior(robot);

    // First move the robot to a suitable joint configuration
    std::array<double, 3> x_goal = {{lmotion.x_l_0.x, lmotion.x_l_0.y, 0.4}};
    CMotionGenerator motion_generator(0.1, x_goal);
    std::cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    robot.control(motion_generator);
    std::cout << "Finished moving to initial joint configuration." << std::endl;
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
    double desired_x = 0.0;
    double desired_y = 0.0;
    enum MotionStage stage = LANDING_LINEAR;
    double t = 0.0;
    int i = 0;

    double diff_x = 0.0;
    double diff_y = 0.0;
    bool prep_start = true;

    // prep circle
    double prep_t0 = 0.0;
    
    // Define callback function to send Cartesian pose goals to get inverse kinematics solved.
    auto cartesian_pose_callback = [=, &t, &i, &initial_pose, &desired_x, &desired_y, &stage, &log_data, &diff_x, &diff_y, &prep_start, &prep_t0](
                                       const franka::RobotState& robot_state,
                                       franka::Duration period) -> franka::CartesianPose {
      // Update time.
      t += period.toSec();

      // Read the initial pose to start the motion from in the first time step.
      if (t == 0.0) {
        initial_pose = robot_state.O_T_EE_c;
      }
      
      if (period.toMSec() > 1){
        std::cout << "*" << period.toMSec() << std::endl;
      }

      // Decide motion stage
      // if (stage == LANDING_ACC && /*&& check_landing_velocity_reached(robot_state, landing_velocity)*/){
      if (stage == LANDING_LINEAR && t >= acceleration_time){
        stage = PREP_CIRCLE;
      }

      if (stage == LANDING_READY && t >= acceleration_time + DT*20){
        stage = PREP;
      }

      if (i >= prep_motion.size() - 1){
        stage = FINISHED;
        std::cout << "time to finish" << std::endl;
      }

      franka::CartesianPose pose_desired = initial_pose;
      double d_n;
      switch (stage) {
        case LANDING_LINEAR:
            //linear
            // desired_x = lmotion.x_l_0.x + (lmotion.a.x*std::pow(t, 4)/4.0 + lmotion.b.x*std::pow(t, 3)/3.0);
            // desired_y = lmotion.x_l_0.y + (lmotion.a.y*std::pow(t, 4)/4.0 + lmotion.b.y*std::pow(t, 3)/3.0);
            // std::cout << "(l) desired:" << desired_x << "," << desired_y << std::endl;
            // linear norm
            d_n = lmotion.coeff_4*std::pow(t, 4)/4.0 + lmotion.coeff_3*std::pow(t, 3)/3.0;
            desired_x = lmotion.x_l_0.x + d_n*std::cos(lmotion.th_n);
            desired_y = lmotion.x_l_0.y + d_n*std::sin(lmotion.th_n);
          break;
        
        case LANDING_CIRCLE:
          break;

        case LANDING_SMOOTH:
          break;

        case LANDING_READY:
            desired_x += lmotion.v_p_0.x*period.toSec();
            desired_y += lmotion.v_p_0.y*period.toSec();
            // std::cout << "LANDING_READY---------------" << std::endl;  
          break;
        
        case PREP:
          if (prep_start){
            diff_x = robot_state.O_T_EE[12] - prep_motion[i].x;
            diff_y = robot_state.O_T_EE[13] - prep_motion[i].y;
            std::cout << "PREP START---------------" << std::endl;
            std::cout << "[v_l_T] measured: " << robot_state.O_dP_EE_c[0] << "," << robot_state.O_dP_EE_c[1] 
                      << "  desired:" << lmotion.v_p_0.x << "," << lmotion.v_p_0.y << std::endl;
            std::cout << "[a_l_T] measured: " << robot_state.O_ddP_EE_c[0] << "," << robot_state.O_ddP_EE_c[1] 
                      << "  desired:" << lmotion.a_p_0.x << "," << lmotion.a_p_0.y << std::endl;
            prep_start = false;
          }

          i += period.toMSec();  // in case of packet loss happens
          // desired_x = prep_motion[i].x + diff_x;
          // desired_y = prep_motion[i].y + diff_y;
          desired_x = prep_motion[i].x + diff_x;
          desired_y = prep_motion[i].y + diff_y;

          std::cout << "(p) measured:" << robot_state.O_T_EE[12] << "," << robot_state.O_T_EE[13]
                    << "  desired:" <<  desired_x << "," << desired_y
                    << "  acc:" << robot_state.O_ddP_EE_c[0] << "," << robot_state.O_ddP_EE_c[1] << std::endl;
          break;
        
        case PREP_CIRCLE:
          if (prep_start){
            prep_t0 = t;
            diff_x = robot_state.O_T_EE[12] - ref_x;
            diff_y = robot_state.O_T_EE[13] - ref_y;
            std::cout << "PREP START---------------" << std::endl;
            std::cout << "[v_l_T] measured: " << robot_state.O_dP_EE_c[0] << "," << robot_state.O_dP_EE_c[1] 
                      << "  desired:" << lmotion.v_p_0.x << "," << lmotion.v_p_0.y << std::endl;
            std::cout << "[a_l_T] measured: " << robot_state.O_ddP_EE_c[0] << "," << robot_state.O_ddP_EE_c[1] 
                      << "  desired:" << lmotion.a_p_0.x << "," << lmotion.a_p_0.y << std::endl;
            prep_start = false;
          }
            desired_x = ref_x + prep_r*std::cos(prep_dth*(t-prep_t0)) - prep_r ;
            desired_y = ref_y + prep_r*std::sin(prep_dth*(t-prep_t0));
            std::cout << "(p) measured:" << robot_state.O_T_EE[12] << "," << robot_state.O_T_EE[13]
                    << "  desired:" <<  desired_x << "," << desired_y
                    << "  acc:" << robot_state.O_ddP_EE_c[0] << "," << robot_state.O_ddP_EE_c[1] << std::endl;
          break;
        case FINISHED:
          std::cout << "(f)" << std::endl;
          pose_desired.O_T_EE[12] = desired_x;
          pose_desired.O_T_EE[13] = desired_y;
          return franka::MotionFinished(pose_desired);  // 꼭 pose_desired 넘겨줘야 되나????
      }

      // log data
      log_data << robot_state.time.toMSec() << "," << robot_state.control_command_success_rate << ","
               // << robot_state.dq[0] << "," << robot_state.dq[1] << "," << robot_state.dq[2] << "," << robot_state.dq[3] << ","
               // << robot_state.dq[4] << "," << robot_state.dq[5] << "," << robot_state.dq[6] << ","
               // << robot_state.dq_d[0] << "," << robot_state.dq_d[1] << "," << robot_state.dq_d[2] << "," << robot_state.dq_d[3] << ","
               // << robot_state.dq_d[4] << "," << robot_state.dq_d[5] << "," << robot_state.dq_d[6] << ","
               // << robot_state.ddq_d[0] << "," << robot_state.ddq_d[1] << "," << robot_state.ddq_d[2] << "," << robot_state.ddq_d[3] << ","
               // << robot_state.ddq_d[4] << "," << robot_state.ddq_d[5] << "," << robot_state.ddq_d[6] << ","
               // << robot_state.tau_J[0] << "," << robot_state.tau_J[1] << "," << robot_state.tau_J[2] << "," << robot_state.tau_J[3] << ","
               // << robot_state.tau_J[4] << "," << robot_state.tau_J[5] << "," << robot_state.tau_J[6] << ","
               // << robot_state.O_T_EE[12] << "," << robot_state.O_T_EE[13] << ","
               // << robot_state.O_T_EE_c[12] << "," << robot_state.O_T_EE_c[13] << ","
               // << robot_state.O_dP_EE_c[0] << "," << robot_state.O_dP_EE_c[1] << ","
               // << robot_state.O_ddP_EE_c[0] << "," << robot_state.O_ddP_EE_c[1]
               << std::endl;

      // Send desired pose.
      pose_desired.O_T_EE[12] = desired_x;
      pose_desired.O_T_EE[13] = desired_y;
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
    log_stream << franka::logToCSV(log);
    std::cout << "Log file written to: " << log_file.path() << std::endl;
  } catch (...) {
    std::cout << "Failed to write log file." << std::endl;
  }
}