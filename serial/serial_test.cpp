// ls -la /dev/ttyUSB*
// sudo chmod +x /dev/ttyUSB*
// g++ -o serial_test serial_test.cpp
#include <iostream>
#include <cstring>
#include <chrono>
#include <thread>

#include <unistd.h>
#include <sys/types.h>
#include <sys/poll.h>
#include <termios.h>
#include <fcntl.h>

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

typedef enum _LOGGER {
   DEBUG=0,
   INFO,
   WARN,
   ERROR
}LOGGER;

#define LOGGER_LEVEL INFO

int main(int argc, char** argv)
{
   // 통신 포트를 파일 개념으로 사용하기 위한 디스크립터 입니다.
   // 이 파일 디스크립터를 이용하여 표준 입출력 함수를 이용할 수 있습니다. 
   int fd;

   // 시리얼 통신 환경을 설정하기 위해 termios 구조체를 선언했습니다. 
   struct termios newtio;

   // /dev/ttyUSB0 사용하기 위해 open()함수를 사용합니다.
   // O_RDWR은 파일 디스크립터인 fd를 읽기와 쓰기 모드로 열기 위한 지정이다
   fd = open("/dev/ttyUSB3", O_RDWR | O_NOCTTY | O_RSYNC);

   // 시리얼 통신환경 설정을 위한 구조체 변수 newtio 값을 0 바이트로 깨끗이 채웁니다. 
   std::memset( &newtio, 0, sizeof(newtio) );   

   newtio.c_cflag = B115200;   // 통신 속도 115200 
   newtio.c_cflag |= CS8;      // 데이터 비트가 8bit 
   newtio.c_cflag |= CLOCAL;   // 외부 모뎀을 사용하지 않고 내부 통신 포트 사용 
   newtio.c_cflag |= CREAD;    // 쓰기는 기본, 읽기도 가능하게 
   newtio.c_iflag = 0;         // parity 비트는 없음
   newtio.c_oflag = 0;
   newtio.c_lflag = 0;
   newtio.c_cc[VTIME] = 0; 
   newtio.c_cc[VMIN] = 1; 

   tcflush(fd, TCIFLUSH);
   tcsetattr(fd, TCSANOW, &newtio);   // 포트에 대한 통신 환경을 설정합니다. 
   
   // FT 값 요구
   unsigned char req_msg[11];
   std::memset(req_msg, 0x00, sizeof(req_msg));
   req_msg[0] = 0x55; 
   req_msg[1] = 0x0B;
   req_msg[9] = 0x0B;
   req_msg[10] = 0xAA;
   write(fd, req_msg, 11);

   // FT 값 읽어오기
   unsigned char start[1];
   unsigned char buffer[18];
   unsigned char checksum = 0x00;
   short ft_raw[6];
   float ft[6];
   float ft_initial[6];
   unsigned char overload;
   int num_of_initial = 30;
   int num = 0;

   // 
   double force_xy = 0.0;
   double prev_force_xy = 0.0;
   double dforce_xy = 0.0;
   double prev_dforce_xy = 0.0;
   double dforce_xy_thresh = 100.0;

   // ROS
   ros::init(argc, argv, "serial_test");
   ros::NodeHandle nh;
   ros::Publisher force_pub = nh.advertise<std_msgs::Float64MultiArray>("force", 1);
   ros::Rate sensor_rate(200);

   printf("Initialize FT \n");
   std::memset(ft_initial, 0x00, sizeof(ft_initial));
   for (int i=0; i<num_of_initial; i++){
      
      // read until get 0x55
      while (true) {
         std::memset(start, 0x00, sizeof(start));
         read(fd, start, sizeof(start));
         if (start[0] == uint8_t(0x55))
            break;
      }
      std::memset(start, 0x00, sizeof(buffer));
      read(fd, buffer, sizeof(buffer));

      // print raw msg
      if (LOGGER_LEVEL <= DEBUG){
         printf("55 ");
         for (int j=0; j<18; j++)
            printf("%x ", buffer[j]);
         printf("\n");
      }

      // convert msg to ft values
      checksum = 0x00;
      for (int i=0; i<16; i++)   checksum += buffer[i];
      if (buffer[17] == 0xAA && buffer[16] == checksum) {
         for (int i=0; i<6; i++) {
            ft_raw[i] = (signed short)(uint8_t(buffer[2*i+1])*256 + uint8_t(buffer[2*i+2]));
            if (i<3)
               ft[i] = (float)(ft_raw[i]/50.0);
            else
               ft[i] = (float)(ft_raw[i]/2000.0);
            ft_initial[i] += ft[i];
         }
         overload = uint8_t(buffer[13]);      
         
         if (LOGGER_LEVEL <= DEBUG){
            if (overload & 0x01)
               printf("overload tz\n");
            else if (overload & 0x02)
               printf("overload ty\n");
            else if (overload & 0x04)
               printf("overload tx\n");
            else if (overload & 0x08)
               printf("overload fz\n");
            else if (overload & 0x10)
               printf("overload fy\n");
            else if (overload & 0x20)
               printf("overload fz\n");
         }
         num++;
      }
   }

   for (int j=0; j<6; j++) {
      ft_initial[j] /= num;
   }
   printf("fx: %2.3f, fy: %2.3f, fz: %2.3f, ", ft_initial[0], ft_initial[1], ft_initial[2]);
   printf("tx: %2.3f, ty: %2.3f, tz: %2.3f\n", ft_initial[3], ft_initial[4], ft_initial[5]);


   

   printf("Read From FT \n");
   tcflush (fd, TCIFLUSH);
   std::this_thread::sleep_for(std::chrono::microseconds(5000));
   
   std::chrono::time_point<std::chrono::system_clock> initial = std::chrono::system_clock::now();
   std::chrono::time_point<std::chrono::system_clock> previous = initial;
   std::chrono::time_point<std::chrono::system_clock> current;
   std::chrono::time_point<std::chrono::system_clock> start_to_read;
   std::chrono::time_point<std::chrono::system_clock> end_to_read;
   int num_of_data = 0;
   
   while (true) {

      start_to_read = std::chrono::system_clock::now();

      // read until get 0x55
      while (true) {
         std::memset(start, 0x00, sizeof(start));
         read(fd, start, sizeof(start));
         if (start[0] == uint8_t(0x55)) {
            if (LOGGER_LEVEL <= DEBUG) {
               printf("\n");
            }
            break;
         }
         else {
            if (LOGGER_LEVEL <= DEBUG) {
               printf("%x ", start[0]);
            }
         }
      }

      std::memset(start, 0x00, sizeof(buffer));
      read(fd, buffer, sizeof(buffer));

      // print raw msg
      if (LOGGER_LEVEL <= DEBUG){
         printf("R 55 ");
         for (int j=0; j<18; j++)
            printf("%x ", buffer[j]);
         printf("\n");
      }

      // convert msg to ft values
      checksum = 0x00;
      for (int i=0; i<15; i++)   checksum += buffer[i];
      if (buffer[17] == 0xAA && buffer[16] == checksum) {
         for (int i=0; i<6; i++) {
            ft_raw[i] = (signed short)(uint8_t(buffer[2*i+1])*256 + uint8_t(buffer[2*i+2]));
            if (i<3)
               ft[i] = (float)(ft_raw[i]/50.0) - ft_initial[i];
            else
               ft[i] = (float)(ft_raw[i]/2000.0) - ft_initial[i];
         }
         overload = uint8_t(buffer[13]);

         printf("fx: %2.3f, fy: %2.3f, fz: %2.3f, ", ft[0], ft[1], ft[2]);
         // printf("tx: %2.3f, ty: %2.3f, tz: %2.3f ", ft[3], ft[4], ft[5]);
         
         current = std::chrono::system_clock::now();
         double dt = (std::chrono::duration_cast<std::chrono::microseconds>(current-previous).count())/1000.0;
         double time_from_start = (std::chrono::duration_cast<std::chrono::microseconds>(current-initial).count())/1000.0;
         printf("|| time_from_start: %.2fms, dt: %.2fms", time_from_start, dt);
         printf("\n");

         if (LOGGER_LEVEL <= DEBUG){
            if (overload & 0x01)
               printf("overload tz\n");
            else if (overload & 0x02)
               printf("overload ty\n");
            else if (overload & 0x04)
               printf("overload tx\n");
            else if (overload & 0x08)
               printf("overload fz\n");
            else if (overload & 0x10)
               printf("overload fy\n");
            else if (overload & 0x20)
               printf("overload fz\n");
         }

         previous = current;
         num_of_data ++;

         if (num_of_data >= 100000)
            break;

         // sleep for 0.5ms
         end_to_read = std::chrono::system_clock::now();
         int time_for_read = std::chrono::duration_cast<std::chrono::microseconds>(end_to_read-start_to_read).count();
         // std::cout << "time=" << time_for_read << "ms" << std::endl;

         // publish msg
         std_msgs::Float64MultiArray force_msg;
         for (int i=0; i<6; i++)
            force_msg.data.push_back(std::abs(ft[i]));
         force_xy = std::sqrt(ft[0]*ft[0] + ft[1]*ft[1]);
         force_msg.data.push_back(force_xy);

         dforce_xy = (force_xy-prev_force_xy)/(0.005);
         if (std::abs(dforce_xy) < dforce_xy_thresh)
         {
            force_msg.data.push_back(dforce_xy);
            prev_dforce_xy = dforce_xy;
         }   
         else
         {
            force_msg.data.push_back(prev_dforce_xy);
         }
         force_pub.publish(force_msg);
         prev_force_xy = force_xy;

         // std::this_thread::sleep_for(std::chrono::microseconds(4800));
         sensor_rate.sleep();
      }

   }

   double total_time = (std::chrono::duration_cast<std::chrono::microseconds>(current-initial).count())/1000.0;
   double avg_time = total_time/num_of_data;
   printf("\n\ntotal number of data = %d, total time = %fms, avg time = %fms\n", num_of_data, total_time, avg_time);

   close(fd);
   
   return 0; 
}