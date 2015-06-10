#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "roscopter/FilteredPosition.h"
#include "roscopter/Attitude.h"
#include "roscopter/RC.h"
#include <sstream>
#include <string>
#include <time.h>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#define LOITER 1420
//#define LOITER 1900

using namespace std_msgs;
using namespace cv;

ros::Publisher chatter_pub;
ros::Publisher rc_pub;

ros::Subscriber subZ;
ros::Subscriber subY;
ros::Subscriber subX;
ros::Subscriber subPhi;
ros::Subscriber sub_state_machine;

roscopter::RC rc_control;

Int32 relativeAltitude;
Float32 errorX;
Float32 errorY;
Float32 errorPhi;

//CONSTS//
const float sample_time = .1; //s
const float gravity = 9.81; // En m/s*s
const float mass = 5; //En kilos
const float lambda_xy = 0.9;
const float lambda_z = 0.5;
const float lambda_phi = 0.1;
const float k_x = -1;
const float k_y = -1;
const float k_z = -0.8;
const float k_phi = -0.5;
const float magic_control_factor_xy = 150;
const float magic_control_factor_z = 150;
const float magic_control_factor_phi = 90;
const float mid_channel = 1500;

//CONTROL VARS//
bool CONTROL_ON = false;
bool SEEN = false;
float u_eq_x = 0;
float u_eq_y = 0;
float u_eq_phi = 0;
float u_eq_z = 0;

float u_x = 0;
float u_y = 0;
float u_z = 0;
float u_phi = 0;

//Z Control//
const float z_ref = 7; //En metros
float z_actual = 0; //En metros
float z_error_actual = 0; 
float z_error_anterior = 0; 
float velocity_error_z = 0; 

//X Control//
float x_error_actual = 0; 
float x_error_anterior = 0; 
float velocity_error_x = 0; 

//Y Control//
float y_error_actual = 0; 
float y_error_anterior = 0; 
float velocity_error_y = 0; 

//Phi Control//
float phi_error_actual = 0; 
float phi_error_anterior = 0; 
float velocity_error_phi = 0; 

string s1 = "Error X: ";
string s2 = "    U_X: ";
string s3 = "Control: ";

string s4 = "Error Y: ";
string s5 = "    U_Y: ";
string s6 = "Control: ";

string s7 = "Error Z: ";
string s8 = "    U_Z: ";
string s9 = "Control: ";

string s10 = "Error P: ";
string s11 = "    U_P: ";
string s12 = "Control: ";

string s13 = "Jaguar SEEN = ";

void sendControl()
{
  double roll = 1500;
  double pitch = 1500;
  double yaw = 1475;
  double throttle = 1500;

  if (SEEN)
  {
    roll = 1500-(u_x*magic_control_factor_xy);
    pitch = 1500+(u_y*magic_control_factor_xy);
  }
  yaw = 1500-(u_phi*magic_control_factor_phi);
  throttle = 1500+(u_z*magic_control_factor_z);
  
  if (roll > 1700)
    roll = 1700;
  if (roll < 1300)
    roll = 1300;

  if (pitch > 1700)
    pitch  = 1700;
  if (pitch  < 1300)
    pitch  = 1300;

  if (yaw > 1600)
    yaw  = 1600;
  if (yaw  < 1400)
    yaw  = 1400;

  if (throttle > 1550)
    throttle  = 1550;
  if (throttle  < 1300)
    throttle  = 1300;

  rc_control.channel[0] = roll;
  rc_control.channel[1] = pitch;
  rc_control.channel[2] = 1500; //throttle
  rc_control.channel[3] = yaw; 
  rc_control.channel[4] = LOITER; // 5: 1900 -> Loiter
  rc_control.channel[5] = 0;
  rc_control.channel[6] = 0;
  rc_control.channel[7] = 0;

  s3 = "Control: ";
  std::string str1 = boost::lexical_cast<std::string>(roll);
  s3 += str1; 
  
  s6 = "Control: ";
  std::string str2 = boost::lexical_cast<std::string>(pitch);
  s6 += str2;  
  
  s9 = "Control: ";
  std::string str3 = boost::lexical_cast<std::string>(throttle);
  s9 += str3; 
  
  s12 = "Control: ";
  std::string str4 = boost::lexical_cast<std::string>(yaw);
  s12 += str4; 

  ROS_INFO("    Roll = %f", roll);
  ROS_INFO("   Pitch = %f", pitch);
  ROS_INFO("Throttle = %f", throttle);
  ROS_INFO("     Yaw = %f", yaw);

  rc_pub.publish(rc_control);
}

void emergencyStop()
{

  s3 = "Control: 0";
  
  s6 = "Control: 0";
  
  s9 = "Control: 0";
  
  s12 = "Control: 0";

  rc_control.channel[0] = 0;
  rc_control.channel[1] = 0;
  rc_control.channel[2] = 0;
  rc_control.channel[3] = 0;
  rc_control.channel[4] = 0; 
  rc_control.channel[5] = 0;
  rc_control.channel[6] = 0;
  rc_control.channel[7] = 0;
  rc_pub.publish(rc_control);
}

void CallbackStateMachine(const std_msgs::Int32::ConstPtr& msg)
{
  Int32 state_machine_msg = *msg;

  if ((int) state_machine_msg.data == 0)
  {
    SEEN = false;
    s13 = "Jaguar SEEN = NO";
  }
  else 
  {
    SEEN = true;
    s13 = "Jaguar SEEN = YES";
  }
}

void CallbackZ(const roscopter::FilteredPosition::ConstPtr& msg)
{
  //ROS_INFO("relative_altitude: [%d]", msg->relative_altitude);
  roscopter::FilteredPosition filtered = *msg;
  relativeAltitude.data =  filtered.relative_altitude;
  
  z_actual = ((float) relativeAltitude.data)/1000;  
}

void CallbackX(const std_msgs::Float32::ConstPtr& msg)
{
  //ROS_INFO("error_X: [%f]", msg->data);  
  errorX = *msg;
  x_error_actual = (float) errorX.data;

  s1 = "Error X: ";
  std::string str = boost::lexical_cast<std::string>(x_error_actual);
  s1 += str;
}

void CallbackY(const std_msgs::Float32::ConstPtr& msg)
{
  //ROS_INFO("error_Y: [%f]", msg->data)
  errorY = *msg;
  y_error_actual = (float) errorY.data;

  s4 = "Error Y: ";
  std::string str = boost::lexical_cast<std::string>(y_error_actual);
  s4 += str;
}

void CallbackPhi(const std_msgs::Float32::ConstPtr& msg)
{
  //ROS_INFO("error_Phi: [%f]", msg->data);
  errorPhi = *msg;
  phi_error_actual = (float) errorPhi.data;

  s10 = "Error P: ";
  std::string str = boost::lexical_cast<std::string>(phi_error_actual);
  s10 += str;
}

void calculate_u_z()
{
  z_error_anterior = z_error_actual;
  z_error_actual = z_ref - z_actual;
  velocity_error_z = (z_error_actual - z_error_anterior)/.65;

  //0. Olds
  //u_eq_z = mass*(-lambda*velocity_error_z+gravity);
  //u_eq_z = mass*(-lambda*velocity_error_z);
  
  //1.
  u_eq_z = z_error_actual - (lambda_z*velocity_error_z);
  
  //2. Z as reference
  //u_eq_z = z_ref - (lambda_z*velocity_error_z);
  
  u_z = u_eq_z - k_z*(2/M_PI)*atan(velocity_error_z+lambda_z*z_error_actual);

  //ROS_ERROR("u_eq_z : %f", u_eq_z);
  //ROS_ERROR("u_z : %f", u_z);
  s7 = "Error Z: ";
  std::string str = boost::lexical_cast<std::string>(z_error_actual);
  s7 += str; 

  s8 = "    U_Z: ";
  std::string str2 = boost::lexical_cast<std::string>(u_z);
  s8 += str2; 
}

void calculate_u_x()
{
  velocity_error_x = (x_error_actual - x_error_anterior)/.65;

  u_eq_x = asin(-lambda_xy*velocity_error_x/gravity);
  u_x = u_eq_x - k_x*(2/M_PI)*atan(velocity_error_x+lambda_xy*x_error_actual);
  
  //ROS_ERROR("Error actual: %f", x_error_actual);
  //ROS_ERROR("u_eq_x : %f", u_eq_x);
  //ROS_ERROR("u_x : %f", u_x);

  s2 = "    U_X: ";
  std::string str = boost::lexical_cast<std::string>(u_x);
  s2 += str; 

  x_error_anterior = x_error_actual;
}

void calculate_u_y()
{
  velocity_error_y = (y_error_actual - y_error_anterior)/.65;

  u_eq_y = asin(-lambda_xy*velocity_error_y/gravity);
  u_y = u_eq_y - k_y*(2/M_PI)*atan(velocity_error_y+lambda_xy*y_error_actual);
  
  //ROS_ERROR("u_eq_y : %f", u_eq_y);
  //ROS_ERROR("u_y : %f", u_y);
  s5 = "    U_Y: ";
  std::string str = boost::lexical_cast<std::string>(u_y);
  s5 += str; 

  y_error_anterior = y_error_actual;
}

void calculate_u_phi()
{
  velocity_error_phi = (phi_error_actual - phi_error_anterior)/.65;

  u_eq_phi = phi_error_actual-(lambda_phi*velocity_error_phi);
  u_phi = u_eq_phi - k_phi*(2/M_PI)*atan(velocity_error_phi+lambda_phi*phi_error_actual);
  
  //ROS_ERROR("u_eq_phi : %f", u_eq_phi);
  //ROS_ERROR("u_phi : %f", u_phi);
  s11 = "    U_P: ";
  std::string str = boost::lexical_cast<std::string>(u_phi);
  s11 += str; 

  phi_error_anterior = phi_error_actual;
}

void drawVars(cv::Mat image)
{
  cv::putText(image, s1, cv::Point(30,60), 1, 1, cv::Scalar(0,0,0), 2);
  cv::putText(image, s2, cv::Point(30,80), 1, 1, cv::Scalar(0,0,0), 2);
  cv::putText(image, s3, cv::Point(30,100), 1, 1, cv::Scalar(0,0,0), 2);

  cv::putText(image, s4, cv::Point(30,140), 1, 1, cv::Scalar(0,0,0), 2);
  cv::putText(image, s5, cv::Point(30,160), 1, 1, cv::Scalar(0,0,0), 2);
  cv::putText(image, s6, cv::Point(30,180), 1, 1, cv::Scalar(0,0,0), 2);

  cv::putText(image, s7, cv::Point(200,60), 1, 1, cv::Scalar(0,0,0), 2);
  cv::putText(image, s8, cv::Point(200,80), 1, 1, cv::Scalar(0,0,0), 2);
  cv::putText(image, s9, cv::Point(200,100), 1, 1, cv::Scalar(0,0,0), 2);

  cv::putText(image, s10, cv::Point(200,140), 1, 1, cv::Scalar(0,0,0), 2);
  cv::putText(image, s11, cv::Point(200,160), 1, 1, cv::Scalar(0,0,0), 2);
  cv::putText(image, s12, cv::Point(200,180), 1, 1, cv::Scalar(0,0,0), 2);
  
  cv::putText(image, s13, cv::Point(100,220), 1, 1, cv::Scalar(0,0,0), 2);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sliding_control");
  ros::NodeHandle n;
  // Real Time 30 FPS
  ros::Rate loop_rate(100);
 
  chatter_pub = n.advertise<std_msgs::Int32>("sliding_control", 1000);
  rc_pub = n.advertise<roscopter::RC>("send_rc", 1);

  subZ = n.subscribe("filtered_pos", 1000, CallbackZ);
  subY = n.subscribe("error_y", 1000, CallbackY);
  subX = n.subscribe("error_x", 1000, CallbackX);
  subPhi = n.subscribe("error_phi", 1000, CallbackPhi);
  sub_state_machine = n.subscribe("state_machine", 1000, CallbackStateMachine);
  clock_t begin_time = clock();
  
  cv::Mat image = cv::imread("/home/erobots/workspace/src/sliding_control/src/white.jpg");

  if (!image.empty())
    {
      cv::imshow("Sliding Control", image);
    }

  while(ros::ok())  
  {
    ros::spinOnce();
    loop_rate.sleep();

    // Keyboard Interrupt
    char key;
    if (key = cv::waitKey(1))
    {
      switch(key) {
      // Numbers: Control of Helicopter
      case 'c': CONTROL_ON = !CONTROL_ON; break;
      case 'q': CONTROL_ON = false; emergencyStop(); break;
      default: break;
      }
    }
    

    if ((float( clock () - begin_time ) /  (CLOCKS_PER_SEC/1000)) >= (sample_time*1000))
    {
      //ROS_ERROR("han pasado: 100 ms");
      calculate_u_z();
      calculate_u_y();
      calculate_u_x();
      calculate_u_phi();
      begin_time = clock();
    
      if(CONTROL_ON){
        sendControl();
        image = cv::imread("/home/erobots/workspace/src/sliding_control/src/white.jpg", CV_LOAD_IMAGE_COLOR);
        cv::putText(image, "CONTROL ON", cv::Point(30,30), 1, 1, cv::Scalar(0,0,0), 2);
      }
      else{
        emergencyStop();
        image = cv::imread("/home/erobots/workspace/src/sliding_control/src/white.jpg", CV_LOAD_IMAGE_COLOR);
        cv::putText(image, "CONTROL OFF", cv::Point(30,30), 1, 1, cv::Scalar(0,0,0), 2);
      }

      drawVars(image);
    }


    if (!image.empty())
    {
      cv::imshow("Sliding Control", image);
    }

  }

  cv::destroyWindow("RC Controller");

  return 0;
}

