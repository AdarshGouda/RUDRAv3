
//Date: 14-Apr-2021
//Intent: Building on v1, this is to integrate cmd_vel to drive the motor.
//        Microcontroler: Teensy 4.1 --> I doubt we can publish odom using Arduino Mega

//References: https://github.com/XRobots/ReallyUsefulRobot/blob/main/arduino/RUR099/RUR099.ino



#include <Sabertooth.h> //Modified "Serial" to "Serial1" in the header file to enable default Serial on TX1 for Mega.
//This is because ROS communicates on Serial0 by default at a different baud rate.

#include <ros.h>
#include "geometry_msgs/Twist.h"
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/UInt16.h>

#include <Wire.h>

#include <Encoder.h>



ros::NodeHandle nh;
geometry_msgs::TransformStamped t;
nav_msgs::Odometry odom_msg;
ros::Publisher odom_pub("odom", &odom_msg);
tf::TransformBroadcaster broadcaster;

//Variables initialization

Sabertooth STRight(128), STLeft(129); //Change the pins on Sabertooth to set 128 and 129 on each ST board, use the wizard to determine this.


// tf variables to be broadcast

double x = 0;
double y = 0;
double theta = 0;

char base_link[] = "/base_link";
char odom[] = "/odom";



const double radius = 0.0675;               //Wheel radius, in m    // Verify these dimensions again
const double wheelbase = 0.23;              //Wheelbase, in m

const double encoder_cpr = 330;             //Time time around I will have to measure this.


double speed_req = 0;                         //Desired linear speed for the robot, in m/s
double angular_speed_req = 0;                 //Desired angular speed for the robot, in rad/s

double speed_req_left = 0;                    //Desired speed for left wheel in m/s

double speed_act_left1 = 0;                    //Actual speed for left wheel in m/s
double speed_cmd_left1 = 0;                    //Command speed for left wheel in m/s


double speed_act_left2 = 0;                    //Actual speed for left wheel in m/s
double speed_cmd_left2 = 0;                    //Command speed for left wheel in m/s

double speed_req_right = 0;                   //Desired speed for right wheel in m/s

double speed_act_right1 = 0;                   //Actual speed for right wheel in m/s
double speed_cmd_right1 = 0;                   //Command speed for right wheel in m/s


double speed_act_right2 = 0;                   //Actual speed for right wheel in m/s
double speed_cmd_right2 = 0;                   //Command speed for right wheel in m/s

const double max_speed = 1.20;
//const double min_speed = 0.85;                //Max speed in m/s



int PWM_leftMotor1 = 0;                     //PWM command for left motor
int PWM_rightMotor1 = 0;                    //PWM command for right motor
int PWM_leftMotor2 = 0;                     //PWM command for left motor
int PWM_rightMotor2 = 0;                    //PWM command for right motor

//Note: PWM signal varies from -127 to 0 to +127 for Sabertooth.

float speedL1 = 0.0;        //These variables are included for troubleshooting.
float speedR1 = 0.0;
float speedL2 = 0.0;
float speedR2 = 0.0;

//Encoder declareation...

Encoder EncR1(40,41);
long R1oldPosition = -999;
long R1newPosition = -999;

Encoder EncL1(13,14);
long L1oldPosition = -999;
long L1newPosition = -999;

Encoder EncL2(5,6);
long L2oldPosition = -999;
long L2newPosition = -999;

Encoder EncR2(19,18);
long R2oldPosition = -999;
long R2newPosition = -999;


//--------------------------------------------------------------------------------------

//function that will be called when receiving command from /cmd_vel topic

void handle_cmd (const geometry_msgs::Twist& cmd_vel) {
  noCommLoops = 0;                                                  //Reset the counter for number of main loops without communication

  speed_req = cmd_vel.linear.x;                                     //Extract the commanded linear speed from the message

  angular_speed_req = cmd_vel.angular.z;                            //Extract the commanded angular speed from the message

  speed_req_left = speed_req - angular_speed_req * (wheelbase / 2); //Calculate the required speed for the left motor to comply with commanded linear and angular speeds
  speed_req_right = speed_req + angular_speed_req * (wheelbase / 2); //Calculate the required speed for the right motor to comply with commanded linear and angular speeds

}

//-----------------------------------------------------------------------------------------------------

void setup () {

  SabertoothTXPinSerial.begin(9600); //Again, the serial communication is happening on TX1 and not the default TX0

  nh.initNode();                            //init ROS node
  nh.getHardware()->setBaud(57600);         //set baud for ROS serial communication, Note: This is Serial0
  nh.subscribe(cmd_vel);                    //suscribe to ROS topic for velocity commands
  nh.advertise(speed_pub);                  //prepare to publish speed in ROS topic


}

//-----------------------------------------------------------------------------------------------------



void loop() {


  nh.spinOnce();


  if ((millis() - lastMilli) >= LOOPTIME)

  { // enter timed loop


    // Insert code here to implement PID convert speed_req to speed_adjusted


    // Insert code here to convert adjusted_speed_adjusted to PWM signal -127 > pwm < 127


    // Insert code to calculate each wheel rpm based on Encoder implementation. For this Counts Per Revolution is needed.


    // Insert code here for IMU data reading



    
    //MOVE_BASE
    //Motor Left 1

    if (noCommLoops >= noCommLoopMax) {                   //Stopping if too much time without command

      STLeft.motor(1, 0);

    }
    else if (L1disable == 1) {                            //Troubleshooting

      STLeft.motor(1, 0);

    }
    else if (speed_req_left == 0) {                       //Stopping

      STLeft.motor(1, 0);
    }
    else {                                                //Going forward

      STLeft.motor(1, PWM_leftMotor1);

    }


    //Motor Left 2


    if (noCommLoops >= noCommLoopMax) {                   //Stopping if too much time without command

      STLeft.motor(2, 0);

    }
    else if (L2disable == 1) {                            //Troubleshooting

      STLeft.motor(2, 0);

    }
    else if (speed_req_left == 0) {                       //Stopping

      STLeft.motor(2, 0);
    }
    else {                                                //Going forward

      STLeft.motor(2, PWM_leftMotor1);

    }

    //Motor Right 1

    if (noCommLoops >= noCommLoopMax) {                   //Stopping if too much time without command

      STRight.motor(1, 0);

    }
    else if (R1disable == 1) {                            //Troubleshooting

      STRight.motor(1, 0);

    }
    else if (speed_req_left == 0) {                       //Stopping

      STRight.motor(1, 0);
    }
    else {                                                //Going forward

      STRight.motor(1, PWM_leftMotor1);

    }

    //Motor Right 2
    if (noCommLoops >= noCommLoopMax) {                   //Stopping if too much time without command

      STRight.motor(2, 0);

    }
    else if (R2disable == 1) {                            //Troubleshooting

      STRight.motor(2, 0);

    }
    else if (speed_req_left == 0) {                       //Stopping

      STRight.motor(2, 0);
    }
    else {                                                //Going forward

      STRight.motor(2, PWM_leftMotor1);

    }

    noCommLoops++;
    if (noCommLoops == 65535) {
      noCommLoops = noCommLoopMax;
    }

    //publishOdom(LOOPTIME);

    //publishIMU(LOOPTIME);

  }


}
