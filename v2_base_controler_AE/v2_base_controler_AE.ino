/*Create Date: 25-Apr-2021

  //Revision: AE
  //Rev Date: 05-Apr-2021


  //Intent:  for joystick take readings as steering and throttle

*/

//NOTE: It is important that ros.h accompanies this sketch to increase the buffer size to enable publishing odom and imu topics.
//      The default ros.h from ros_lib will not work unless the buffer sizes are changed.


#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif


//ROS
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <rudra/joy_msg.h>

//IMU - /sudo apt-get install ros-melodic-imu-tools

#include <Wire.h>
#include <I2Cdev.h>        // from https://github.com/jrowberg/i2cdevlib/tree/master/Arduino
#include <MPU6050.h>       // from https://github.com/jrowberg/i2cdevlib/tree/master/Arduino

#define ACCEL_SCALE 1 / 16384    // LSB/g  //These Define factors are for MPU6050 datasheet
#define GYRO_SCALE 1 / 131       // LSB/(deg/s)
#define MAG_SCALE 0.3            // uT/LSB
#define G_TO_ACCEL 9.81

MPU6050 accelgyro(0x69);      //Change the address if I2C is on 0x68, which is typically the case with most MPU6050 sensors.
unsigned long time;
int16_t ax, ay, az;
int16_t gx, gy, gz;
float heading;

//Sabertooth

#include <Sabertooth.h> //serial is modified for serial 1  in Sabertooth.h

Sabertooth STRight(128), STLeft(129);

//Encoder

#define ENCODER_OPTIMIZE_INTERRUPTS // comment this out on Non-Teensy boards
#include <Encoder.h>

Encoder EncR1(8, 7);
long R1oldPosition = -999;

Encoder EncL1(2, 3);
long L1oldPosition = -999;

Encoder EncL2(4, 5);
long L2oldPosition = -999;

Encoder EncR2(32, 31);
long R2oldPosition = -999;

//Timing Parameters

unsigned long lastMilli = 0;

#define LOOPTIME  100
const byte noCommLoopMax = 10;                //number of main loops the robot will execute without communication before stopping
unsigned int noCommLoops = 0;                 //main loop without communication counter


/*-----------Base Desing Constants----------------------*/


const double radius = 0.0675;                 //Wheel radius, in m
const double wheelbase = 0.29;                //Wheelbase, in m

const double encoder_cpr = 1683;              //counts per revolution calculated experimentally by moving the robot manually 1m and averaged for 10 times.

/*-----------------Cmd_Vel derivation variables -----------------------*/

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

int PWM_leftMotor1 = 0;                        //PWM command for left motor 1
int PWM_rightMotor1 = 0;                       //PWM command for right motor 1
int PWM_leftMotor2 = 0;                        //PWM command for left motor 2
int PWM_rightMotor2 = 0;                       //PWM command for right motor 2

volatile float pos_left1 = 0;                  //Left motor 1 encoder position
volatile float pos_right1 = 0;                 //Right motor 1 encoder position
volatile float pos_left2 = 0;                  //Left motor 2 encoder position
volatile float pos_right2 = 0;                 //Right motor 2 encoder position

/*---------PID Parameters-------------------*/

bool pid_flag = true;                          //Enable or disable PID, if disabled PWM is calcualted based on a proportinal factor determined thru experimentation

float integral_, derivative_, prev_error_;    // errors Kp, Kd and Ki
int min_val_ = -127;                          //Sabertooth ranges form -127 to 127, O being stop
int max_val_ = 127;

float kp_ = 20.0;                             //tuned parameters for RudraBot
float ki_ = 5.0;
float kd_ = 0.05;

/*------------------Odom Parameters--------------------------*/

double speed_act_left_avg = 0.0;
double speed_act_right_avg = 0.0;

double two_pi = 6.28319;

double x_pos = 0.0;                          //for Odom
double y_pos = 0.0;
double theta = 0.0;

double gyro_x = 0.0;
double gyro_y = 0.0;
double gyro_z = 0.0;

double rate = 10.0;

bool publish_tf = true;
bool fuse_imu = true;

double alpha = 0.5;
double dt = 0.1;
double dx = 0.0;
double dy = 0.0;
double dth_odom = 0.0;
double dth_gyro = 0.0;
double dth = 0.0;
double dxy = 0.0;
double vx = 0.0;
double vy = 0.0;
double vth = 0.0;

char base_link[] = "/base_link";
char odom[] = "/odom";
char imu_link[] = "/imu_link";

double gyro_bias_x = 0.0;
double gyro_bias_y = 0.0;
double gyro_bias_z = 0.0;

double acc_bias_x = 0.0;
double acc_bias_y = 0.0;
double acc_bias_z = 0.0;

/****************** PS2 ***********************/

bool  manual_mode = false;

int  joy_Rx = 128;
int  joy_Ry = 128;
int  joy_Ly = 128;
int  joy_Lx = 128;

int throttle = 0;
int steering = 0;

int leftSpeed = 0;
int rightSpeed = 0;

//------------------------------------------------

ros::NodeHandle nh;

//function that will be called when receiving command from host

void handle_cmd (const geometry_msgs::Twist& cmd_vel) {

  noCommLoops = 0;

  speed_req = cmd_vel.linear.x;                                     //Extract the commanded linear speed from the message

  angular_speed_req = cmd_vel.angular.z;                            //Extract the commanded angular speed from the message

  speed_req_left = speed_req - angular_speed_req * (wheelbase / 2); //Calculate the required speed for the left motor to comply with commanded linear and angular speeds
  speed_req_right = speed_req + angular_speed_req * (wheelbase / 2); //Calculate the required speed for the right motor to comply with commanded linear and angular speeds

}

void joy_cmd (const rudra::joy_msg& msg) {

  manual_mode = msg.SELECT;

  joy_Rx = msg.Rx;
  joy_Ry = msg.Ry;
  joy_Ly = msg.Ly;
  joy_Lx = msg.Lx;

  throttle = map(joy_Ly, 0, 225, 127, -127);
  steering = map(joy_Rx, 0, 225, 127, -127);


}

geometry_msgs::TransformStamped t;
nav_msgs::Odometry odom_msg;
ros::Publisher odom_pub("odom", &odom_msg);
tf::TransformBroadcaster broadcaster;

ros::Subscriber<geometry_msgs::Twist> cmd_vel("cmd_vel", handle_cmd);

ros::Subscriber<rudra::joy_msg> joy_msg("joy_msg", joy_cmd);

sensor_msgs::Imu imu;
ros::Publisher imu_pub("imu/data_raw", &imu);

//https://github.com/OusamaSalafi/catkin_ws/blob/master/src/irobot/src/imu.cpp
//https://github.com/bandasaikrishna/orientations_from_IMU_MPU_6050/blob/main/mpu_6050_driver/scripts/imu_node.py


void setup() {

  SabertoothTXPinSerial.begin(9600);
  //Serial.begin(9600);

  nh.initNode();                            //init ROS node
  nh.getHardware()->setBaud(115200);         //set baud for ROS serial communication
  nh.subscribe(cmd_vel);                    //suscribe to ROS topic for velocity commands
  nh.subscribe(joy_msg);
  nh.advertise(odom_pub);
  nh.advertise(imu_pub);
  broadcaster.init(nh);                     //prepare to publish speed in ROS topic

  STRight.motor(1, 0);
  STRight.motor(2, 0);
  STLeft.motor(1, 0);
  STLeft.motor(2, 0);

  EncL1.write(0);
  EncL2.write(0);
  EncR1.write(0);
  EncR2.write(0);

  while (!nh.connected())
  {
    nh.spinOnce();
  }
  nh.loginfo("RUDRA CONNECTED");
  delay(1);

  //Initialize IMU

  Wire.begin();

  accelgyro.initialize();
  accelgyro.setI2CBypassEnabled(true);

  if (accelgyro.testConnection()) {

    nh.loginfo("MPU6050 connection successful");
    delay(1);

    nh.loginfo("MPU6050 initiating calibration");
    delay(1);

    for (int i = 0; i < 200; i++) {

      accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

      acc_bias_x += ax;
      acc_bias_y += ay;
      acc_bias_z += az;

      gyro_bias_x += gx;
      gyro_bias_y += gy;
      gyro_bias_z += gz;

    }

    acc_bias_x = acc_bias_x / 200.0;
    acc_bias_y = acc_bias_y / 200.0;
    acc_bias_z = acc_bias_z / 200.0;

    gyro_bias_x = gyro_bias_x / 200.0;
    gyro_bias_y = gyro_bias_y / 200.0;
    gyro_bias_z = gyro_bias_z / 200.0;

    nh.loginfo("MPU6050 calibration successful");
    delay(1);

  } else
  {
    nh.loginfo("MPU6050 connection FAILED !!");
    delay(1);
  }

}

void loop() {

  nh.spinOnce();


  if ((millis() - lastMilli) >= LOOPTIME)

  { // enter timed loop

    lastMilli = millis();

    pos_left1 = EncL1.read();        //read the accumulated encoder positions
    pos_left2 = EncL2.read();
    pos_right1 = EncR1.read();
    pos_right2 = EncR2.read();

    if (abs(pos_left1) < 5) {

      speed_act_left1 = 0;
    }
    else {

      speed_act_left1 = ((pos_left1 / encoder_cpr) * 2 * PI) * (1000 / LOOPTIME) * radius; // calculate speed of left wheel
    }

    if (abs(pos_left2) < 5) {                                                  //Avoid taking into account small disturbances
      speed_act_left2 = 0;
    }
    else {
      speed_act_left2 = ((pos_left2 / encoder_cpr) * 2 * PI) * (1000 / LOOPTIME) * radius; // calculate speed of left wheel
    }

    if (abs(pos_right1) < 5) {                                                 //Avoid taking into account small disturbances
      speed_act_right1 = 0;
    }
    else {
      speed_act_right1 = ((pos_right1 / encoder_cpr) * 2 * PI) * (1000 / LOOPTIME) * radius; // calculate speed of right wheel
    }

    if (abs(pos_right2) < 5) {                                                 //Avoid taking into account small disturbances
      speed_act_right2 = 0;
    }
    else {
      speed_act_right2 = ((pos_right2 / encoder_cpr) * 2 * PI) * (1000 / LOOPTIME) * radius; // calculate speed of right wheel
    }

    EncL1.write(0); //reset encoder count to zero
    EncL2.write(0);
    EncR1.write(0);
    EncR2.write(0);

    if (manual_mode) {

      if ((throttle <= 5) && (throttle >= -5)) {

        throttle = 0;

      }


      if ((steering <= 5) && (steering >= -5)) {

        steering = 0;

      }

      leftSpeed = throttle - steering;
      rightSpeed = throttle + steering;

      if ((leftSpeed <= 2) && (leftSpeed >= -2)) {

        stop_base();

      }

      else if ((rightSpeed <= 2) && (rightSpeed >= -2)) {

        stop_base();

      } else {


        STRight.motor(1, rightSpeed);
        STRight.motor(2, rightSpeed);
        STLeft.motor(1, leftSpeed);
        STLeft.motor(2, leftSpeed);
      }



    } else {


      if (pid_flag) {

        PWM_leftMotor1 = compute_PID(speed_req_left, speed_act_left1);
        PWM_leftMotor2 = compute_PID(speed_req_left, speed_act_left2);
        PWM_rightMotor1 = compute_PID(speed_req_right, speed_act_right1);
        PWM_rightMotor2 = compute_PID(speed_req_right, speed_act_right2);

      } else {

        PWM_leftMotor1 = speed_req_left * 127 / 1.5;
        PWM_leftMotor2 = speed_req_left * 127 / 1.5;
        PWM_rightMotor1 = speed_req_right * 127 / 1.5;
        PWM_rightMotor2 = speed_req_right * 127 / 1.5;
      }


      if (speed_req_right == 0) {

        stop_base();

      } else if (noCommLoops >= noCommLoopMax) {
        stop_base();

      } else {
        STRight.motor(1, PWM_rightMotor1);
        STRight.motor(2, PWM_rightMotor2);

      }

      if (speed_req_left == 0) {
        stop_base();

      } else if (noCommLoops >= noCommLoopMax) {
        stop_base();

      } else {

        STLeft.motor(1, PWM_leftMotor1);
        STLeft.motor(2, PWM_leftMotor2);

      }

      noCommLoops++;
      if (noCommLoops == 65535) {
        noCommLoops = noCommLoopMax;
      }

    }

    //IMU Readings
    //NOTE: the urdf file contains trasformation for "imu_link". So does for "laser"

    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    ax -= acc_bias_x;
    ay -= acc_bias_y;
    az -= acc_bias_z;

    ax = ax * (double) ACCEL_SCALE * G_TO_ACCEL;
    ay = ay * (double) ACCEL_SCALE * G_TO_ACCEL;
    az = az * (double) ACCEL_SCALE * G_TO_ACCEL;

    gx = gx - gyro_bias_x;
    gy = gy - gyro_bias_y;
    gz = gz - gyro_bias_z;


    gx = gx * (double) GYRO_SCALE * DEG_TO_RAD;
    gy = gy * (double) GYRO_SCALE * DEG_TO_RAD;
    gz = gz * (double) GYRO_SCALE * DEG_TO_RAD;

    //nh.loginfo(gz);

    imu.header.stamp = nh.now();
    imu.header.frame_id = imu_link;

    imu.angular_velocity.x = gx;
    imu.angular_velocity.y = gy;
    imu.angular_velocity.z = gz;

    imu.linear_acceleration.x = ax;
    imu.linear_acceleration.y = ay;
    imu.linear_acceleration.z = az;

    imu_pub.publish(&imu);     //very important to publish the data

    //NOTE: Here IMU is published as /imu/data_raw. This topic is then subscribed by <node name="imu_filter_node_for_orientation" pkg="imu_complementary_filter" type="complementary_filter_node" >
    //      the complementary filter node is launched with rudra_bringup.launch. the complementary filter node then converts /imu/data_raw to desired /imu/data topic


    // *** Publishing Odometry ***

    speed_act_left_avg = (speed_act_left1 + speed_act_left2) / 2.0;
    speed_act_right_avg = (speed_act_right1 + speed_act_right2) / 2.0;


    //dt = LOOPTIME / 1000;

    dxy = (speed_act_left_avg + speed_act_right_avg) * dt / 2.0;

    dth_odom = ((speed_act_right_avg - speed_act_left_avg) * dt) / wheelbase;

    if (fuse_imu) {

      dth = alpha * dth_odom + (1 - alpha) * dt * gz;         // this is complementary filter

    } else {

      dth = dth_odom;

    }

    dx = cos(dth) * dxy;
    dy = sin(dth) * dxy;

    x_pos += (cos(theta) * dx - sin(theta) * dy);
    y_pos += (sin(theta) * dx + cos(theta) * dy);
    theta += dth;

    if (theta >= two_pi) theta -= two_pi;
    if (theta <= -two_pi) theta += two_pi;

    // *** broadcast odom--> base_link transform ***

    geometry_msgs::TransformStamped t;

    t.header.frame_id = odom;
    t.child_frame_id = base_link;

    t.transform.translation.x = x_pos;
    t.transform.translation.y = y_pos;
    t.transform.translation.z = 0.0;

    t.transform.rotation = tf::createQuaternionFromYaw(theta);
    t.header.stamp = nh.now();

    broadcaster.sendTransform(t);

    // *** broadcast /tf ***

    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = nh.now();
    odom_msg.header.frame_id = odom;
    odom_msg.pose.pose.position.x = x_pos;
    odom_msg.pose.pose.position.y = y_pos;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = tf::createQuaternionFromYaw(theta);

    odom_msg.child_frame_id = base_link;
    odom_msg.twist.twist.linear.x = (speed_act_left_avg + speed_act_right_avg) / 2.0;           // forward linear velovity
    odom_msg.twist.twist.linear.y = 0.0;                                                        // robot does not move sideways
    odom_msg.twist.twist.angular.z = (speed_act_right_avg - speed_act_left_avg) / wheelbase;    // anglular velocity

    odom_pub.publish(&odom_msg);   //publish, make sure this sketch is accompanied by its own ros.h else the rosmaster will throw error related to buffer size...


  }//End of timed loop


}//End of main loop


/**************************** PID Implementation *************************************/


int compute_PID(float setpoint, float measured_value)
{
  double error;
  double motor_pwm;

  //A good implementation would be to constrain setpoint between min and max to prevent pid from having too much error.
  //here I have not contrained the setpoint. My Kp Ki Kd is able to handle this so I did not bother implementing it. It might upset my PID tuning now.

  error = setpoint - measured_value;

  integral_ += error;

  derivative_ = error - prev_error_;

  if (setpoint == 0 && error == 0)
  {
    integral_ = 0;
  }

  motor_pwm = (kp_ * error) + (ki_ * integral_) + (kd_ * derivative_); //beauty of PID is that the output can be used as PWM value for motors, do not have to worry about actual motor curves
  prev_error_ = error;

  return constrain(motor_pwm, min_val_, max_val_);
}

/***********************************************************************************/


void stop_base() {

  STRight.motor(1, 0);
  STRight.motor(2, 0);
  STLeft.motor(1, 0);
  STLeft.motor(2, 0);

  integral_ = 0; //avoiding integral wind-up for PID control.. every time the robot stops, the integral error is set to zero otherwise the PID will accelerate too quicky because of accumulated error from previous operation
}
