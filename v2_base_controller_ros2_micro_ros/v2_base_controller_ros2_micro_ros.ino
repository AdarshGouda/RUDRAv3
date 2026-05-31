/* ROS 2 / micro-ROS base controller for RUDRA.

   This is the ROS 2 replacement for the rosserial sketch in
   v2_base_controler_AE. It keeps the Sabertooth, encoder, PID, odometry,
   and MPU6050 logic, but talks to ROS 2 through a micro-ROS Agent.

   Topics:
     Subscribes:
       /cmd_vel             geometry_msgs/msg/Twist
       /manual_mode         std_msgs/msg/Bool
       /manual_cmd_vel      geometry_msgs/msg/Twist

     Publishes:
       /odom                nav_msgs/msg/Odometry
       /imu/data_raw        sensor_msgs/msg/Imu

   Manual/joystick placeholder:
     If the joystick lives on an Arduino Uno, let the Uno or a ROS 2 node
     publish /manual_mode and /manual_cmd_vel. manual_cmd_vel uses
     linear.x as throttle and angular.z as steering, both normalized -1..1.
*/

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rosidl_runtime_c/string_functions.h>

#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/bool.h>

#include <I2Cdev.h>
#include <MPU6050.h>

#include <Sabertooth.h>

#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>

#define ACCEL_SCALE (1.0 / 16384.0)
#define GYRO_SCALE (1.0 / 131.0)
#define G_TO_ACCEL 9.81

#define LOOPTIME 100
#define MICRO_ROS_BAUD 115200

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { error_loop(); } }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; (void) temp_rc; }

#ifdef __locale_ctype_ptr
#undef __locale_ctype_ptr
#endif

extern "C" const char _ctype_[];

extern "C" const char * __locale_ctype_ptr(void) {
  return _ctype_;
}

MPU6050 accelgyro(0x69);
int16_t ax, ay, az;
int16_t gx, gy, gz;

Sabertooth STRight(128), STLeft(129);

Encoder EncR1(8, 7);
Encoder EncL1(2, 3);
Encoder EncL2(4, 5);
Encoder EncR2(32, 31);

unsigned long last_milli = 0;
const byte no_comm_loop_max = 10;
unsigned int no_comm_loops = 0;

const double radius = 0.0675;
const double wheelbase = 0.29;
const double encoder_cpr = 1683;

double speed_req = 0.0;
double angular_speed_req = 0.0;
double speed_req_left = 0.0;
double speed_req_right = 0.0;

double speed_act_left1 = 0.0;
double speed_act_left2 = 0.0;
double speed_act_right1 = 0.0;
double speed_act_right2 = 0.0;
double speed_act_left_avg = 0.0;
double speed_act_right_avg = 0.0;

int PWM_leftMotor1 = 0;
int PWM_leftMotor2 = 0;
int PWM_rightMotor1 = 0;
int PWM_rightMotor2 = 0;

volatile float pos_left1 = 0.0;
volatile float pos_left2 = 0.0;
volatile float pos_right1 = 0.0;
volatile float pos_right2 = 0.0;

bool pid_flag = true;
float integral_ = 0.0;
float derivative_ = 0.0;
float prev_error_ = 0.0;
int min_val_ = -127;
int max_val_ = 127;

float kp_ = 20.0;
float ki_ = 5.0;
float kd_ = 0.05;

double x_pos = 0.0;
double y_pos = 0.0;
double theta = 0.0;
const double two_pi = 6.28319;

bool fuse_imu = true;
double alpha = 0.5;
double dt = LOOPTIME / 1000.0;
double dxy = 0.0;
double dth_odom = 0.0;
double dth = 0.0;

double gyro_bias_x = 0.0;
double gyro_bias_y = 0.0;
double gyro_bias_z = 0.0;
double acc_bias_x = 0.0;
double acc_bias_y = 0.0;
double acc_bias_z = 0.0;

bool manual_mode = false;
int manual_throttle_pwm = 0;
int manual_steering_pwm = 0;

rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;

rcl_subscription_t cmd_vel_sub;
rcl_subscription_t manual_mode_sub;
rcl_subscription_t manual_cmd_vel_sub;
rcl_publisher_t odom_pub;
rcl_publisher_t imu_pub;

geometry_msgs__msg__Twist cmd_vel_msg;
geometry_msgs__msg__Twist manual_cmd_vel_msg;
std_msgs__msg__Bool manual_mode_msg;
nav_msgs__msg__Odometry odom_msg;
sensor_msgs__msg__Imu imu_msg;

void error_loop() {
  stop_base();
  while (1) {
    delay(100);
  }
}

void stamp_now(builtin_interfaces__msg__Time * stamp) {
  uint64_t now_ms = millis();
  stamp->sec = now_ms / 1000;
  stamp->nanosec = (now_ms % 1000) * 1000000;
}

void set_quaternion_from_yaw(geometry_msgs__msg__Quaternion * q, double yaw) {
  q->x = 0.0;
  q->y = 0.0;
  q->z = sin(yaw * 0.5);
  q->w = cos(yaw * 0.5);
}

int normalized_to_pwm(double value) {
  value = constrain(value, -1.0, 1.0);
  return (int)(value * 127.0);
}

void cmd_vel_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;

  no_comm_loops = 0;
  speed_req = msg->linear.x;
  angular_speed_req = msg->angular.z;
  speed_req_left = speed_req - angular_speed_req * (wheelbase / 2.0);
  speed_req_right = speed_req + angular_speed_req * (wheelbase / 2.0);
}

void manual_mode_callback(const void * msgin) {
  const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;
  manual_mode = msg->data;
  no_comm_loops = 0;
}

void manual_cmd_vel_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;

  manual_throttle_pwm = normalized_to_pwm(msg->linear.x);
  manual_steering_pwm = normalized_to_pwm(msg->angular.z);
  no_comm_loops = 0;
}

void setup_ros_entities() {
  Serial.begin(MICRO_ROS_BAUD);
  set_microros_transports();
  delay(2000);

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "rudra_base_controller", "", &support));

  RCCHECK(rclc_subscription_init_default(
    &cmd_vel_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));

  RCCHECK(rclc_subscription_init_default(
    &manual_mode_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "manual_mode"));

  RCCHECK(rclc_subscription_init_default(
    &manual_cmd_vel_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "manual_cmd_vel"));

  RCCHECK(rclc_publisher_init_default(
    &odom_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
    "odom"));

  RCCHECK(rclc_publisher_init_default(
    &imu_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "imu/data_raw"));

  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_subscription(
    &executor, &cmd_vel_sub, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(
    &executor, &manual_mode_sub, &manual_mode_msg, &manual_mode_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(
    &executor, &manual_cmd_vel_sub, &manual_cmd_vel_msg, &manual_cmd_vel_callback, ON_NEW_DATA));

  nav_msgs__msg__Odometry__init(&odom_msg);
  sensor_msgs__msg__Imu__init(&imu_msg);

  if (!rosidl_runtime_c__String__assign(&odom_msg.header.frame_id, "odom")) {
    error_loop();
  }
  if (!rosidl_runtime_c__String__assign(&odom_msg.child_frame_id, "base_link")) {
    error_loop();
  }
  if (!rosidl_runtime_c__String__assign(&imu_msg.header.frame_id, "imu_link")) {
    error_loop();
  }
}

void setup() {
  SabertoothTXPinSerial.begin(9600);

  STRight.motor(1, 0);
  STRight.motor(2, 0);
  STLeft.motor(1, 0);
  STLeft.motor(2, 0);

  EncL1.write(0);
  EncL2.write(0);
  EncR1.write(0);
  EncR2.write(0);

  Wire.begin();
  accelgyro.initialize();
  accelgyro.setI2CBypassEnabled(true);

  if (accelgyro.testConnection()) {
    for (int i = 0; i < 200; i++) {
      accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
      acc_bias_x += ax;
      acc_bias_y += ay;
      acc_bias_z += az;
      gyro_bias_x += gx;
      gyro_bias_y += gy;
      gyro_bias_z += gz;
      delay(2);
    }

    acc_bias_x /= 200.0;
    acc_bias_y /= 200.0;
    acc_bias_z /= 200.0;
    gyro_bias_x /= 200.0;
    gyro_bias_y /= 200.0;
    gyro_bias_z /= 200.0;
  }

  setup_ros_entities();
}

void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5)));

  if ((millis() - last_milli) < LOOPTIME) {
    return;
  }

  last_milli = millis();

  read_encoder_speeds();
  update_motors();
  read_and_publish_imu();
  update_and_publish_odom();
}

void read_encoder_speeds() {
  pos_left1 = EncL1.read();
  pos_left2 = EncL2.read();
  pos_right1 = EncR1.read();
  pos_right2 = EncR2.read();

  speed_act_left1 = encoder_speed(pos_left1);
  speed_act_left2 = encoder_speed(pos_left2);
  speed_act_right1 = encoder_speed(pos_right1);
  speed_act_right2 = encoder_speed(pos_right2);

  EncL1.write(0);
  EncL2.write(0);
  EncR1.write(0);
  EncR2.write(0);
}

double encoder_speed(float counts) {
  if (abs(counts) < 5) {
    return 0.0;
  }
  return ((counts / encoder_cpr) * 2.0 * PI) * (1000.0 / LOOPTIME) * radius;
}

void update_motors() {
  if (manual_mode) {
    int left_speed = manual_throttle_pwm - manual_steering_pwm;
    int right_speed = manual_throttle_pwm + manual_steering_pwm;

    left_speed = constrain(left_speed, -127, 127);
    right_speed = constrain(right_speed, -127, 127);

    if (abs(left_speed) <= 2 || abs(right_speed) <= 2 || no_comm_loops >= no_comm_loop_max) {
      stop_base();
    } else {
      STRight.motor(1, right_speed);
      STRight.motor(2, right_speed);
      STLeft.motor(1, left_speed);
      STLeft.motor(2, left_speed);
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

    if ((speed_req_left == 0.0 && speed_req_right == 0.0) || no_comm_loops >= no_comm_loop_max) {
      stop_base();
    } else {
      STRight.motor(1, PWM_rightMotor1);
      STRight.motor(2, PWM_rightMotor2);
      STLeft.motor(1, PWM_leftMotor1);
      STLeft.motor(2, PWM_leftMotor2);
    }
  }

  no_comm_loops++;
  if (no_comm_loops == 65535) {
    no_comm_loops = no_comm_loop_max;
  }
}

void read_and_publish_imu() {
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  double acc_x = (ax - acc_bias_x) * ACCEL_SCALE * G_TO_ACCEL;
  double acc_y = (ay - acc_bias_y) * ACCEL_SCALE * G_TO_ACCEL;
  double acc_z = (az - acc_bias_z) * ACCEL_SCALE * G_TO_ACCEL;

  double gyro_x = (gx - gyro_bias_x) * GYRO_SCALE * DEG_TO_RAD;
  double gyro_y = (gy - gyro_bias_y) * GYRO_SCALE * DEG_TO_RAD;
  double gyro_z = (gz - gyro_bias_z) * GYRO_SCALE * DEG_TO_RAD;

  stamp_now(&imu_msg.header.stamp);
  imu_msg.angular_velocity.x = gyro_x;
  imu_msg.angular_velocity.y = gyro_y;
  imu_msg.angular_velocity.z = gyro_z;
  imu_msg.linear_acceleration.x = acc_x;
  imu_msg.linear_acceleration.y = acc_y;
  imu_msg.linear_acceleration.z = acc_z;

  RCSOFTCHECK(rcl_publish(&imu_pub, &imu_msg, NULL));
}

void update_and_publish_odom() {
  speed_act_left_avg = (speed_act_left1 + speed_act_left2) / 2.0;
  speed_act_right_avg = (speed_act_right1 + speed_act_right2) / 2.0;

  dxy = (speed_act_left_avg + speed_act_right_avg) * dt / 2.0;
  dth_odom = ((speed_act_right_avg - speed_act_left_avg) * dt) / wheelbase;

  if (fuse_imu) {
    double gyro_z = imu_msg.angular_velocity.z;
    dth = alpha * dth_odom + (1.0 - alpha) * dt * gyro_z;
  } else {
    dth = dth_odom;
  }

  double dx = cos(dth) * dxy;
  double dy = sin(dth) * dxy;

  x_pos += (cos(theta) * dx - sin(theta) * dy);
  y_pos += (sin(theta) * dx + cos(theta) * dy);
  theta += dth;

  if (theta >= two_pi) theta -= two_pi;
  if (theta <= -two_pi) theta += two_pi;

  stamp_now(&odom_msg.header.stamp);
  odom_msg.pose.pose.position.x = x_pos;
  odom_msg.pose.pose.position.y = y_pos;
  odom_msg.pose.pose.position.z = 0.0;
  set_quaternion_from_yaw(&odom_msg.pose.pose.orientation, theta);

  odom_msg.twist.twist.linear.x = (speed_act_left_avg + speed_act_right_avg) / 2.0;
  odom_msg.twist.twist.linear.y = 0.0;
  odom_msg.twist.twist.angular.z = (speed_act_right_avg - speed_act_left_avg) / wheelbase;

  RCSOFTCHECK(rcl_publish(&odom_pub, &odom_msg, NULL));
}

int compute_PID(float setpoint, float measured_value) {
  double error = setpoint - measured_value;
  integral_ += error;
  derivative_ = error - prev_error_;

  if (setpoint == 0.0 && error == 0.0) {
    integral_ = 0.0;
  }

  double motor_pwm = (kp_ * error) + (ki_ * integral_) + (kd_ * derivative_);
  prev_error_ = error;

  return constrain(motor_pwm, min_val_, max_val_);
}

void stop_base() {
  STRight.motor(1, 0);
  STRight.motor(2, 0);
  STLeft.motor(1, 0);
  STLeft.motor(2, 0);
  integral_ = 0.0;
}
