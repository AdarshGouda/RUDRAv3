
/*
   Created Date: 7-May-2021

   Rev: AD

   Rev Date: 07 May 2021

   Intent: Use PS2 and figure out what is needed for joy_msg
           (Joy commands to turn the bot)
*/

#include <ros.h>
#include <ros/time.h>
#include <rudra/joy_msg.h>
#include <PS2X_lib.h>

PS2X ps2x;


ros::NodeHandle nh;
rudra::joy_msg msg;
ros::Publisher pub("joy_msg", &msg);

char frameid[] = "/joy_msg";

bool manual_mode = false;
int error = 0; 
byte vibrate = 0;

void setup() {
  nh.initNode();
  nh.advertise(pub);

  error = ps2x.config_gamepad(9, 7, 6, 8, true, true);
}


void loop() {

  ps2x.read_gamepad(false, vibrate);

  if (ps2x.Button(PSB_SELECT)) {

    manual_mode = !manual_mode;
    msg.SELECT = manual_mode;
  }


  msg.head = nh.now();

  msg.Ry = ps2x.Analog(PSS_RY);
  msg.Rx = ps2x.Analog(PSS_RX);
  msg.Ly = ps2x.Analog(PSS_LY);
  msg.Lx = ps2x.Analog(PSS_LX);


  pub.publish(&msg);
  nh.spinOnce();
  delay(100);
}
