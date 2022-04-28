
/* 
 *  ===============================================================================
 *  Create Date: 17-Apr-2021
 *  
 *  Revison: AA
 *  
 *  Revision Date: -
 *  
 *  Intent: 1) To use MPU6050 library
 *          2) Convert the raw IMU data to real world values. 
 *  
 *  
 *  Observations: 1) Requires an Interrupt pin but would work without the INT as well.
 *                2) A LED blink would be a good indicator of MPU transmitting Data.  
 *  
 *  
 *  ================================================================================
   Connection to the Teensy
   MPU 6050     Teensy
   Vcc          3.3V
   GND          GND
   SCL          pin A5
   SDA          pin A4
   XDA          NC  (no connection)
   XCL          NC  (no connection)
   ADO          NC
   INT          Pin 2
   =================================================
*/

#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"

#define ACCEL_SCALE 1 / 16384    // LSB/g  //These Define factors are for MPU6050 datasheet
#define GYRO_SCALE 1 / 131       // LSB/(deg/s)
#define MAG_SCALE 0.3            // uT/LSB
#define G_TO_ACCEL 9.81

MPU6050 accelgyro(0x69);      //Change the address if I2C is on 0x68, which is typically the case with most MPU6050 sensors.
unsigned long time;
int16_t ax, ay, az;
int16_t gx, gy, gz;
float heading;
bool blinkState = false;

int LED_PIN = 13;

void setup() {
  Wire.begin();               // join I2C bus (I2Cdev library doesn't do this automatically)
  Serial.begin(115200);       // We do not need this when implementing in ROS. No need create and new serial.
  while (!Serial);

  Serial.println("Initializing I2C devices...");
  delay(500);
  accelgyro.initialize();
  accelgyro.setI2CBypassEnabled(true);

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  delay(500);
  accelgyro.setFullScaleAccelRange(0);
  pinMode(LED_PIN, OUTPUT);   // configure Arduino LED
}


void loop() {

  time = millis();  //prints time since program started



  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);     // read raw accel/gyro measurements from device

  // these methods (and a few others) are also available
  // accelgyro.getAcceleration(&ax, &ay, &az);
  // accelgyro.getRotation(&gx, &gy, &gz);

  ax = ax * (double) ACCEL_SCALE * G_TO_ACCEL;
  ay = ay * (double) ACCEL_SCALE * G_TO_ACCEL;
  az = az * (double) ACCEL_SCALE * G_TO_ACCEL;

  gx = gx * (double) GYRO_SCALE * DEG_TO_RAD;
  gy = gy * (double) GYRO_SCALE * DEG_TO_RAD;
  gz = gz * (double) GYRO_SCALE * DEG_TO_RAD;



  blinkState = !blinkState;                               // blink LED to indicate activity
  digitalWrite(LED_PIN, blinkState);

  //  delay(100); // run at ~100 Hz

   plotAccel();
   //plotGyro();
  
}


void plotAccel() {
  Serial.print(ax); Serial.print(",");
  Serial.print(ay); Serial.print(",");
  Serial.println(az);
  delay(100); // run at ~100 Hz
}

void plotGyro() {
  Serial.print(gx); Serial.print(",");
  Serial.print(gy); Serial.print(",");
  Serial.println(gz);
  delay(100); //
}
