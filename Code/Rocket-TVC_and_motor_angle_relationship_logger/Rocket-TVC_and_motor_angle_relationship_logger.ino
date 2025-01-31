/*

  Spaghetti code, so user needs to manually change current controlled motor nade end value.

*/

#include "MPU9250.h"
#include <Servo.h>

Servo bottomServo;
Servo topServo;

float yaw;
float pitch;
float roll;

String data;
int dataInt;

MPU9250 mpu;

unsigned long currentTime;
unsigned long prevTime = 0;
unsigned long prevTimeSec;

int times;
int servo = 2350; // ---------------------------------------------------------------------------------------------------------------------------------------------------------Change

float average;

int accuracy = 50;  // Defines how many motor steps are moved in each iteration, lower value is more accurate.
                    // Default value 10

void setup() {

  Serial.begin(115200);

  while (!Serial);

  delay(1000);


  Wire1.setSDA(18);
  Wire1.setSCL(19);
  Wire1.begin();

  if (!mpu.setup(0x68, Wire1)) {
    while (1) {
      Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
      delay(5000);
    }
  }

  bottomServo.attach(6, 500, 2500);  // ~1700 ~2150 ~2350
  topServo.attach(7, 500, 2500);     // ~1900 ~2250 ~2500
  bottomServo.writeMicroseconds(2150);
  topServo.writeMicroseconds(2250);

  // calibrate anytime you want to
  Serial.println("Accel Gyro calibration will start in 5sec.");
  Serial.println("Please leave the device still on the flat plane.");
  mpu.verbose(true);
  delay(5000);
  mpu.calibrateAccelGyro();
  //print_calibration();
  mpu.verbose(false);
  mpu.setFilterIterations(10);
  mpu.setMagneticDeclination(12.47);

  bottomServo.writeMicroseconds(2400);// ----------------------------------------------------------------------------------------------------------------------Change
  //topServo.writeMicroseconds(2500);


  Serial.println("Waiting for input to start calibration...");
  // Wait for Serial input until proceeding
  while (Serial.available() == 0) {
    mpu.update();
    print_roll_pitch_yaw();
  }

  Serial.print("Angle");
  Serial.print(",");
  Serial.println("Steps");

  prevTime = millis();
  prevTimeSec = prevTime;
  mpu.update();
  print_roll_pitch_yaw();
}

void loop() {

  if (mpu.update()) {
        static uint32_t prev_ms = millis();
        if (millis() > prev_ms + 25) {
            print_roll_pitch_yaw();
            prev_ms = millis();
        }
  }

  if ((millis() - prevTime) <= 500 && servo >= 1700) {// ----------------------------------------------------------------------------------------------------------------------Change
    //Serial.println("bbbbbbbbbbbbbbbbbbbbbbbb");
    bottomServo.writeMicroseconds(servo);// ----------------------------------------------------------------------------------------------------------------------Change
    //topServo.writeMicroseconds(servo);

    servo -= accuracy;

    times = 0;
    average = 0;

    while ((millis() - prevTimeSec) <= 1000) {
      if (mpu.update()) {
        static uint32_t prev_ms = millis();
        if (millis() > prev_ms + 25) {
          print_roll_pitch_yaw();
          prev_ms = millis();
          times += 1;
          if (times > 20) {
            average += pitch;// ----------------------------------------------------------------------------------------------------------------------Change (roll for top/pitch for bottom)
          }
        }
      }
    }
    
    prevTimeSec = millis();
    prevTime = millis();
  }

  average = average/(times-20);

  Serial.print(average);
  Serial.print(",");
  Serial.println(servo + accuracy);

  while ( (servo + accuracy) == 1700) {// ----------------------------------------------------------------------------------------------------------------------Change
    servo = servo;
  }

}

void print_roll_pitch_yaw() {

  yaw = mpu.getYaw();
  pitch = mpu.getPitch();
  roll = mpu.getRoll();
}