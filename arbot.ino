#ifdef USE_ARBOT

/* 
 * The ARbot consists of an iRobot Create with a connected
 * arduino_irobot_bridge, which is connected to 
 * an Arduino variant, the MinSegMega (from www.minseg.com).
 */

#include <LiquidCrystal.h>
#include <Roomba.h>
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
//#include "InfraredSeeker.h"

// Universal constants
double DEGREES_TO_RADIANS = M_PI / 180.0;
double RADIANS_TO_DEGREES = 180.0 / M_PI;

// Devices
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);
Roomba roomba(&Serial3); // Using the MinSegMega we connect to the robot through TX3/RX3
MPU6050 accelgyro;

// Robot parameters.
double HALF_BASELINE = 264 / 2; // mm

// Bias calculation time in microseconds
unsigned long BIAS_TIME = 10000000;

double GYRO_DIVISOR = 131 * 1000000;

// Current and last pose, represented in metres and radians.
double x = 0, y = 0, theta = 0;
double lastX = 0, lastY = 0, lastTheta = 0;

// For reading sensor data from the robot.
uint8_t roombaBuffer[52];

// For estimating bias of the gyro.
bool biasMeasuringMode = true;
double gyroBias = 0;
unsigned long biasCount = 0;
unsigned long startTime; // In microseconds

// To keep track of time (in microseconds)
unsigned long time, lastTime, lastToggleTime;

// For controlling the iRobot Create's LEDs
long ledBits = 0;

void displayPose() {
  lcd.clear();
  lcd.print(x);
  lcd.setCursor(8, 0);
  lcd.print(y);
  lcd.setCursor(0, 1);
  lcd.print(RADIANS_TO_DEGREES * theta);
}

void updateBias(int16_t gyroValue) {
  gyroBias += gyroValue;
  biasCount++;
  
  if (time - startTime > BIAS_TIME) {
    gyroBias /= biasCount;
    biasMeasuringMode = false;
    displayPose();
    
    // Also start connection with robot.
    roomba.start();
    roomba.fullMode();
    roomba.leds(ledBits, 0, 0);    
  }
}

void updateTheta(int16_t gyroValue) {
  // Update theta if the deviation from the bias value is above a noise threshold.
  if (abs(gyroValue - gyroBias) > 1.0 * gyroBias) {
    // Integrate the angular speed.
    theta += DEGREES_TO_RADIANS * (time - lastTime) * (gyroValue - gyroBias) / GYRO_DIVISOR;    
  }
}

void updateXY() {
  bool ret = roomba.getSensors(Roomba::SensorDistance, roombaBuffer, 2);
  double distance = roombaBuffer[1] + 256 * roombaBuffer[0];
  
  // The distance reported by the robot is in mm.
  distance *= 1.0/1000.0;
  
  // We use a partially updated theta value to better reflect movement over the
  // whole of the sampling period.
  double midTheta = (theta + lastTheta)/2.0f;
  x += distance * cos(midTheta);
  y += distance * sin(midTheta);
}

/*
void handleIRSeeker() {
  InfraredResult result = InfraredSeeker::ReadDC();
  irseeker_msg.strength = result.Strength;
  irseeker_msg.angle = (float) DEGREES_TO_RADIANS * InfraredSeeker::DirectionAngle(result.Direction);
  irseeker_pub.publish(&irseeker_msg);
}
*/

void arbotSetup(){
  lcd.begin(16, 2);
  lcd.print("RAB 1.1");
  lcd.setCursor(0, 1);
  lcd.print("Do not move bot!");
  
  // Attempting to address perceived serial issue by clearing outgoing serial buffer (with flush)
  // and incoming buffer.
  Serial3.flush();
  while (Serial3.available() > 0)
    // Read (and discard) the incoming byte:
    Serial3.read();
  
  // Initialize IMU
  Wire.begin(); // needed for I2C
  accelgyro.initialize();  
  startTime = micros();
  lastToggleTime = startTime;
  
  // Note that we do not initialize the connection with the robot yet.  This is
  // done after the bias measuring period is finished.
  
//  InfraredSeeker::Initialize();
}

void arbotLoop(){
  lastTime = time;
  time = micros();
  
  int16_t gyroValue = accelgyro.getRotationZ();

  if (biasMeasuringMode)
    updateBias(gyroValue);
  else {
    lastX = x;
    lastY = y;
    lastTheta = theta;
    
    updateTheta(gyroValue);
    updateXY();
    
    if (x != lastX || y != lastY || theta != lastTheta)
      displayPose();
  }

  // Toggle the play LED every 250 ms
  if (time - lastToggleTime > 0.25e6) {
    ledBits = ledBits ^ ROOMBA_MASK_LED_PLAY;    
    roomba.leds(ledBits, 0, 0);
    lastToggleTime = time;
  }
  
//  handleIRSeeker();
}

double arbotGetX() {
  return x;
}

double arbotGetY() {
  return y;
}

double arbotGetTheta() {
  return theta;
}

void arbotSetAdvanceLed(long value) {
  if (value == 0)
    ledBits = ledBits & ~ROOMBA_MASK_LED_ADVANCE;    
  else if (value == 1)
    ledBits = ledBits | ROOMBA_MASK_LED_ADVANCE;  
  roomba.leds(ledBits, 0, 0);
}

void arbotSetVelocity(double v, long w) {
  // Apply differential-drive kinematic model. We do not divide by the wheel
  // radius because the 'driveDirect' command below
  // actually expects wheel roll speeds, not rates of rotation.
  int16_t leftVelocity = (v - HALF_BASELINE * w); // / WHEEL_RADIUS;
  int16_t rightVelocity = (v + HALF_BASELINE * w); // / WHEEL_RADIUS;

  /*
  lcd.clear();
  lcd.print(leftVelocity);
  lcd.print(", ");
  lcd.print(rightVelocity);
  */

  roomba.driveDirect(leftVelocity, rightVelocity);
}

#endif
