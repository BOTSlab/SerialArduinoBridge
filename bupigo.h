// Using the usual Servo library disables a bunch of PWM pins, including those
// we need to control the Lego NXT motors.  So trying this alternate instead...
#include <ServoTimer2.h>

// BuPiGo specific setup and loop functions
void bupigoSetup();
void bupigoLoop();

// Read x, y, or theta coordinate of current odometry estimate.  The returned
// values are in mm for x and y and in radians * 1000 for theta.
long bupigoGetX();
long bupigoGetY();
long bupigoGetTheta();

// Set the direction of movement (0 = stopped, 1 = forward, 2 = reverse,
// 3 = left, 4 = right).
void bupigoSetMotors(long dir);

// Set the speeds of the two motors
void bupigoSetSpeeds(long leftSpeed, long rightSpeed);

// Set the forward and angular speeds in units of mm / sec and radians / 1000 seconds
void bupigoSetVelocity(long forwardSpeed, long angularSpeed);

// Set the servo's desired angle
void bupigoSetServoAngle(long angle);
