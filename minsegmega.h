// MinSegMega specific setup and loop functions
void msmSetup();
void msmLoop();

// Read x, y, or theta coordinate of current odometry estimate.  The returned
// values are in mm for x and y and in radians * 1000 for theta.
long msmGetX();
long msmGetY();
long msmGetTheta();

// Set the speeds of the two motors.  For each motor we have a direction and a
// PWM parameter.
void msmSetMotors(int leftDir, int leftPWM, int rightDir, int rightPWM);

//void msmDrive(int speed, int heading);
