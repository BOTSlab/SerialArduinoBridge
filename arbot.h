
// ARbot specific setup and loop functions
void arbotSetup();
void arbotLoop();

// Read x, y, or theta coordinate of current odometry estimate.  The returned values are in mm
// for x and y and in radians * 1000 for theta.
long arbotGetX();
long arbotGetY();
long arbotGetTheta();

// Set the state of the LED next to the advance button on the iRobot Create.  0 = off and 1 = on.
void arbotSetAdvanceLed(long intensity);

// Set the forward and angular speeds in units of mm / sec and radians / seconds
void arbotSetVelocity(double forwardSpeed, double angularSpeed);
