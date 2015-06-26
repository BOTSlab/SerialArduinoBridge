// MinSegMega specific setup and loop functions
void msmSetup();
void msmLoop();

// Read x, y, or theta coordinate of current odometry estimate.  The returned
// values are in mm for x and y and in radians * 1000 for theta.
long msmGetX();
long msmGetY();
long msmGetTheta();

// Set the direction of movement
void msmSetMotors(int dir);

//void msmDrive(int speed, int heading);
