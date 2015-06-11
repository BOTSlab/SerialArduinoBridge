// RTbot specific setup and loop functions
void rtbotSetup();
void rtbotLoop();

// Read x, y, or theta coordinate of current odometry estimate.  The returned values are in mm
// for x and y and in radians * 1000 for theta.
long rtbotGetX();
long rtbotGetY();
long rtbotGetTheta();

// Activate the motors to stop (0), go forward (1), reverse (2), go left (3), or go right (4)
void rtbotSetMotors(int stp_for_rev_lft_rgt);

//void rtbotDrive(int speed, int heading);
