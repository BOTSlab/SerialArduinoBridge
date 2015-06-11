#ifdef USE_MSM

/** 
 * Code to interact with a MinSegMega connected to two Lego NXT motors.
 */

// Universal constants
double DEGREES_TO_RADIANS = M_PI / 180.0;
double RADIANS_TO_DEGREES = 180.0 / M_PI;

// Parameters used to convert from encoder values to movements of the wheels
// along the floor.
double WHEEL_RADIUS_MM = 28.0;
double RADIANS_PER_STEP = 1.0 * DEGREES_TO_RADIANS;
double DISTANCE_PER_STEP = WHEEL_RADIUS_MM * RADIANS_PER_STEP;

// Baseline distance between the wheels in mm
double BASELINE_MM = 103.0;
double HALF_BASELINE_MM = BASELINE_MM / 2.0;

// Pin ID's for setting motor PWM and direction.
int MOTOR1_PWM = 2;
int MOTOR1_DIR = 46;

int MOTOR2_PWM = 44;
int MOTOR2_DIR = 45;

// Current pose represented in metres and radians.
double x = 0, y = 0, theta = 0;

// Directly from pages 186-7 of "Introduction to Mobile Robots" by Siegwart 
// and Nourbakhsh, First edition.
/*
void updatePose() {
    // Distance travelled by left and right wheels
    double delta_s_l = DISTANCE_PER_STEP * 
                        (stepper_pos_left - last_stepper_pos_left);
    double delta_s_r = DISTANCE_PER_STEP *
                        (stepper_pos_right - last_stepper_pos_right);

    double delta_s = (delta_s_l + delta_s_r) / 2.0;
    double delta_theta = (delta_s_r - delta_s_l) / BASELINE_MM;

    x += delta_s * cos(theta + delta_theta / 2.0);
    y += delta_s * sin(theta + delta_theta / 2.0);
    theta += delta_theta;
}
*/

long msmGetX() {
    return (long)(x * 1000);
}

long msmGetY() {
    return (long)(y * 1000);
}

long msmGetTheta() {
    return (long)(theta * 1000);
}

void msmSetMotors(int leftDir, int leftPWM, int rightDir, int rightPWM) {
    analogWrite(MOTOR1_PWM, leftPWM);
    digitalWrite(MOTOR1_DIR, leftDir);

    analogWrite(MOTOR2_PWM, rightPWM);
    digitalWrite(MOTOR2_DIR, rightDir);
}

void msmSetup(){
  pinMode(MOTOR1_DIR, OUTPUT);
  pinMode(MOTOR2_DIR, OUTPUT);
}

void msmLoop(){
    //updatePose();

    //last_stepper_pos_left = stepper_pos_left;
    //last_stepper_pos_right = stepper_pos_right;
}

#endif
