#ifdef USE_RTBOT

/** 
 * Code to interact with the RTbots (Red-team robots) primarily form the
 * perspective of using stepper motor counts for localization.
 */

#include <avr/interrupt.h>

long loop_count = 0;

// Universal constants
double DEGREES_TO_RADIANS = M_PI / 180.0;
double RADIANS_TO_DEGREES = 180.0 / M_PI;

// Pins for PIC stepper control
int RTBOT_STEPPER_CONTROL_2 = 4;
int RTBOT_STEPPER_CONTROL_1 = 5;
int RTBOT_STEPPER_CONTROL_0 = 6;

// Parameters used to convert from stepper_pos_left, stepper_pos_right into the
// angular positions of the wheels in radians.
double WHEEL_RADIUS_MM = 73.025;
double RADIANS_PER_STEP = 0.9 * DEGREES_TO_RADIANS;
double DISTANCE_PER_STEP = WHEEL_RADIUS_MM * RADIANS_PER_STEP;

// Baseline distance between the wheels in mm
double BASELINE_MM = 103.0;
double HALF_BASELINE_MM = BASELINE_MM / 2.0;

enum Direction { FORWARD, TURN_LEFT, TURN_RIGHT, STOP, REVERSE };
Direction direction = STOP;

// Position of the two motors in terms of number of steps.
volatile int stepper_pos_left, stepper_pos_right;

// Values of stepper_pos_left and stepper_pos_right at the end of the last time
// step.  These values are compared to the current values to determine the
// relative movement for dead reckoning.
int last_stepper_pos_left, last_stepper_pos_right;

// Current pose represented in metres and radians.
double x = 0, y = 0, theta = 0;

// Directly from pages 186-7 of "Introduction to Mobile Robots" by Siegwart 
// and Nourbakhsh, First edition.
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

// Motor step interrupt handler
void stepperInterrupt() {
    switch (direction) {
        case FORWARD:
            stepper_pos_left++;
            stepper_pos_right++;
            break;
        case TURN_LEFT:
            stepper_pos_left--;
            stepper_pos_right++;
            break;
        case TURN_RIGHT:
            stepper_pos_left++;
            stepper_pos_right--;
            break;
        case STOP:
            break;
        case REVERSE:
            stepper_pos_left--;
            stepper_pos_right--;
            break;
    }
}

long rtbotGetX() {
    return (long)(x * 1000);
}

long rtbotGetY() {
    return (long)(y * 1000);
}

long rtbotGetTheta() {
    return (long)(theta * 1000);
}

/*
void rtbotSetLed(long value) {
    if (value == 0)
        digitalWrite(RTBOT_LED_PIN, LOW);    
    else if (value == 1)
        digitalWrite(RTBOT_LED_PIN, HIGH);
}
*/

/*
 * Low-level control functions for motors.  The microcontroller on the robot
 * control board sends a 3-bit control-word to the motors as follows:
 *  B2 | B1 | B0 | State
 *  --------------------
 *  0  | 0  | 0  | FORWARD
 *  0  | 0  | 1  | TURN LEFT
 *  0  | 1  | 0  | TURN RIGHT
 *  0  | 1  | 1  | STOP (NOT MOVING AND MOTORS POWERED DOWN)
 *  1  | 0  | 0  | REVERSE
 *  1  | 1  | 1  | HOLD (NOT MOVING BUT MOTORS STILL POWERED)
 *
 * See ~/Dropbox/redteamrobotics2011/dropbox
 *       copy/software/powertrain/pic/finalcontroller.c
 *
 * Its also important to set stepper_dir_left and stepper_dir_right for each of
 * these so that the stepper interrupt handler keeps track appropriately.
 */
  
void rtbotStop() {
    digitalWrite(RTBOT_STEPPER_CONTROL_2, LOW);
    digitalWrite(RTBOT_STEPPER_CONTROL_1, HIGH);
    digitalWrite(RTBOT_STEPPER_CONTROL_0, HIGH);
}

void rtbotHold() {
    digitalWrite(RTBOT_STEPPER_CONTROL_2, HIGH);
    digitalWrite(RTBOT_STEPPER_CONTROL_1, HIGH);
    digitalWrite(RTBOT_STEPPER_CONTROL_0, HIGH);
}

void rtbotForward() {
    digitalWrite(RTBOT_STEPPER_CONTROL_2, LOW);
    digitalWrite(RTBOT_STEPPER_CONTROL_1, LOW);
    digitalWrite(RTBOT_STEPPER_CONTROL_0, LOW);
}

void rtbotReverse() {
    digitalWrite(RTBOT_STEPPER_CONTROL_2, HIGH);
    digitalWrite(RTBOT_STEPPER_CONTROL_1, LOW);
    digitalWrite(RTBOT_STEPPER_CONTROL_0, LOW);
}

void rtbotTurnLeft() {
    digitalWrite(RTBOT_STEPPER_CONTROL_2, LOW);
    digitalWrite(RTBOT_STEPPER_CONTROL_1, LOW);
    digitalWrite(RTBOT_STEPPER_CONTROL_0, HIGH);
}

void rtbotTurnRight() {
    digitalWrite(RTBOT_STEPPER_CONTROL_2, LOW);
    digitalWrite(RTBOT_STEPPER_CONTROL_1, HIGH);
    digitalWrite(RTBOT_STEPPER_CONTROL_0, LOW);
}

void rtbotSetMotors(int stp_for_rev_lft_rgt) {
    if (stp_for_rev_lft_rgt == 0)
        direction = STOP;
    else if (stp_for_rev_lft_rgt == 1)
        direction = FORWARD;
    else if (stp_for_rev_lft_rgt == 2)
        direction = REVERSE;
    else if (stp_for_rev_lft_rgt == 3)
        direction = TURN_LEFT;
    else if (stp_for_rev_lft_rgt == 4)
        direction = TURN_RIGHT;
}

double sign(double x) {
    if (x > 0) return 1;
    if (x < 0) return -1;
    return 0;
}

/**
 * Driving the robot by specifing desired forward and angular speeds is
 * difficult because the control board only accepts forward, reverse commands
 * and turning left or right, on the spot.  The approach adopted here is to
 * accept a speed integer which is interpreted as either stop (0) or go (0) and
 * an integer heading in degrees.  Depending upon the value of heading we will
 * alternate between forward/reverse commands and turn-left/turn-right commands
 * so as to drive the robot towards the desired heading.
 *
 * speed = 0 means stopped,
 * speed = 1 means go.
 *
 * heading is interpreted as an angle.  For angles in the range [-90, 90],
 * the robot will drive forwards (or turn on the spot for -90 and 90).  For
 * angles in the ranges [91, 180] or [-91, -180] the robot will drive
 * backwards.
 */
/*
void rtbotDrive(int speed, int heading) {
    if (heading >= -90)
    
    drive_divisor = 

    drive_straight = loop_count % stepper_divisor_left == 0;
    drive_turn = stepper_on_right = loop_count % stepper_divisor_right == 0;

    // Set the desired directions of both steppers.
    stepper_dir_left = sign(wheel_speed_left);
    stepper_dir_right = sign(wheel_speed_right);

    // Finally, determine what control word to send to the PIC based on
    // stepper_dir_left and stepper_dir_right...
    if (stepper_dir_left == 0 && stepper_dir_right == 0)
        rtbotStop();
    else if (stepper_dir_left == 1 && stepper_dir_right == 1)
        rtbotForward();
    else if (stepper_dir_left == -1 && stepper_dir_right == -1)
        rtbotReverse();
    else if (stepper_dir_left == -1 && stepper_dir_right == 1)
        rtbotTurnLeft();     
    else if (stepper_dir_left == 1 && stepper_dir_right == -1)
        rtbotTurnRight();
    else if (stepper_dir_left == 1 && stepper_dir_right == 0)
        rtbotForward();
        rtbotTurnRight();
    else if (stepper_dir_left == -1 && stepper_dir_right == 0)
        rtbotForward();
        rtbotTurnRight();
}
*/

void move() {
    switch (direction) {
        case FORWARD:
            rtbotForward();
            break;
        case TURN_LEFT:
            rtbotTurnLeft();     
            break;
        case TURN_RIGHT:
            rtbotTurnRight();     
            break;
        case STOP:
            rtbotStop();
            break;
        case REVERSE:
            rtbotReverse();
            break;
    }
}

void rtbotSetup(){
    // Setup the pins to control the stepper motors (via the PIC)  
    pinMode(RTBOT_STEPPER_CONTROL_2, OUTPUT);
    pinMode(RTBOT_STEPPER_CONTROL_1, OUTPUT);
    pinMode(RTBOT_STEPPER_CONTROL_0, OUTPUT);

    // Setup the interrupt for the stepper feedback
    attachInterrupt(1, stepperInterrupt, RISING);
}

void rtbotLoop(){
    updatePose();

    move();

    last_stepper_pos_left = stepper_pos_left;
    last_stepper_pos_right = stepper_pos_right;
    loop_count++;
}

#endif
