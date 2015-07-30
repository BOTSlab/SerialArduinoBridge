#ifdef USE_BUPIGO

/** 
 * Code to interact with a BuPiGo which consists of a MinSegMega connected to
 * two Lego NXT motors.
 */

// Universal constants
double DEGREES_TO_RADIANS = M_PI / 180.0;
double RADIANS_TO_DEGREES = 180.0 / M_PI;

// Parameters used to convert from encoder values to movements of the wheels
// along the floor.
double WHEEL_RADIUS_MM = 28.0;
double RADIANS_PER_STEP = 0.5 * DEGREES_TO_RADIANS; // Each encoder pulse is 0.5 deg of a turn
double DISTANCE_PER_STEP = WHEEL_RADIUS_MM * RADIANS_PER_STEP;

// Baseline distance between the wheels in mm
double BASELINE_MM = 103.0;
double HALF_BASELINE_MM = BASELINE_MM / 2.0;

// Pin ID's for setting motors
int MOTOR_LEFT_FORWARD = 44;  // Right motor Forward (M1)
int MOTOR_LEFT_BACKWARD = 45; // Right motor Reverse (M2)
int MOTOR_RIGHT_FORWARD = 2;    // Left motor Forward  (M3)
int MOTOR_RIGHT_BACKWARD = 46;  // Left motor Reverse  (M4)

// Pin ID for servo
int SERVO_PIN = 3;

// The servo object
ServoTimer2 servo;

// Current pose represented in metres and radians.
double x = 0, y = 0, theta = 0;

/*******************  Encoder Variables ********************/
// Left encoder pins
int encoderPin_LA = 19;
int encoderPin_LB = 18;

// Position tracking variables for left/right encoders
volatile int encoder_pos_right = 0;
volatile int encoder_pos_left = 0;
int last_encoder_pos_right = 0;
int last_encoder_pos_left = 0;

// Quadrature encoder state variables set during interrupts
volatile bool LA_state = false; // Left encoder A
volatile bool LB_state = false; // Left encoder B
volatile bool RA_state = false; // Right encoder A
volatile bool RB_state = false; // Right encoder B
/*************************************************************/

// Directly from pages 186-7 of "Introduction to Mobile Robots" by Siegwart 
// and Nourbakhsh, First edition.

void updatePose() {
    // Distance travelled by left and right wheels
    double delta_s_l = DISTANCE_PER_STEP * 
                        (encoder_pos_left - last_encoder_pos_left);
    double delta_s_r = DISTANCE_PER_STEP *
                        (encoder_pos_right - last_encoder_pos_right);

    double delta_s = (delta_s_l + delta_s_r) / 2.0;
    double delta_theta = (delta_s_r - delta_s_l) / BASELINE_MM;

    x += delta_s * cos(theta + delta_theta / 2.0);
    y += delta_s * sin(theta + delta_theta / 2.0);
    theta += delta_theta;
}


long bupigoGetX() {
    return (long)(x * 1000);
}

long bupigoGetY() {
    return (long)(y * 1000);
}

long bupigoGetTheta() {
    return (long)(theta * 1000);
}

void bupigoSetMotors(long dir) {
  Serial.print("dir: ");
  Serial.println(dir);
  
  switch(dir){
    case 0: // Stop
      digitalWrite(MOTOR_RIGHT_FORWARD, LOW);
      digitalWrite(MOTOR_RIGHT_BACKWARD, LOW);
      digitalWrite(MOTOR_LEFT_FORWARD, LOW);
      digitalWrite(MOTOR_LEFT_BACKWARD, LOW);
      break;
    case 1: // Forward
      digitalWrite(MOTOR_RIGHT_FORWARD, HIGH);
      digitalWrite(MOTOR_RIGHT_BACKWARD, LOW);
      digitalWrite(MOTOR_LEFT_FORWARD, HIGH);
      digitalWrite(MOTOR_LEFT_BACKWARD, LOW);
      break;
    case 2: // Reverse
      digitalWrite(MOTOR_RIGHT_FORWARD, LOW);
      digitalWrite(MOTOR_RIGHT_BACKWARD, HIGH);
      digitalWrite(MOTOR_LEFT_FORWARD, LOW);
      digitalWrite(MOTOR_LEFT_BACKWARD, HIGH);
      break;
    case 3: // Turn Left
      digitalWrite(MOTOR_RIGHT_FORWARD, HIGH);
      digitalWrite(MOTOR_RIGHT_BACKWARD, LOW);
      digitalWrite(MOTOR_LEFT_FORWARD, LOW);
      digitalWrite(MOTOR_LEFT_BACKWARD, HIGH);
      break;
    case 4: // Turn Right
      digitalWrite(MOTOR_RIGHT_FORWARD, LOW);
      digitalWrite(MOTOR_RIGHT_BACKWARD, HIGH);
      digitalWrite(MOTOR_LEFT_FORWARD, HIGH);
      digitalWrite(MOTOR_LEFT_BACKWARD, LOW);
      break;
  }
}

void bupigoSetSpeeds(long leftSpeed, long rightSpeed) {
  if (leftSpeed >= 0) {   
    digitalWrite(MOTOR_LEFT_BACKWARD, LOW);
    analogWrite(MOTOR_LEFT_FORWARD, leftSpeed);
  } else {
    digitalWrite(MOTOR_LEFT_FORWARD, LOW);
    analogWrite(MOTOR_LEFT_BACKWARD, -leftSpeed);
  }

  if (rightSpeed >= 0) {
    digitalWrite(MOTOR_RIGHT_BACKWARD, LOW);
    analogWrite(MOTOR_RIGHT_FORWARD, rightSpeed);
  } else {
    digitalWrite(MOTOR_RIGHT_FORWARD, LOW);
    analogWrite(MOTOR_RIGHT_BACKWARD, -rightSpeed);
  } 
}

void bupigoSetVelocity(long forwardSpeed, long angularSpeed) {
  double v = forwardSpeed; // Forward speed in mm / sec
  double w = angularSpeed / 1000.0; // Angular speed in radians / sec
  
  // Apply differential-drive kinematic model.
  long leftVelocity = (long)( (v - HALF_BASELINE_MM * w) / WHEEL_RADIUS_MM);
  long rightVelocity = (long)( (v + HALF_BASELINE_MM * w) / WHEEL_RADIUS_MM);

  bupigoSetSpeeds(leftVelocity, rightVelocity);
}

void bupigoSetServoAngle(long angle) {
  // Need to convert to microseconds for ServoTimer2
  long ms = map(angle, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  servo.write(ms);
}  

/******************************************************************************

  Interrupt functions to determine the direction and amount of rotation for each
  motor. The encoder on each motor takes two readings (A and B). The graphs below
  illustrate the pulse patterns generated by rotation in each direction.

                A leads B                             B leads A
      __________________________             __________________________
      |                                      |
      |__      ____      ____                |____      ____      ____ 
Pin A |  |____|    |____|    |__      Pin A  |    |____|    |____|    |
      |                                      |
      |____      ____      ____              |__      ____      ____   
Pin B |    |____|    |____|    |      Pin B  |  |____|    |____|    |__
      |                                      |
      |__________________________            |__________________________
      
            Moving Forward                         Moving Backward  
      
*******************************************************************************/

// Interrupt triggered on RA changing state
void doEncoderRA(){
  RA_state = !RA_state;
  if (RA_state != RB_state){
    encoder_pos_right++;
  }
  else{
    encoder_pos_right--;
  }
}

// Interrupt triggered on RB changing state
void doEncoderRB(){
  RB_state = !RB_state;
  if (RA_state == RB_state){
    encoder_pos_right++;
  }
  else{
    encoder_pos_right--;
  }
}

// Interrupt triggered on LA changing state
void doEncoderLA(){
  LA_state = !LA_state;
  if (LA_state != LB_state){
    encoder_pos_left++;
  }
  else{
    encoder_pos_left--;
  }
}

// Interrupt triggered on LB changing state
void doEncoderLB(){
  LB_state = !LB_state;
  if (LA_state == LB_state){
    encoder_pos_left++;
  }
  else{
    encoder_pos_left--;
  }
}

void bupigoSetup(){
  // Set initial state for right encoder
  pinMode(encoderPin_LA, INPUT);
  pinMode(encoderPin_LB, INPUT);
  LA_state = digitalRead(encoderPin_LA);
  LB_state = digitalRead(encoderPin_LB);
  
  // Set initial state for right encoder
  // Encoder is attached to unmapped pins E6/E7, hence the low level port manipulation
  DDRE &= 0x3f;
  PORTE |= 0xC0;
  RA_state = ((PINE & 0x40) == 0x40);
  RB_state = ((PINE & 0x80) == 0x80);
  
  // Attach interrupt functions for each encoder pin
  // to their corresponding interrupt pin. Each time
  // the value of an encoder pin changes, the program
  // pauses its routine to evaluate the triggered function.
  attachInterrupt(5, doEncoderLB, CHANGE);
  attachInterrupt(4, doEncoderLA, CHANGE);
  attachInterrupt(6, doEncoderRA, CHANGE);
  attachInterrupt(7, doEncoderRB, CHANGE);
  
  
  // Set motor control pins to be outputs
  pinMode(MOTOR_RIGHT_FORWARD, OUTPUT);
  pinMode(MOTOR_RIGHT_BACKWARD, OUTPUT);
  pinMode(MOTOR_LEFT_FORWARD, OUTPUT);
  pinMode(MOTOR_LEFT_BACKWARD, OUTPUT);
  
  servo.attach(SERVO_PIN);
}

void bupigoLoop(){
    updatePose();

    last_encoder_pos_left = encoder_pos_left;
    last_encoder_pos_right = encoder_pos_right;
}

#endif
