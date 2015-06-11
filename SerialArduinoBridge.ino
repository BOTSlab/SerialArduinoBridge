/*********************************************************************
 *  
    ROSArduinoBridge - Adapted for Memorial University's RTbot and ARbot platforms by Andrew Vardy
    ------------------------------------------------------------------------------------------------
    Changes made to the original:
      - Undefined USE_BASE (this file)
      - Adjusted pins referenced in servos.h to pins 9 and 10
      - Commented out the inclusion of MegaEncoderCounter.h in encoder_driver (this seems unnecessary but was specifically called for: https://github.com/hbrobotics/ros_arduino_bridge )      
    
    Andrew Vardy
    
    
    Original documentation follows
    -------------------------------
 
    A set of simple serial commands to control a differential drive
    robot and receive back sensor and odometry data. Default 
    configuration assumes use of an Arduino Mega + Pololu motor
    controller shield + Robogaia Mega Encoder shield.  Edit the
    readEncoder() and setMotorSpeed() wrapper functions if using 
    different motor controller or encoder method.

    Created for the Pi Robot Project: http://www.pirobot.org
    and the Home Brew Robotics Club (HBRC): http://hbrobotics.org
    
    Authors: Patrick Goebel, James Nugen

    Inspired and modeled after the ArbotiX driver by Michael Ferguson
    
    Software License Agreement (BSD License)

    Copyright (c) 2012, Patrick Goebel.
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

     * Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.
     * Redistributions in binary form must reproduce the above
       copyright notice, this list of conditions and the following
       disclaimer in the documentation and/or other materials provided
       with the distribution.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#define USE_RTBOT      // Enable the RTbot code
//#undef USE_RTBOT     // Disable the RTbot code

#define USE_STEPPER_INTERRUPT      // Enable the RTbot to utilize interrupts to update stepper_pos_left and stepper_pos_right
//#undef USE_STEPPER_INTERRUPT     // Disable stepper interrupt code

//#define USE_ARBOT      // Enable the ARbot code
#undef USE_ARBOT     // Disable the ARbot code

//#define USE_PIXY      // Enable the Pixy code
#undef USE_PIXY     // Disable the Pixy code

//#define USE_BASE      // Enable the base controller code
#undef USE_BASE     // Disable the base controller code

/* Define the motor controller and encoder library you are using */
#ifdef USE_BASE
   /* The Pololu VNH5019 dual motor driver shield */
   #define POLOLU_VNH5019

   /* The Pololu MC33926 dual motor driver shield */
   //#define POLOLU_MC33926

   /* The RoboGaia encoder shield */
   #define ROBOGAIA
#endif

#define USE_SERVOS  // Enable use of PWM servos as defined in servos.h
//#undef USE_SERVOS     // Disable use of PWM servos

/* Serial port baud rate */
#define BAUDRATE     57600

/* Maximum PWM signal */
#define MAX_PWM        255

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

/* Include definition of serial commands */
#include "commands.h"

/* Sensor functions */
#include "sensors.h"
#ifdef USE_PIXY
  #include <SPI.h>
  #include <Pixy.h>
#endif

/* Include servo support if required */
#ifdef USE_SERVOS
   #include <Servo.h>
   #include "servos.h"
#endif

/* Include RTbot support if required */
#ifdef USE_RTBOT
  #include "rtbot.h"
#endif

/* Include ARbot support if required */
#ifdef USE_ARBOT
  #include "arbot.h"
#endif

#ifdef USE_BASE
  /* Motor driver function definitions */
  #include "motor_driver.h"

  /* Encoder driver function definitions */
  #include "encoder_driver.h"

  /* PID parameters and functions */
  #include "diff_controller.h"

  /* Run the PID loop at 30 times per second */
  #define PID_RATE           30     // Hz

  /* Convert the rate into an interval */
  const int PID_INTERVAL = 1000 / PID_RATE;
  
  /* Track the next time we make a PID calculation */
  unsigned long nextPID = PID_INTERVAL;

  /* Stop the robot if it hasn't received a movement command
   in this number of milliseconds */
  #define AUTO_STOP_INTERVAL 2000
  long lastMotorCommand = AUTO_STOP_INTERVAL;
#endif

/* Variable initialization */

// A pair of varibles to help parse serial commands (thanks Fergs)
int arg = 0;
int index = 0;

// Variable to hold an input character
char chr;

// Variable to hold the current single-character command
char cmd;

// Character arrays to hold the first and second arguments
char argv1[16];
char argv2[16];

// The arguments converted to integers
long arg1;
long arg2;

#ifdef USE_PIXY
  // Camera for color detection
  Pixy pixy;
  
  // Declare Pixy functions
  uint16_t getPixyBlobCount();
  uint16_t getBlobType(int index);
  uint16_t getBlobX(int index);
  uint16_t getBlobY(int index);
  uint16_t getBlobWidth(int index);
  uint16_t getBlobHeight(int index);
#endif

/* Clear the current command parameters */
void resetCommand() {
  cmd = NULL;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = 0;
  arg2 = 0;
  arg = 0;
  index = 0;
}

/* Run a command.  Commands are defined in commands.h */
int runCommand() {
  int i = 0;
  char *p = argv1;
  char *str;
  int pid_args[4];
  arg1 = atoi(argv1);
  arg2 = atoi(argv2);
  
  switch(cmd) {
  case GET_BAUDRATE:
    Serial.println(BAUDRATE);
    break;
  case ANALOG_READ:
    Serial.println(analogRead(arg1));
    break;
  case DIGITAL_READ:
    Serial.println(digitalRead(arg1));
    break;
  case ANALOG_WRITE:
    analogWrite(arg1, arg2);
    Serial.println("OK"); 
    break;
  case DIGITAL_WRITE:
    if (arg2 == 0) digitalWrite(arg1, LOW);
    else if (arg2 == 1) digitalWrite(arg1, HIGH);
    Serial.println("OK"); 
    break;
  case PIN_MODE:
    if (arg2 == 0) pinMode(arg1, INPUT);
    else if (arg2 == 1) pinMode(arg1, OUTPUT);
    Serial.println("OK");
    break;
  case PING:
    Serial.println(Ping(arg1));
    break;
#ifdef USE_RTBOT
  case RTBOT_READ_ODOMETRY:
    Serial.print(rtbotGetX());
    Serial.print(" ");
    Serial.print(rtbotGetY());
    Serial.print(" ");
    Serial.println(rtbotGetTheta());
    break;
  case RTBOT_SET_MOTORS:
    rtbotSetMotors(arg1);
    Serial.println("OK");
    break;
#endif
#ifdef USE_ARBOT
  case ARBOT_READ_ODOMETRY:
    Serial.print(arbotGetX());
    Serial.print(" ");
    Serial.print(arbotGetY());
    Serial.print(" ");
    Serial.println(arbotGetTheta());
    break;
  case ARBOT_SET_ADVANCE_LED:
    arbotSetAdvanceLed(arg1);
    Serial.println("OK");
    break;
  case ARBOT_SET_VELOCITY:
    arbotSetVelocity(arg1, arg2);
    Serial.println("OK");
    break;
#endif
#ifdef USE_PIXY
  case GET_PIXY_BLOB_COUNT:
    Serial.println(getPixyBlobCount());
    Serial.println("OK");
    break;
  case GET_BLOB_AT_INDEX:
    Serial.print(getBlobType(arg1));
    Serial.print(" ");
    Serial.print(getBlobX(arg1));
    Serial.print(" ");
    Serial.print(getBlobY(arg1));
    Serial.print(" ");
    Serial.print(getBlobWidth(arg1));
    Serial.print(" ");
    Serial.println(getBlobHeight(arg1));
    break;
#endif
    
#ifdef USE_SERVOS
  case SERVO_WRITE:
    servos[arg1].write(arg2);
    Serial.println("OK");
    break;
  case SERVO_READ:
    Serial.println(servos[arg1].read());
    break;
#endif
    
#ifdef USE_BASE
  case READ_ENCODERS:
    Serial.print(readEncoder(LEFT));
    Serial.print(" ");
    Serial.println(readEncoder(RIGHT));
    break;
   case RESET_ENCODERS:
    resetEncoders();
    resetPID();
    Serial.println("OK");
    break;
  case MOTOR_SPEEDS:
    /* Reset the auto stop timer */
    lastMotorCommand = millis();
    if (arg1 == 0 && arg2 == 0) {
      setMotorSpeeds(0, 0);
      moving = 0;
    }
    else moving = 1;
    leftPID.TargetTicksPerFrame = arg1;
    rightPID.TargetTicksPerFrame = arg2;
    Serial.println("OK"); 
    break;
  case UPDATE_PID:
    while ((str = strtok_r(p, ":", &p)) != '\0') {
       pid_args[i] = atoi(str);
       i++;
    }
    Kp = pid_args[0];
    Kd = pid_args[1];
    Ki = pid_args[2];
    Ko = pid_args[3];
    Serial.println("OK");
    break;
#endif
  default:
    Serial.println("Invalid Command");
    break;
  }
}

/* Setup function--runs once at startup. */
void setup() {
  Serial.begin(BAUDRATE);
  
// Initialize the RTbot if used */
#ifdef USE_RTBOT
  rtbotSetup();
#endif

// Initialize the ARbot if used */
#ifdef USE_ARBOT
  arbotSetup();
#endif

// Initialize the Pixy if used */
#ifdef USE_PIXY
  pixy.init();
#endif

// Initialize the motor controller if used */
#ifdef USE_BASE
  initMotorController();
  resetPID();
#endif

/* Attach servos if used */
#ifdef USE_SERVOS
  int i;
  for (i = 0; i < N_SERVOS; i++) {
    servos[i].attach(servoPins[i]);
  }
#endif
}

/* Enter the main loop.  Read and parse input from the serial port
   and run any valid commands. Run a PID calculation at the target
   interval and check for auto-stop conditions.
*/
void loop() {
  while (Serial.available() > 0) {
    
    // Read the next character
    chr = Serial.read();

    // Terminate a command with a CR
    if (chr == 13) {
      if (arg == 1) argv1[index] = NULL;
      else if (arg == 2) argv2[index] = NULL;
      runCommand();
      resetCommand();
    }
    // Use spaces to delimit parts of the command
    else if (chr == ' ') {
      // Step through the arguments
      if (arg == 0) arg = 1;
      else if (arg == 1)  {
        argv1[index] = NULL;
        arg = 2;
        index = 0;
      }
      continue;
    }
    else {
      if (arg == 0) {
        // The first arg is the single-letter command
        cmd = chr;
      }
      else if (arg == 1) {
        // Subsequent arguments can be more than one character
        argv1[index] = chr;
        index++;
      }
      else if (arg == 2) {
        argv2[index] = chr;
        index++;
      }
    }
  }
  
// If we are using the RTbot, run its own required inner loop
#ifdef USE_RTBOT
  rtbotLoop();
#endif
  
// If we are using the ARbot, run its own required inner loop
#ifdef USE_ARBOT
  arbotLoop();
#endif

// If we are using base control, run a PID calculation at the appropriate intervals
#ifdef USE_BASE
  if (millis() > nextPID) {
    updatePID();
    nextPID += PID_INTERVAL;
  }
  
  // Check to see if we have exceeded the auto-stop interval
  if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL) {;
    setMotorSpeeds(0, 0);
    moving = 0;
  }

#endif
}

#ifdef USE_PIXY
// returns number of blobs detected by pixy
uint16_t getPixyBlobCount()
{
    return pixy.getBlocks();
}

uint16_t getBlobType(int index)
{
    return pixy.blocks[index].signature-1;
}

uint16_t getBlobX(int index)
{
    return pixy.blocks[index].x;
}

uint16_t getBlobY(int index)
{
    return pixy.blocks[index].y;
}

uint16_t getBlobWidth(int index)
{
    return pixy.blocks[index].width;
}

uint16_t getBlobHeight(int index)
{
    return pixy.blocks[index].height;
}
#endif
