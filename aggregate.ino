/*
 * Raw starting code for swarm aggregation using arbots.   
 *
 * @author: Andrew Vardy
 */

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "arbot.h"

boolean started=false;
int count=0;

void setup() {
  printf("Setup\n");
  
  // Initialize the ARbot if used
  arbotSetup();
}

void loop() {
  // If we are using the ARbot, run its own required inner loop
  arbotLoop();

  // Hitting the play button will set 'started' to true and allow
  // 'doAction' to proceed.  Hitting again will set 'started' to false.
  if (!started && arbotPlayPressed())
    startAction();
  if (started && arbotPlayPressed())
    stopAction();

  if (started) {
    // Do whatever it is we are doing!
    doAction();

    count++;
  }
}

void doAction() {
  // A simple sample action where we alternate between forwards movement and rotation.
  if ((count/10) % 2 == 0)
    arbotSetVelocity(100.0, 0.0);
  else    
    arbotSetVelocity(0.0, 1.0);
}

void startAction() {
  started = true;
  count = 0;
}

void stopAction() {
  started = false;
  arbotSetVelocity(0.0, 0.0);
  count = 0;
}

