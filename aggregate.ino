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

void setup() {
  printf("Setup\n");
  
    // Initialize the ARbot if used
    arbotSetup();
}

void loop() {
    // If we are using the ARbot, run its own required inner loop
    arbotLoop();

    // Call functions in arbot.h here.
}

