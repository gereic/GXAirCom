// ---------------------------------------------------------------------------
// Connect a two-pin dual LED to the following pins with inline 220 ohm resistor.
//   Pins  9 & 10 - ATmega328, ATmega128, ATmega640, ATmega8, Uno, Leonardo, etc.
//   Pins 11 & 12 - ATmega2560/2561, ATmega1280/1281, Mega
//   Pins 12 & 13 - ATmega1284P, ATmega644
//   Pins 14 & 15 - Teensy 2.0
//   Pins 25 & 26 - Teensy++ 2.0
// Connect the center lead of a potentiometer to analog pin A0 and the other two leads to +5V and ground.
// ---------------------------------------------------------------------------

#include <toneAC.h>

unsigned long timestamp = 0; // Stores when the next time the routine is set to run.

void setup() {}

void loop() {
  if (millis() > timestamp) { // Is it time yet?
    timestamp += 500;         // Set the next time routine will run. 500 ms because the lowest frequency is 2 Hz, which is a half second.
    int pot = analogRead(A0);            // Read the potentiometer connected to analog pin A0 to control alternating flashing speed.
    int freq = map(pot, 0, 1023, 2, 40); // Convert pot analog values to a range from 2 to 40 Hz.
    toneAC(freq, 10, 0, true);           // Set the frequency and have it run forever in the background (next event should take over in 500 ms).
  }
  /* Do a bunch of other stuff here, it won't affect toneAC doing its thing. */
}
