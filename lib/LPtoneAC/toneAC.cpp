// ---------------------------------------------------------------------------
// Created by Tim Eckel - teckel@leethost.com
// Copyright 2013 License: GNU GPL v3 http://www.gnu.org/licenses/gpl-3.0.html
//
// Rewrited by Baptiste PELLEGRIN
// Copyright 2016-2019 License: GNU GPL v3
// Added low-power consumption capability 
//
// See "toneAC.h" for purpose, syntax, version history, links, and more.
// ---------------------------------------------------------------------------

#include "toneAC.h"

static bool toneACMuted = false;

#ifdef TONEAC_LENGTH
unsigned long _tAC_time; // Used to track end note with timer when playing note in the background.
#endif

#ifdef TONEAC_VOLUME
//uint8_t _tAC_volume[] = { 200, 100, 67, 50, 40, 33, 29, 22, 11, 2 }; // Duty for linear volume control.
uint8_t _tAC_volume[] = { 150, 72, 51, 38, 32, 23, 20, 19, 10, 2 }; //new duty values for three phased Low Power mode
#endif

void toneAC(unsigned long frequency
#ifdef TONEAC_VOLUME
            , uint8_t volume
#endif
#ifdef TONEAC_LENGTH
            , unsigned long length, uint8_t background
#endif
	    ) {

  /* check if no tone */ 
  if (toneACMuted || frequency == 0
#ifdef TONEAC_VOLUME     
      || volume == 0
#endif
      ) { noToneAC(); return; } 

  /* check volume */
/*
#ifdef TONEAC_VOLUME
  if (volume > 10) volume = 10;
#endif
  log_i("volume=%d, ac_volume=%d",volume,_tAC_volume[volume-1]);
*/


  /* set duty cycle */
#ifdef TONEAC_VOLUME
  //unsigned int duty = top / _tAC_volume[volume - 1]; // Calculate the duty cycle (volume).
#else
  unsigned int duty = top >> 1;
#endif

  /* compute length time */
#ifdef TONEAC_LENGTH
  if (length > 0 && background) {  // Background tone playing, returns control to your sketch.

    _tAC_time = millis() + length; // Set when the note should end.
    TIMSK1 |= _BV(OCIE1A);         // Activate the timer interrupt.
  }
#endif

#ifdef TONEAC_LENGTH
  if (length > 0 && !background) { delay(length); noToneAC(); } // Just a simple delay, doesn't return control untill finished.
#endif
  ledcWriteTone(BEEPERCHANNEL,frequency + 300);
  //_ledcSetupTimerFreq(BEEPERCHANNEL,frequency + 300,10);
  ledcWrite(BEEPERCHANNEL,volume);
}

void noToneAC() {
  ledcWrite(BEEPERCHANNEL,0);
  //Serial.println("noToneAC");
}

#ifdef TONEAC_LENGTH
ISR(TIMER1_COMPA_vect) { // Timer interrupt vector.
  if (millis() >= _tAC_time) noToneAC(); // Check to see if it's time for the note to end.
}
#endif

void toneACMute(bool newMuteState) {

  /* stop tone if needed */
  if( newMuteState ) {
    noToneAC();
  }

  /* save */
  toneACMuted = newMuteState;
}


