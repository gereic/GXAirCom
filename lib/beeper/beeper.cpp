/* beeper -- Make beeps
 *
 * Copyright 2016-2019 Baptiste PELLEGRIN
 * 
 * This file is part of GNUVario.
 *
 * GNUVario is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * GNUVario is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <Arduino.h>
#include <beeper.h>
#include <toneAC.h>

beeper::beeper(double sinkingThreshold, double climbingThreshold, double nearClimbingSensitivity, uint8_t baseVolume) {

  /* save volume */
  volume = baseVolume;
  
  /* set threshold */
  setThresholds(sinkingThreshold, climbingThreshold, nearClimbingSensitivity);
  
  /* init private vars */
  beepStartTime = 0;
  beepState = 0;
  beepType = BEEP_TYPE_SILENT;
}

void beeper::setThresholds(double sinkingThreshold, double climbingThreshold, double nearClimbingSensitivity) {

  beepSinkingThreshold = sinkingThreshold;
  beepGlidingThreshold = climbingThreshold - nearClimbingSensitivity;
  beepClimbingThreshold = climbingThreshold;
}

void beeper::setVolume(uint8_t newVolume) {

  volume = newVolume;
}

void beeper::setGlidingBeepState(boolean status) {
  if( status ) {
    bst_set(GLIDING_BEEP_ENABLED);
    if( beepType == BEEP_TYPE_GLIDING ) {
      beepStartTime = millis();
      beepPaternBasePosition = 0.0;
      beepPaternPosition = 0.0;
    }
  } else {
    bst_unset(GLIDING_BEEP_ENABLED);
  }
}
	    
void beeper::setGlidingAlarmState(boolean status) {
  if( status ) {
    bst_set(GLIDING_ALARM_ENABLED);
  } else {
    bst_unset(GLIDING_ALARM_ENABLED);
  }
}


void beeper::setBeepParameters(double velocity) {

  /* save velocity */
  beepVelocity = velocity;

  /* compute the beep freq that depend to beep type */
  switch( beepType ) {
  case BEEP_TYPE_SINKING :
    beepFreq = SINKING_BEEP_FREQ_COEFF * velocity + SINKING_BEEP_BASE_FREQ;
    break;

  case BEEP_TYPE_SILENT :
    beepFreq = 0.0;
    break;

  case BEEP_TYPE_GLIDING :
    beepFreq = CLIMBING_BEEP_FREQ_COEFF * velocity + CLIMBING_BEEP_BASE_FREQ;
    break;

  case BEEP_TYPE_CLIMBING :
    beepFreq = CLIMBING_BEEP_FREQ_COEFF * velocity + CLIMBING_BEEP_BASE_FREQ;
    break;
  }

}


void beeper::setVelocity(double velocity) {
  
  /* check if we need to change the beep type */
  boolean beepTypeChange = false;
  switch( beepType ) {
  case BEEP_TYPE_SINKING :
    if( velocity >  beepSinkingThreshold + BEEP_VELOCITY_SENSITIVITY )
      beepTypeChange = true;
    break;

  case BEEP_TYPE_SILENT :
    if( velocity < beepSinkingThreshold || velocity > beepGlidingThreshold )
      beepTypeChange = true;
    break;

  case BEEP_TYPE_GLIDING :
    if( velocity < beepGlidingThreshold - BEEP_VELOCITY_SENSITIVITY || velocity > beepClimbingThreshold )
      beepTypeChange = true;
    break;

  case BEEP_TYPE_CLIMBING :
    if( velocity < beepClimbingThreshold - BEEP_VELOCITY_SENSITIVITY )
       beepTypeChange = true;
    break;
  }

  /* check if alarm need to be started */
  boolean startAlarm = false;
  if( bst_isset(GLIDING_ALARM_ENABLED) && beepTypeChange && !bst_isset(CLIMBING_ALARM) && !bst_isset(SINKING_ALARM) ) {
    /* need climbing alarm ? */
    if( beepType == BEEP_TYPE_SINKING || beepType == BEEP_TYPE_SILENT ) {
      if( velocity > beepGlidingThreshold && velocity < beepClimbingThreshold ) {
        startAlarm = true;
        bst_set(CLIMBING_ALARM);
      }
    }
    /* else need sinking alarm ? */
    else {
      if( velocity > beepSinkingThreshold && velocity < beepGlidingThreshold ) {
        startAlarm = true;
        bst_set(SINKING_ALARM);
      }
    }
  }

  /* check if alarm need to be stopped */
  /* (when climbing or sinking beep start ) */
  if( ( beepTypeChange ) &&
      ( bst_isset(CLIMBING_ALARM) || bst_isset(SINKING_ALARM) ) &&
      ( velocity > beepClimbingThreshold || velocity < beepSinkingThreshold ) ) {
    bst_unset(CLIMBING_ALARM);
    bst_unset(SINKING_ALARM);
  }
      
	

  /* start a new beep if needed */
  if( (beepTypeChange && !bst_isset(CLIMBING_ALARM) && !bst_isset(SINKING_ALARM) ) ||
      (startAlarm) ) {
    beepStartTime = millis();
    beepPaternBasePosition = 0.0;
    beepPaternPosition = 0.0;
    bst_set(BEEP_NEW_FREQ); //force changing freq
  }

  /* set the new beep type if changed */
  if( beepTypeChange ) {
    if( velocity < beepSinkingThreshold ) {
      beepType = BEEP_TYPE_SINKING;      
    } else if( velocity < beepGlidingThreshold ) {
      beepType = BEEP_TYPE_SILENT;
    } else if( velocity < beepClimbingThreshold ) {
      beepType = BEEP_TYPE_GLIDING;
    } else {
      beepType = BEEP_TYPE_CLIMBING;
    }
    //log_i("new beepType=%d",beepType);
  }
 
  /* check if we need to change the beep parameters  */
  /* !!! not forcing freq change on patern beeps !!! */
  /*     just wait for the next silent               */
  if( startAlarm || beepTypeChange ||
      velocity > beepVelocity + BEEP_VELOCITY_SENSITIVITY ||
      velocity < beepVelocity - BEEP_VELOCITY_SENSITIVITY ) {
    setBeepParameters(velocity);
    if( beepType == BEEP_TYPE_SINKING )
      bst_set(BEEP_NEW_FREQ);  //sinking beep don't have patern
  }
}

void beeper::setBeepPaternPosition(double velocity) {

  /* check alarm */
  boolean haveAlarm = false;
  if( bst_isset(CLIMBING_ALARM) || bst_isset(SINKING_ALARM) ) {
    haveAlarm = true;
  }

  /***********************************/
  /* check if the beep have a patern */
  /***********************************/
  if( !haveAlarm &&
      (beepType == BEEP_TYPE_SINKING || beepType == BEEP_TYPE_SILENT) ) {
    return;
  }
  
  unsigned long currentTime = millis();
  double currentLength = (double)(currentTime - beepStartTime) / 1000.0;

  /*******************************************/
  /* does the position depends on velocity ? */
  /*******************************************/
  if( !haveAlarm &&
      beepType == BEEP_TYPE_CLIMBING ) {
    currentLength *= (beepVelocity * CLIMBING_BEEP_VELOCITY_FILTER_COEFF + CLIMBING_BEEP_VELOCITY_FILTER_BASE);

    /* avoid going backward */
    if( currentLength + beepPaternBasePosition > beepPaternPosition ) {
      beepPaternPosition = currentLength + beepPaternBasePosition;
    }
  } else {
    beepPaternPosition = currentLength;
  }

  /**************************************/
  /* check if the patern end is reached */
  /**************************************/

  /* alarm case */
  if( haveAlarm ) {
    /* climbing alarm */
    if( bst_isset(CLIMBING_ALARM) ) {
      /* if alarm done, reset */
      if( beepPaternPosition > CLIMBING_ALARM_LENGTH ) {
	bst_unset(CLIMBING_ALARM);
	beepStartTime = currentTime;
	beepPaternBasePosition = 0.0;
	beepPaternPosition = 0.0;
	setBeepPaternPosition(velocity);
	bst_set(BEEP_NEW_FREQ);
	return;
      }
    }
    /* sinking alarm */
    else {
      /* if alarm done reset */
      if( beepPaternPosition > SINKING_ALARM_LENGTH ) {
	bst_unset(SINKING_ALARM);
	beepStartTime = currentTime;
	beepPaternBasePosition = 0.0;
	beepPaternPosition = 0.0;
	setBeepPaternPosition(velocity);
	bst_set(BEEP_NEW_FREQ);
	return;
      }
    }
  }
  /* looping patern case */
  else {
    double loopingPaternLength;
    if(  beepType == BEEP_TYPE_GLIDING ) {
      loopingPaternLength = GLIDING_BEEP_LENGTH;
    }else {
      loopingPaternLength = CLIMBING_BEEP_LENGTH;
    }

    while( beepPaternPosition > loopingPaternLength ) {
        beepPaternPosition -= loopingPaternLength;
        beepStartTime = millis();
        beepPaternBasePosition = beepPaternPosition;
    }
  }
}


void beeper::setTone() {
  
  /* alarme case */
  if(  bst_isset(CLIMBING_ALARM) || bst_isset(SINKING_ALARM) ) { 

    /******************/
    /* climbing alarm */
    /******************/
    if( bst_isset(CLIMBING_ALARM) ) {

      /* get half position */
      double halfPaternPosition = beepPaternPosition;
      if( halfPaternPosition > (CLIMBING_ALARM_LENGTH/2.0) ) {
	halfPaternPosition -= (CLIMBING_ALARM_LENGTH/2.0);
      }

      /* set tone */
      if( halfPaternPosition < CLIMBING_ALARM_HIGH_LENGTH ) {
	if( !bst_isset(BEEP_HIGH) ) {
	  toneAC(CLIMBING_ALARM_FREQ, volume);
	  bst_set(BEEP_HIGH);
	} else if( bst_isset(BEEP_NEW_FREQ) ) {
	  toneAC(CLIMBING_ALARM_FREQ, volume);
	}
      } else {
	toneAC(0.0);
	bst_unset(BEEP_HIGH);
      }
    }

    /*****************/
    /* sinking alarm */
    /*****************/
    else {
      if( !bst_isset(BEEP_HIGH) || bst_isset(BEEP_NEW_FREQ) ) {
	toneAC(SINKING_ALARM_FREQ, volume);
	bst_set(BEEP_HIGH);
      }
    }
  } else {
    
    /****************/
    /* sinking beep */
    /****************/
    if( beepType == BEEP_TYPE_SINKING ) {
      if( !bst_isset(BEEP_HIGH) || bst_isset(BEEP_NEW_FREQ) ) {
	toneAC(beepFreq, volume);
	bst_set(BEEP_HIGH);
      }
    }

    /**********/
    /* silent */
    /**********/
    else if( beepType == BEEP_TYPE_SILENT ) {
      toneAC(0.0);
      bst_unset(BEEP_HIGH);
    }

    /***********/
    /* gliding */
    /***********/
    else if(  beepType == BEEP_TYPE_GLIDING ) {
      if( bst_isset(GLIDING_BEEP_ENABLED) ) {
	if( beepPaternPosition < GLIDING_BEEP_HIGH_LENGTH ) {
	  if( !bst_isset(BEEP_HIGH) ) {
	    toneAC(beepFreq, volume);
	    bst_set(BEEP_HIGH);
	  } else if( bst_isset(BEEP_NEW_FREQ) ) {
	    toneAC(beepFreq, volume);
	  }
	} else {
	  toneAC(0.0);
	  bst_unset(BEEP_HIGH);
	}
      } else {
	toneAC(0.0);
	bst_unset(BEEP_HIGH);
      }
    }

    /************/
    /* climbing */
    /************/
    else {
      if( beepPaternPosition < CLIMBING_BEEP_HIGH_LENGTH ) {
	if( !bst_isset(BEEP_HIGH) ) {
	  toneAC(beepFreq, volume);
	  bst_set(BEEP_HIGH);
	} else if( bst_isset(BEEP_NEW_FREQ) ) {
	  toneAC(beepFreq, volume);
	}
      } else {
	toneAC(0.0);
	bst_unset(BEEP_HIGH);
      }
    }
  }

  /***************/
  /* tone is set */
  /***************/
  bst_unset(BEEP_NEW_FREQ);
}	


void beeper::update() {
  setBeepPaternPosition(beepVelocity);
  setTone();
  
}
