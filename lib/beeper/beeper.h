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

#ifndef BEEPER_H
#define BEEPER_H

/***************************/
/* beep general parameters */
/***************************/
#define BEEP_DEFAULT_VOLUME 10

/* default threshold */
#define BEEP_VELOCITY_DEFAULT_SINKING_THRESHOLD (-2.0)
#define BEEP_VELOCITY_DEFAULT_CLIMBING_THRESHOLD 0.2
#define BEEP_VELOCITY_DEFAULT_NEAR_CLIMBING_SENSITIVITY 0.5

/* avoid changing beep freq too often */
#define BEEP_VELOCITY_SENSITIVITY 0.1


/*********************/
/* THE CLIMBING BEEP */
/*********************/
/* length of beep in vertical meters */ 
#define CLIMBING_BEEP_HIGH_LENGTH 0.5
#define CLIMBING_BEEP_LOW_LENGTH 0.5
#define CLIMBING_BEEP_LENGTH (CLIMBING_BEEP_HIGH_LENGTH + CLIMBING_BEEP_LOW_LENGTH)

/* climbing beep sound freq computation : BEEP_FREQ_COEFF * velocity + BEEP_BASE_FREQ */
#define CLIMBING_BEEP_BASE_FREQ 386.0
#define CLIMBING_BEEP_FREQ_COEFF 141.0

/* climbing beep velocity filter */
/* filteredVelocity = beepVelocity * BEEP_VELOCITY_FILTER_COEFF + BEEP_VELOCITY_FILTER_BASE */
#define CLIMBING_BEEP_VELOCITY_FILTER_BASE 1.62
#define CLIMBING_BEEP_VELOCITY_FILTER_COEFF 0.51

/********************/
/* THE SINKING BEEP */
/********************/
#define SINKING_BEEP_BASE_FREQ 386.0
#define SINKING_BEEP_FREQ_COEFF 33.0

/********************/
/* THE GLIDING BEEP */
/********************/
#define GLIDING_BEEP_HIGH_LENGTH 0.10
#define GLIDING_BEEP_LOW_LENGTH 1.40
#define GLIDING_BEEP_LENGTH (GLIDING_BEEP_HIGH_LENGTH + GLIDING_BEEP_LOW_LENGTH)

/**********************/
/* THE CLIMBING ALARM */
/**********************/
#define CLIMBING_ALARM_HIGH_LENGTH 0.10
#define CLIMBING_ALARM_LOW_LENGTH 0.30
#define CLIMBING_ALARM_LENGTH (CLIMBING_ALARM_HIGH_LENGTH + CLIMBING_ALARM_LOW_LENGTH)

#define  CLIMBING_ALARM_FREQ 1000.0

/*********************/
/* THE SINKING ALARM */
/*********************/
#define SINKING_ALARM_LENGTH 0.7

#define SINKING_ALARM_FREQ 100.0 


/**************/
/* beep state */
/**************/

#define bst_set(bit) beepState |= (1 << bit)
#define bst_unset(bit) beepState &= ~(1 << bit)
#define bst_isset(bit) (beepState & (1 << bit))

#define BEEP_HIGH 0
#define GLIDING_BEEP_ENABLED 1
#define GLIDING_ALARM_ENABLED 2
#define CLIMBING_ALARM 3
#define SINKING_ALARM 4
#define BEEP_NEW_FREQ 5

#define BEEP_TYPE_SILENT 0
#define BEEP_TYPE_SINKING 1
#define BEEP_TYPE_GLIDING 2
#define BEEP_TYPE_CLIMBING 3


/******************/
/* the main class */
/******************/

class beeper {

 public:
  beeper(double sinkingThreshold = BEEP_VELOCITY_DEFAULT_SINKING_THRESHOLD,
	 double climbingThreshold = BEEP_VELOCITY_DEFAULT_CLIMBING_THRESHOLD,
	 double nearClimbingSensitivity = BEEP_VELOCITY_DEFAULT_NEAR_CLIMBING_SENSITIVITY,
	 uint8_t volume = BEEP_DEFAULT_VOLUME);

  /* to stop beeper, set volume = 0 or set a very low and hight threshold. ex : -1000.0 and 1000.0 */
  void setThresholds(double sinkingThreshold = BEEP_VELOCITY_DEFAULT_SINKING_THRESHOLD,
		     double climblingThreshold = BEEP_VELOCITY_DEFAULT_CLIMBING_THRESHOLD,
		     double nearClimbingSensitivity = BEEP_VELOCITY_DEFAULT_NEAR_CLIMBING_SENSITIVITY);

  void setVolume(uint8_t newVolume = BEEP_DEFAULT_VOLUME);

  /* set near thermal alarm status */
  void setGlidingBeepState(boolean status);
  void setGlidingAlarmState(boolean status);

  /* run each time you get new velocity data */
  void setVelocity(double velocity);

  /* run as often as possible */
  void update();

 private:
  void setBeepParameters(double velocity);
  void setBeepPaternPosition(double velocity);
  void setTone();
  double beepSinkingThreshold;
  double beepGlidingThreshold;
  double beepClimbingThreshold;
  uint8_t volume;
  unsigned long beepStartTime;
  double beepVelocity;
  double beepFreq;
  double beepPaternBasePosition;
  double beepPaternPosition;
  uint8_t beepState;
  uint8_t beepType;
};


#endif
