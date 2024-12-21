/*
Peet Bros. (PRO) anemometer interface.

Read the wind speed and direction interrupts from two input pins, do some math,
and output this

The anemometer itself is designed so that the two inputs does not signal
at the same time. We can use simple non-interruptable interrupts and still
be pretty sure not to lose any events.

Useful resources:
    https://github.com/guywithaview/peet-bros-wind/tree/master
    http://arduino.cc/en/Tutorial/DigitalPins
    http://learn.parallax.com/reed-switch-arduino-demo
    http://www.agrolan.co.il/UploadProductFiles/AWVPRO.pdf

http://www.peetbros.com/shop/item.aspx?itemid=137

*/

#include <PeetBros.h>


const unsigned long DEBOUNCE = 10000ul;      // Minimum switch time in microseconds
const unsigned long TIMEOUT = 1500000ul;       // Maximum time allowed between speed pulses in microseconds

// speed is actually stored as (km/h). Deviations below should match these units.
const int BAND_0 =  20;
const int BAND_1 =  150;

const int SPEED_DEV_LIMIT_0 =  10;     // Deviation from last measurement to be valid. Band_0: 0 to 20 kmh
const int SPEED_DEV_LIMIT_1 = 20;     // Deviation from last measurement to be valid. Band_1: 20 to 150 kmh
const int SPEED_DEV_LIMIT_2 = 55;     // Deviation from last measurement to be valid. Band_2: 150+ kmh

// Should be larger limits as lower speed, as the direction can change more per speed update
const int DIR_DEV_LIMIT_0 = 25;     // Deviation from last measurement to be valid. Band_0: 0 to 20 kmh
const int DIR_DEV_LIMIT_1 = 18;     // Deviation from last measurement to be valid. Band_1: 20 to 150 kmh
const int DIR_DEV_LIMIT_2 = 10;     // Deviation from last measurement to be valid. Band_2: 150+ kmh

volatile unsigned long speedPulse = 0ul;    // Time capture of speed pulse
volatile unsigned long dirPulse = 0ul;      // Time capture of direction pulse
volatile unsigned long speedTime = 0ul;     // Time between speed pulses (microseconds)
volatile unsigned long directionTime = 0ul; // Time between direction pulses (microseconds)
volatile boolean PB_newData = false;           // New speed pulse received
int8_t _wspeedPin = -1;
int8_t _wDirPin = -1;

void IRAM_ATTR isr_rotated();
void IRAM_ATTR isr_direction();
float PB_actSpeed;
float PB_actDir;
bool PB_valid;

void peetBros_init(int8_t windSpeedPin,int8_t windDirPin){
  PB_valid = false;
  PB_actSpeed = 0.0;
  PB_actDir = 0.0;
  _wspeedPin = windSpeedPin;
  _wDirPin = windDirPin;
  pinMode(windSpeedPin, INPUT);
  pinMode(windDirPin, INPUT);
  log_i("speedPin=%d,dirPin=%d",windSpeedPin,windDirPin);
  attachInterrupt(digitalPinToInterrupt(windSpeedPin), isr_rotated, FALLING);
  attachInterrupt(digitalPinToInterrupt(windDirPin), isr_direction, FALLING);
}

boolean checkDirDev(float kmh, float dev)
{
    if (kmh < BAND_0)
    {
        if ((abs(dev) < DIR_DEV_LIMIT_0) || (abs(dev) > 360 - DIR_DEV_LIMIT_0)) return true;
    }
    else if (kmh < BAND_1)
    {
        if ((abs(dev) < DIR_DEV_LIMIT_1) || (abs(dev) > 360 - DIR_DEV_LIMIT_1)) return true;
    }
    else
    {
        if ((abs(dev) < DIR_DEV_LIMIT_2) || (abs(dev) > 360 - DIR_DEV_LIMIT_2)) return true;
    }
    return false;
}

boolean checkSpeedDev(float kmh, float dev)
{
    if (kmh < BAND_0)
    {
        if (abs(dev) < SPEED_DEV_LIMIT_0) return true;
    }
    else if (kmh < BAND_1)
    {
        if (abs(dev) < SPEED_DEV_LIMIT_1) return true;
    }
    else
    {
        if (abs(dev) < SPEED_DEV_LIMIT_2) return true;
    }
    return false;
}

void peetBrosRun(){
  unsigned long dirPulse_, speedPulse_;
  unsigned long speedTime_;
  unsigned long directionTime_; 
  bool _newData;
  float windSpeed;
  float rps,kmh = 0,dev;
  static float prevKmh = 0;
  int16_t windDirection = 0;
  static int16_t prevDir = 0;

  noInterrupts();
  dirPulse_ = dirPulse;
  speedPulse_ = speedPulse;
  speedTime_ = speedTime;
  directionTime_ = directionTime;
  _newData = PB_newData;
  PB_newData = false;
  interrupts();


  if (_newData == false){
    //log_i("no new Data");
    return;
  }
  // Make speed zero, if the pulse delay is too long
  if (micros() - speedPulse_ > TIMEOUT){
    log_i("no rotation");
    PB_valid = false;
    return;
  }
  if (speedTime_ <= 0){   
    PB_valid = false;
    log_i("speedTime=0"); 
    return;
  } 
  rps = 1000000.0/double(speedTime_);
  if (rps < 0.010){
    PB_actSpeed = 0.0;
    PB_actDir = 0.0;
    PB_valid = false;
    return;
  }
  if (rps < 3.229){
    kmh = (-0.1095*(rps * rps) + 2.9318*rps - 0.1412) * 1.6094; 
  }else if (rps < 54.362){
    kmh = (0.0052*(rps * rps) + 2.1980*rps + 1.1091) * 1.6094;
  }else{
    kmh = (0.1104*(rps * rps) - 9.5685*rps + 329.87) * 1.6094;    
  }  
  dev = (kmh - prevKmh);
  prevKmh = kmh;
  //log_i("dt=%d,st=%d,speed=%.2f,rps=%.2f,dev=%.1f",directionTime_,speedTime_,PB_actSpeed,rps,dev);
  if (checkSpeedDev(kmh,dev) == false) return; //not valid speed change
  PB_actSpeed = kmh;    
  if (directionTime_ > speedTime_) return;
  // Calculate direction from captured pulse times
  windDirection = ((directionTime_ * 360) / speedTime_) % 360;  
  dev = float(windDirection - prevDir);  
  prevDir = windDirection;
  
  if (checkDirDev(kmh, dev) == false) return; //not valid dir change
  //log_i("dir=%d,speed=%.2f,rps=%.2ff",windDirection,PB_actSpeed,rps);
  PB_actDir = float(windDirection);
  PB_valid = true;
}

uint8_t peetBrosgetNewData(float *Dir, float *Speed){
  if (PB_valid == false) return 255;
  *Dir = PB_actDir;
  *Speed = PB_actSpeed;
  PB_valid = false;
  return 1;
}

/*
 * Interrupt called when the rotating part of the wind instrument
 * has completed one rotation.
 *
 * We can find the wind speed by calculating how long the complete
 * rotation took.
*/
void isr_rotated() {
  // Despite the interrupt being set to FALLING edge, double check the pin is now LOW
  if (((micros() - speedPulse) > DEBOUNCE) && (digitalRead(_wspeedPin) == LOW))
  {
      // Work out time difference between last pulse and now
      speedTime = micros() - speedPulse;
      // Direction pulse should have occured after the last speed pulse
      //if ((PB_newData = false) && (dirPulse - speedPulse >= 0)){
      if (dirPulse - speedPulse >= 0){
        directionTime = dirPulse - speedPulse;
        PB_newData = true;
      }
      speedPulse = micros();    // Capture time of the new speed pulse
  }
}

/*
 * After the rotation interrupt happened, the rotating section continues
 * into the next round. Somewhere within that rotation we get a different
 * interrupt when a certain point of the wind wane/direction indicator is
 * passed. By counting how far into the rotation (assumed to be == last rotation)
 * we can compute the angle.
 *
*/
void IRAM_ATTR isr_direction() { 
  if (((micros() - dirPulse) > DEBOUNCE) && (digitalRead(_wDirPin) == LOW))
  {
    dirPulse = micros();        // Capture time of direction pulse
  }
}

