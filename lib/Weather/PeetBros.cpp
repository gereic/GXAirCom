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


const uint64_t DEBOUNCE = 5000ul;      // Minimum switch time in microseconds

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
volatile uint32_t PB_actPulseCount = 0;
volatile uint8_t PB_timerIrq = 0;
hw_timer_t * pb_timer = NULL;

// Zwei GPIOs
int reedPins[] = {-1, -1}; //0 ... speed, 1 ... dir
const int numReeds = sizeof(reedPins) / sizeof(reedPins[0]);

struct wReedState {
  uint8_t stableLevel;
  uint64_t lastStableChangeUs;
  uint32_t pulseCount;
  bool     pulseFlag;
  uint64_t lastPulseUs;
  uint64_t pulseIntervalUs;
};

wReedState reeds[2];

void IRAM_ATTR handleReedInterrupt(int idx);
void IRAM_ATTR reedISR0() { handleReedInterrupt(0); }
void IRAM_ATTR reedISR1() { handleReedInterrupt(1); }

void IRAM_ATTR handleReedInterrupt(int idx) {
  const uint64_t now   = esp_timer_get_time();
  const int      level = digitalRead(reedPins[idx]); // GPIO lesen

  if (level == reeds[idx].stableLevel) return;

  const uint64_t dur = now - reeds[idx].lastStableChangeUs;
  const bool okHigh = (reeds[idx].stableLevel == HIGH) && (dur >= DEBOUNCE);
  const bool okLow  = (reeds[idx].stableLevel == LOW)  && (dur >= DEBOUNCE);

  if (okHigh || okLow) {
    reeds[idx].stableLevel = level;
    reeds[idx].lastStableChangeUs = now;

    if (level == LOW){
      reeds[idx].pulseCount++;
      if (idx == 0){ //speed-pin
        if (dirPulse >= speedPulse){
          speedTime = now - speedPulse;
          directionTime = dirPulse - speedPulse;
          PB_newData = true;
        }
        speedPulse = now;
      }else{
        dirPulse = now;
      }
      reeds[idx].pulseIntervalUs = now - reeds[idx].lastPulseUs;
      reeds[idx].lastPulseUs = now;
      reeds[idx].pulseFlag = true;      
    } 
  }
}

float PB_actSpeed;
float PB_actDir;
uint32_t tsPB_valid =  0;
uint8_t PB_valid = false;
volatile float dirOut = 0.0;      // Direction output in degrees

void IRAM_ATTR pb_onTimer() {
  if (PB_timerIrq) return;
  PB_actPulseCount = reeds[0].pulseCount;
  reeds[0].pulseCount = 0;
  PB_timerIrq = 1;
}

void peetBros_init(int8_t windSpeedPin,int8_t windDirPin){
  PB_valid = 0;
  PB_actSpeed = 0.0;
  PB_actDir = 0.0;
  reedPins[0] = windSpeedPin;
  reedPins[1] = windDirPin;
  log_i("speedPin=%d,dirPin=%d",windSpeedPin,windDirPin);
  // init of interrupts
  for (int i = 0; i < numReeds; i++) {
    pinMode(reedPins[i], INPUT);
    uint8_t level = digitalRead(reedPins[i]);
    reeds[i] = {level, (uint64_t)esp_timer_get_time(), 0, false, (uint64_t)esp_timer_get_time(), 0};
  }
  attachInterrupt(digitalPinToInterrupt(reedPins[0]), reedISR0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(reedPins[1]), reedISR1, CHANGE);

  pb_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(pb_timer, &pb_onTimer, true);
  timerAlarmWrite(pb_timer, 2000000, true); //every 2 seconds
  timerAlarmEnable(pb_timer);      
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
    //log_e("no valid dir change dev=%.0f;speed=%.2f",dev,kmh);
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
    //log_e("no valid speed change dev=%.0f;speed=%.2f",dev,kmh);
    return false;
}

float peetBrosWindSpeed(float rps){ 
  float mph;
  //log_i("rps=%.2f",rps);
  if (rps < 0.010){
    return 0.0;
  }  
  if (rps < 3.229){
    mph = -0.1095*(rps * rps) + (2.9318*rps) - 0.1412; 
  }else if (rps < 54.362){
    mph = 0.0052*(rps * rps) + (2.1980*rps) + 1.1091;
  }else{
    mph = 0.1104*(rps * rps) - (9.5685*rps) + 329.87;    
  } 
  //log_i("mph=%.2f",mph);   
  return mph * 1.6094; //change to kmh
}

void peetBrosGetWindDir(){
  unsigned long speedPulse_;
  unsigned long speedTime_;
  unsigned long directionTime_; 
  bool _newData;
  float rps,kmh = 0,dev;
  static float prevKmh = 0;
  int16_t windDirection = 0;
  static int16_t prevDir = 0;

  if (PB_newData == false){
    //log_i("no new Data");
    return;
  }
  speedTime_ = speedTime;
  directionTime_ = directionTime;
  _newData = PB_newData;
  PB_newData = false;

  if (speedTime_ <= 0){   
    log_e("speedTime=0"); 
    return;
  }
  if (directionTime_ > speedTime_){
    log_e("error wind-direction");
    return;
  } 
  // Calculate direction from captured pulse times
  windDirection = ((directionTime_ * 360) / speedTime_) % 360; 
  PB_actDir = float(windDirection); 
  //log_i("dt=%d,st=%d,wDir=%.0f",directionTime_/1000,speedTime_/1000,PB_actDir);
}

void peetBrosRun(){
  static uint32_t pulseCount = 0;
  static float timeDivider = 2.0;

  peetBrosGetWindDir();

  if (PB_timerIrq){
    tsPB_valid = millis();
    pulseCount += PB_actPulseCount;
    PB_timerIrq = 0;
    if ((pulseCount < 10) & (timeDivider < 10.0)){ //when we have low wind-speed, we count longer to be more precise.
      timeDivider += 2.0;
      return;
    }
    if (pulseCount == 0){
      PB_actSpeed = 0.0;
      PB_valid = 1;  
      timeDivider = 2.0;
      return;      
    }
    float rps = (float)pulseCount / timeDivider; //get rps
    PB_actSpeed =  peetBrosWindSpeed(rps);
    //log_i("cnt=%d,rps=%.2f,kmh=%.2f,td=%.2f",pulseCount,rps,PB_actSpeed,timeDivider);    
    PB_valid = 1;  
    timeDivider = 2.0;
    pulseCount = 0;
    //return;
  } 
}

uint8_t peetBrosgetNewData(float *Dir, float *Speed){
  //PB_valid 0 ... init, 1 .. new Data, 2.. no new data, 255 ... timeout
  if ((PB_valid != 255) && (timeOver(millis(),tsPB_valid,4000))){
    PB_valid = 255;
  }
  uint8_t ret = PB_valid;
  if (PB_valid == 255){
    *Speed = 0.0;
  }else if (PB_valid == 1){
    *Dir = PB_actDir;
    *Speed = PB_actSpeed;
    PB_valid = 2; //set to no new data yet
  }
  return ret;
}