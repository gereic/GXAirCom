/*
Peet Bros. (PRO) anemometer interface.

Read the wind speed and direction interrupts from two input pins, do some math,
and output this

The anemometer itself is designed so that the two inputs does not signal
at the same time. We can use simple non-interruptable interrupts and still
be pretty sure not to lose any events.

Useful resources:
    http://arduino.cc/en/Tutorial/DigitalPins
    http://learn.parallax.com/reed-switch-arduino-demo
    http://www.agrolan.co.il/UploadProductFiles/AWVPRO.pdf

http://www.peetbros.com/shop/item.aspx?itemid=137

Connectors used:
* GND - the two ground wires towards the anemometer.
* pin 2 - rotating pin on anemometer.
* pin 3 - direction pin on anemometer.

Remaining:
* power saving, it uses ~40mA on an Uno now.

*/

#include <PeetBros.h>

volatile uint32_t rotation_took0;
volatile uint32_t rotation_took1;
volatile uint32_t directionTimeStamp;
volatile uint32_t direction_latency0;
volatile uint32_t direction_latency1;
volatile uint32_t last_rotation_at = millis();
uint32_t last_report = millis();
volatile uint8_t cntDir = 0;
volatile uint8_t cntSpeed = 0;
float PeetBrosDir = 0.0;

void isr_rotated();
void isr_direction();
bool valid_sensordata();
float wspeed_to_real();
float wdir_to_degrees();

void peetBros_init(int8_t windSpeedPin,int8_t windDirPin){

  rotation_took0 = 0;
  rotation_took1 = 0;
  direction_latency0 = 0;
  direction_latency1 = 0;
  last_rotation_at = 0;
  pinMode(windSpeedPin, INPUT);
  pinMode(windDirPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(windSpeedPin), isr_rotated, FALLING);
  attachInterrupt(digitalPinToInterrupt(windDirPin), isr_direction, FALLING);
}

uint8_t peetBrosgetNewData(float *Dir, float *Speed){
  /*
  if ((!valid_sensordata) || (direction_latency0 < 0)){
    return 255;
  }
  */
  *Speed = wspeed_to_real(); // kmh
  *Dir = wdir_to_degrees(); //0-360 deg
  if (isnan(*Speed) || isnan(*Dir)){
    return 255; //error not valid value
  }
  return 1;
}



bool valid_sensordata() {
  // XXX: wrapping timers?
  if (last_rotation_at + 10*1000 < millis()) return(false);
  else return(true);
}

float wspeed_to_real() {
  float mph = NAN;

  if (!valid_sensordata()) return(0.0);

  float r0 = 1.0 / (float(rotation_took1) / 1000.0);
  float r1 = r0 * r0;

  if (r0 < 0.010){
    mph = 0.0;
  } 
  else if (r0 < 3.229){
    mph = -0.1095*r1 + 2.9318*r0 - 0.1412;
  } else if (r0 < 54.362){
    mph = 0.0052*r1 + 2.1980*r0 + 1.1091;
  } else if (r0 < 66.332){
    mph = 0.1104*r1 - 9.5685*r0 + 329.87;
  } 

  if (isinf(mph) || isnan(mph) || mph < 0.0) return(0.0);

  //float meters_per_second = mph * 0.48037;
  //float knots = mph * 0.86897;
  float kmh = mph * 1.60934;
  return(kmh);
}

float wdir_to_degrees() {
  float windangle;

  //NORTH (both reeds close simultaneously) 
  //SOUTH (contact closures are 180° out of phase)  
  if (!valid_sensordata()) return(NAN);
  if (direction_latency1 < 0) return(NAN);
  float phaseshift = 0.0;
  if (direction_latency1 <= rotation_took1){
    phaseshift = float(direction_latency1) / float(rotation_took1);
  }else{
    log_e("lat > rot; %d > %d",direction_latency1,rotation_took1);
    return PeetBrosDir; //return last direction
  }
  //log_i("lat=%d,took=%d,phase=%.2f",direction_latency1,rotation_took1,phaseshift);
  if (isnan(phaseshift) || isinf(phaseshift)) windangle = NAN;
  else if (phaseshift == 0.0) windangle = 360.0;
  else if (phaseshift > 0.99) windangle = 360.0;
  else windangle = 360.0 * phaseshift;
  PeetBrosDir = windangle;
  return(PeetBrosDir);
}

/*
 * Interrupt called when the rotating part of the wind instrument
 * has completed one rotation.
 *
 * We can find the wind speed by calculating how long the complete
 * rotation took.
*/
void isr_rotated() {
  uint32_t now = millis();
  uint32_t last_rotation_took;
  if (cntDir != 1){ //check if switch for direction was also on for exactly 1 time    
    cntDir = 0;
    return;
  }
  cntDir = 0;

  last_rotation_took = now - last_rotation_at;

  // spurious interrupt? ignore it.
  // (these are probably an artifact of the push button used for development)
  if (last_rotation_took < 2) return;

  rotation_took1 = last_rotation_took;
  direction_latency1 = directionTimeStamp - last_rotation_at;
  last_rotation_at = now;

  /*
  rotation_took1 = rotation_took0;
  rotation_took0 = last_rotation_took;
  direction_latency1 = direction_latency0;
  direction_latency0 = 0;
  */
}

/*
 * After the rotation interrupt happened, the rotating section continues
 * into the next round. Somewhere within that rotation we get a different
 * interrupt when a certain point of the wind wane/direction indicator is
 * passed. By counting how far into the rotation (assumed to be == last rotation)
 * we can compute the angle.
 *
*/
void isr_direction() {  
  //uint32_t now = millis();
  directionTimeStamp = millis();
  //direction_latency0 = now - last_rotation_at;
  cntDir += 1; //count direction
}

