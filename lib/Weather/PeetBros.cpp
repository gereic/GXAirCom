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
lastRotationAt

*/

#include <PeetBros.h>

#define PETTBROSDEBOUNCE 10

volatile unsigned int rotation_took0;
volatile unsigned int rotation_took1;
volatile signed int direction_latency0;
volatile signed int direction_latency1;
volatile unsigned long last_rotation_at = millis();
//volatile uint32_t dirCnt = 0;
//volatile uint32_t speedCnt = 0;
volatile unsigned long lastRotIsr = 0;
volatile unsigned long lastSpeedIsr = 0;

void IRAM_ATTR isr_rotated();
void IRAM_ATTR isr_direction();
bool valid_sensordata();
float wspeed_to_real();
float wdir_to_degrees();
void print_debug();

void peetBros_init(int8_t windSpeedPin,int8_t windDirPin){

  rotation_took0 = 0;
  rotation_took1 = 0;
  direction_latency0 = 0;
  direction_latency1 = 0;
  pinMode(windSpeedPin, INPUT);
  pinMode(windDirPin, INPUT);
  log_i("speedPin=%d,dirPin=%d",windSpeedPin,windDirPin);
  attachInterrupt(digitalPinToInterrupt(windSpeedPin), isr_rotated, RISING);
  attachInterrupt(digitalPinToInterrupt(windDirPin), isr_direction, RISING);
}

uint8_t peetBrosgetNewData(float *Dir, float *Speed){
  noInterrupts();
  if (!valid_sensordata()){
    interrupts();
    return(255);
  } 
  //Serial.printf("d=%d,s=%d\n",dirCnt,speedCnt);
  *Speed = wspeed_to_real(); // kmh
  *Dir = wdir_to_degrees(); //0-360 deg
  interrupts();
  if (isnan(*Speed) || isnan(*Dir)){
    return 255; //error not valid value
  }
  return 1;
}

void print_debug() {
  Serial.print("since last_rot: ");
  Serial.print(millis() - last_rotation_at); Serial.print("ms; ");
  Serial.print("last dur: ");
  Serial.print(rotation_took0); Serial.print("ms; ");
  Serial.print("wspeed: ");
  Serial.print(wspeed_to_real()); Serial.print("km/h; ");
  Serial.print("last dir_lat: ");
  Serial.print(direction_latency1); Serial.print("ms; ");
  Serial.print("wdir: ");
  Serial.print(wdir_to_degrees()); Serial.print(" degrees; ");
  Serial.println();
}


bool valid_sensordata() {
  // XXX: wrapping timers?
  if ((last_rotation_at + 10*1000 < millis()) || (direction_latency1 < 0)) return(false);
  else return(true);
}

float wspeed_to_real() {
  float mph = NAN;

  //if (!valid_sensordata()) return(NAN);

  // avoid rewriting documented formulas.
  float r0 = 1.0 / (rotation_took0 / 1000.0);
  float r1 = 1.0 / (rotation_took1 / 1000.0);

  if (r0 < 0.010) mph = 0.0;
  else if (r0 < 3.229) mph = -0.1095*r1 + 2.9318*r0 - 0.1412;
  else if (r0 < 54.362) mph = 0.0052*r1 + 2.1980*r0 + 1.1091;
  else if (r0 < 66.332) mph = 0.1104*r1 - 9.5685*r0 + 329.87;

  if (isinf(mph) || isnan(mph) || mph < 0.0) return(NAN);

  //float meters_per_second = mph * 0.48037;
  //float knots = mph * 0.86897;
  float kmh = mph * 1.60934;
  return(kmh);
}

float wdir_to_degrees() {
  float windangle;

  //if (!valid_sensordata()) return(NAN);
  //if (direction_latency1 < 0) return(NAN);

  float avg_rotation_time = ((float(rotation_took0) + float(rotation_took1)) / 2.0);

  float phaseshift = float(direction_latency1) / avg_rotation_time;

  if (isnan(phaseshift) || isinf(phaseshift)) windangle = NAN;
  else if (phaseshift == 0.0) windangle = 360.0;
  else if (phaseshift > 0.99) windangle = 360.0;
  else windangle = 360.0 * phaseshift;

  //Serial.printf("%.2fms; %.2fps; %.2fdeg\n",avg_rotation_time,phaseshift,windangle);
  return(windangle);
}

/*
 * Interrupt called when the rotating part of the wind instrument
 * has completed one rotation.
 *
 * We can find the wind speed by calculating how long the complete
 * rotation took.
*/
void isr_rotated() {
  //unsigned long diff = lastSpeedIsr - millis();
  //if (diff < PETTBROSDEBOUNCE) return; //debounce
  //lastSpeedIsr = millis();
  unsigned long now = millis();
  unsigned int last_rotation_took;
  //speedCnt++;
  // Handle wrapping counters.
  if (now < last_rotation_at)
    last_rotation_took = now + (ULONG_MAX - last_rotation_at);
  else
    last_rotation_took = now - last_rotation_at;

  // I'd love to log this somewhere, but no Serial inside an ISR.
  if (last_rotation_took < 0)
    last_rotation_took = 0;

  last_rotation_at = now;

  rotation_took1 = rotation_took0;
  rotation_took0 = last_rotation_took;

  direction_latency1 = direction_latency0;
  direction_latency0 = -1;
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
  //unsigned long diff = lastRotIsr - millis();
  //if (diff < PETTBROSDEBOUNCE) return; //debounce
  //lastRotIsr = millis();
  unsigned long now = millis();
  unsigned int direction_latency;
  //dirCnt++;
  if (now < last_rotation_at)
    direction_latency = now + (ULONG_MAX - last_rotation_at);
  else
    direction_latency = now - last_rotation_at;

  direction_latency0 = direction_latency;
}

