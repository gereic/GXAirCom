/*
 * tx20.c
 *
 *  Created on: 20.06.2018
 *      Author: gereic
 * Pin	Color	Description
 * 1	Brown	TxD
 * 2	Red	Vcc
 * 3	Green	DTR
 * 4	Yellow	GND
 */

#include <Tx20.h>


#ifdef TX20DEBUG
  #define LEDPIN 25
#endif

volatile uint8_t TX20IncomingData = 0;
volatile uint8_t Tx20BitCount = 0;
volatile uint16_t tTime1 = 0;

volatile uint8_t chk;
volatile uint8_t sa,sb,sd,se;
volatile uint16_t sc,sf, pin;
volatile uint8_t DataReady = 0;
volatile uint8_t DataError = 1;
volatile uint8_t newData = 0;
volatile uint8_t windDir = 0; //[0-15] (0=N)
volatile uint16_t windSpeed = 0; //[0.1m/s]

int8_t tx20DataPin = -1;
hw_timer_t * tx20timer = NULL;

/* function prototypes */
void IRAM_ATTR irqTx20Rising(void);
void IRAM_ATTR IrqTx20Timer(void);
void IRAM_ATTR readDataPin(void);

/* interrupt for receiving-timer */
void IRAM_ATTR IrqTx20Timer(void){
  if (TX20IncomingData == 2){
		//if (Tx20BitCount == 41) timerRestart(tx20timer); //set counter-value to 0
    if (Tx20BitCount == 41){
      timerWrite(tx20timer,0);//reset timer
    }
    readDataPin();
		Tx20BitCount --;
	}else if (TX20IncomingData >= 10){
    //data ready --> wait another 1,2s before start reading next telegram
		tTime1++;
		if (tTime1 >= 1000){
			TX20IncomingData = 1;
		}
	}
}


void IRAM_ATTR irqTx20Rising(){
  if (TX20IncomingData == 1) {
    #ifdef TX20DEBUG
    digitalWrite(LEDPIN,LOW);
    #endif
    Tx20BitCount = 41;
    sa=sb=sd=se=0;
    sc=0;sf=0;
    DataReady = 0;
    TX20IncomingData = 2;
    timerStop(tx20timer); //stop timer  
    timerWrite(tx20timer,500); //set start-value to be in middle of signal
    timerStart(tx20timer);
	}
}

void IRAM_ATTR readDataPin(void){
  #ifdef TX20DEBUG
  digitalWrite(LEDPIN,!digitalRead(LEDPIN));
  #endif
  pin = (digitalRead(tx20DataPin));
  if (Tx20BitCount > 41-5){
    // start, inverted
    sa = (sa<<1)|(pin^1);
  } else
  if (Tx20BitCount > 41-5-4){
    // wind dir, inverted
    sb = sb>>1 | ((pin^1)<<3);
  } else
  if (Tx20BitCount > 41-5-4-12){
    // windspeed, inverted
    sc = sc>>1 | ((pin^1)<<11);
  } else
  if (Tx20BitCount > 41-5-4-12-4){
    // checksum, inverted
    sd = sd>>1 | ((pin^1)<<3);
  } else
  if (Tx20BitCount > 41-5-4-12-4-4){
    // wind dir
    se = se>>1 | (pin<<3);
  } else {
    // windspeed
    sf = sf>>1 | (pin<<11);
  }
  if (Tx20BitCount <= 1){
    chk= ( sb + (sc&0xf) + ((sc>>4)&0xf) + ((sc>>8)&0xf) );chk&=0xf;
    if (sa==4 && sb==se && sc==sf && sd==chk){
      windDir = sb;
      windSpeed = sc;
      DataError = 0;
      TX20IncomingData = 10; //data Ready
      newData = 1;
    } else {
      TX20IncomingData = 20; //data Error
      DataError = 1;
      newData = 2;
      
    }
    
    tTime1 = 0;
    DataReady = 1;
  }

}

void tx20_init(int8_t dataPin){
	#ifdef TX20DEBUG
  pinMode(LEDPIN,OUTPUT);
  #endif
  tx20DataPin = dataPin;
  pinMode(tx20DataPin, INPUT);
	TX20IncomingData = 1;
	newData = 0;
	DataReady = 1;
  tx20timer = timerBegin(0, 80, true);
  timerAttachInterrupt(tx20timer, &IrqTx20Timer, true);
  timerAlarmWrite(tx20timer, 1220, true); //every 1.20 mseconds
  timerAlarmEnable(tx20timer);  
  attachInterrupt(digitalPinToInterrupt(tx20DataPin), irqTx20Rising, RISING);
}

uint8_t tx20getNewData(uint8_t *Dir, uint16_t *Speed){
  *Dir = windDir;
  *Speed = windSpeed;
  uint8_t retData = newData;
  newData = 0; //clear new Data
  return retData;
}
