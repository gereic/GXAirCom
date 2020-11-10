#include <Weather.h>

volatile uint32_t rainCount = 0;
volatile unsigned long rainDebounceTime; // Timer to avoid contact bounce in isr
volatile uint32_t aneometerpulsecount = 0;
volatile uint32_t actPulseCount = 0;
volatile uint8_t timerIrq = 0;
volatile unsigned long ContactBounceTime; // Timer to avoid contact bounce in isr

hw_timer_t * timer = NULL;

void IRAM_ATTR windspeedhandler(void){
  if((millis() - ContactBounceTime) > 15 ) { // debounce the switch contact.
    aneometerpulsecount++;
    ContactBounceTime = millis();
  }
}

void IRAM_ATTR rainhandler(void){
  if((millis() - rainDebounceTime) > 15 ) { // debounce the switch contact.
    rainCount++;
    rainDebounceTime = millis();
  }
}

void IRAM_ATTR onTimer() {
  actPulseCount = aneometerpulsecount;
  aneometerpulsecount = 0;
  timerIrq = 1;
}


Weather::Weather(){
}

bool Weather::initBME280(void){
  uint8_t error;
  sensorAdr = 0x76;
  bool ret = false;
  for (sensorAdr = 0x76; sensorAdr <= 0x77; sensorAdr++)
  {
    //log_i("check device at address 0x%X !",sensorAdr);
    pI2c->beginTransmission(sensorAdr);
    error = pI2c->endTransmission();
    if (error == 0){
      //log_i("I2C device found at address 0x%X !",sensorAdr);
      ret = bme.begin(sensorAdr,pI2c);
      //log_i("check sensor on adr 0x%X ret=%d",sensorAdr,ret);
      if (ret){
        //log_i("found sensor BME280 on adr 0x%X",sensorAdr);
        break;
      }
      
    }
  }
  
  if (!ret) return false;
  log_i("found sensor BME280 on adr 0x%X",sensorAdr);
  //sensor found --> set sampling
  bme.setSampling(Adafruit_BME280::MODE_FORCED, // mode
                  Adafruit_BME280::SAMPLING_X1, // temperature
                  Adafruit_BME280::SAMPLING_X1, // pressure
                  Adafruit_BME280::SAMPLING_X1, // humidity
                  Adafruit_BME280::FILTER_OFF,  //filter
                  Adafruit_BME280::STANDBY_MS_0_5);  //duration
  return true;
}


bool Weather::begin(TwoWire *pi2c, float height,int8_t oneWirePin, int8_t windDirPin, int8_t windSpeedPin,int8_t rainPin){
  pI2c = pi2c;
  _height = height;
  hasTempSensor = false;
  aneometerpulsecount = 0;
  log_i("onewire pin=%d",oneWirePin);
  if (oneWirePin >= 0){
    oneWire.begin(oneWirePin);
    sensors.setOneWire(&oneWire);
    sensors.begin();
    if (sensors.getAddress(tempSensorAdr, 0)){
      log_i("found onewire TempSensor with adr %X:%X:%X:%X:%X:%X:%X:%X ",tempSensorAdr[0],tempSensorAdr[1],tempSensorAdr[2],tempSensorAdr[3],tempSensorAdr[4],tempSensorAdr[5],tempSensorAdr[6],tempSensorAdr[7]);
      if (sensors.readPowerSupply(tempSensorAdr)){
        log_i("parasite powered");
      }else{
        log_i("normal powered");
      }
      sensors.setResolution(tempSensorAdr, 12);      
      sensors.setWaitForConversion(false); //we don't wait for conversion  
      hasTempSensor = true;
    }
  }
  _weather.bTemp = false;
  _weather.bHumidity = false;
  _weather.bPressure = false;
  if (!initBME280()){
      return false;
  }
  bme.readADCValues(); //we read adc-values 2 times and dismiss it
  delay(500);
  bme.readADCValues();
  delay(500); 
  avgFactor = 128; //factor for avg-factor 
  bFirst = false;

  //init-code for aneometer  
  _windDirPin = windDirPin;
  if (windDirPin >= 0){
    _weather.bWindDir = true;
    pinMode(_windDirPin, INPUT);
  }
  
  if (windSpeedPin >= 0){
    _weather.bWindSpeed = true;
    pinMode(windSpeedPin, INPUT);
    attachInterrupt(digitalPinToInterrupt(windSpeedPin), windspeedhandler, FALLING);
  }
  //rain-pin
  if (rainPin >= 0){
    _weather.bRain = true;
    pinMode(rainPin, INPUT);
    attachInterrupt(digitalPinToInterrupt(rainPin), rainhandler, FALLING);
  }
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 2250000, true); //every 2.25 seconds
  timerAlarmEnable(timer);
  return true;
}

void Weather::copyValues(void){
  _weather.bTemp = true;
  _weather.bHumidity = true;
  _weather.bPressure = true;
  _weather.temp = dTempOld;
  _weather.Humidity = dHumidityOld;
  _weather.Pressure = dPressureOld;
}

float Weather::calcPressure(float p, float t, float h){
  if (h > 0){
      //return ((float)p / pow(1-(0.0065*h/288.15),5.255));
      return ((float)p * pow(1-(0.0065*h/(t + (0.0065*h) + 273.15)),-5.257));
  }else{
      return p;
  }

}

void Weather::resetWindGust(void){
  windgust = 0; //reset windgust
}

void Weather::getValues(weatherData *weather){
  *weather = _weather;
}

float Weather::calcExpAvgf(float oldValue, float newValue, float Factor){
  if (Factor <= 0){
      return newValue;
  }else{
      return (newValue / Factor) + oldValue - (oldValue / Factor);
  }
}

void Weather::setTempOffset(float tempOffset){
  _tempOffset = tempOffset; //set temperature-offset
}

void Weather::runBME280(uint32_t tAct){
  static uint32_t tOld = millis();
	float temp = 0;
	float pressure = 0;
	float humidity = 0;
	float rawPressure = 0;
  if ((tAct - tOld) >= WEATHER_REFRESH){
    bme.readADCValues();
    temp = (float)bme.getTemp() / 100.; // in °C
    rawPressure = (float)bme.getPressure() / 100.;
    humidity = (float)(bme.getHumidity()/ 100.); // in %
    pressure = calcPressure(rawPressure,temp,_height); // in mbar
    if (hasTempSensor){
      //if (sensors.getAddress(tempSensorAdr, 0)){
      //  log_i("sensor found");      
      //}
      float actTemp = -127.0;
      for (int i = 0;i < 5;i++){
        if (sensors.isConnected(tempSensorAdr)){
          actTemp = sensors.getTempC(tempSensorAdr); //get temperature of sensor
          sensors.requestTemperatures(); //start next temperature-conversion
          break;
        }
      }
      //log_i("%d temp of sensor %.1f",tAct,actTemp);      
      if (actTemp > -100.0){
        //temp-conversion finished --> take measurement
        dTemp = actTemp;
      }
      
    }else{
      dTemp = temp;
    }
    dTemp += _tempOffset;
    dHumidity = humidity;
    dPressure = pressure;
    //log_i("T1=%f T2=%f h=%f p1=%f p2=%f",dTemp,temp,dHumidity,rawPressure,dPressure);
    if (!bFirst){
      dTempOld = dTemp;
      dHumidityOld = dHumidity;
      dPressureOld = dPressure;
      bFirst = true;
    }

    dTempOld = calcExpAvgf(dTempOld,dTemp,avgFactor);
    dHumidityOld = calcExpAvgf(dHumidityOld,dHumidity,avgFactor);
    dPressureOld = calcExpAvgf(dPressureOld,dPressure,avgFactor);
    copyValues();
    tOld = tAct;
  }

}

void Weather::setWindDirOffset(int16_t winddirOffset){
  _winddirOffset = winddirOffset;
}

float Weather::calcWindspeed(void){
  return (float)_actPulseCount * 1.609;
}

void Weather::checkAneometer(void){
  static bool bFirstWSpeed = false;
  static bool bFirstWDir = false;
  
  if (_weather.bWindDir){
    VaneValue = analogRead(_windDirPin);
    winddir = (map(VaneValue, 0, 1023, 0, 359) + _winddirOffset) % 360;
    if (!bFirstWDir){
      sinWinddir = sin(winddir * DEG2RAD);
      cosWinddir = cos(winddir * DEG2RAD);
      bFirstWDir = true;
    }
    sinWinddir = calcExpAvgf(sinWinddir,sin(winddir * DEG2RAD),avgFactor); //calc avg from sin
    cosWinddir = calcExpAvgf(cosWinddir,cos(winddir * DEG2RAD),avgFactor); //calc avg from cos
    winddirAvg = int16_t(atan2(sinWinddir,cosWinddir) * RAD2DEG * 10); //get deg back vom sin and cos-value
    while (winddirAvg < 0){
      winddirAvg += 3600;
    }
    while (winddirAvg > 3600){
      winddirAvg -= 3600;
    }
    _weather.WindDir = winddirAvg / 10.0;
  }
  if (_weather.bWindSpeed){
    if (timerIrq){
      _actPulseCount = actPulseCount;
      timerIrq = 0;
      float wSpeed = calcWindspeed();
      if (!bFirstWSpeed){
        windspeed = wSpeed;
        bFirstWSpeed = true;
      }
      if (wSpeed > windgust) windgust =  wSpeed;
      windspeed = calcExpAvgf(windspeed,wSpeed,avgFactor);
      //log_i("timer-interrupt pulse_count=%d %d",_actPulseCount,millis());
      //log_i("gust=%.1f speed=%.1f",_weather.WindGust,_weather.WindSpeed);
      _weather.WindSpeed = windspeed;
      _weather.WindGust = windgust;      
    }
  }
}

void Weather::checkRainSensor(void){
  time_t now;
  time(&now);
  //log_i("%04d-%02d-%02d %02d:%02d:%02d",year(now),month(now),day(now),hour(now),minute(now),second(now));
  if (actHour != hour(now)){
    rainTipCount1h = rainCount;
    actHour = hour(now);
    //log_i("hour changed");
  }
  if (actDay != day(now)){
    rainTipCount1d = rainCount;
    actDay = day(now);
    //log_i("day changed");
  }
  _weather.rain1h = float(rainCount - rainTipCount1h) * Bucket_Size;
  _weather.rain1d = float(rainCount - rainTipCount1d) * Bucket_Size;
  //log_i("count= %d rain1h=%.1f %d rain1d=%.1f %d",rainCount, _weather.rain1h,rainTipCount1h,_weather.rain1d,rainTipCount1d);
}

void Weather::run(void){
  uint32_t tAct = millis();
  runBME280(tAct);
  checkAneometer();
  checkRainSensor();
}