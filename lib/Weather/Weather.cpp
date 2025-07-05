#include <Weather.h>


volatile uint32_t rainCount = 0;
volatile uint32_t rainDebounceTime; // Timer to avoid contact bounce in isr
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
  uint32_t tAct = millis();
  if((tAct - rainDebounceTime) > 2000uL ) { // debounce the switch contact.
    rainCount++;
    rainDebounceTime = tAct;
  }
}

void IRAM_ATTR onTimer() {
  actPulseCount = aneometerpulsecount;
  aneometerpulsecount = 0;
  timerIrq = 1;
}


Weather::Weather(){
}

bool Weather::checkI2Caddr(uint8_t i2cAddr) {
  uint8_t error;
  log_i("check device at address 0x%X !",i2cAddr);
  pI2c->beginTransmission(i2cAddr);
  error = pI2c->endTransmission();
  return (error == 0);
}

bool Weather::initBME280(void){
  uint8_t sensorAdr = 0x76;
  bool ret = false;
  for (sensorAdr = 0x76; sensorAdr <= 0x77; sensorAdr++)
  {
    if (checkI2Caddr(sensorAdr)){
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
  bme.readADCValues(); //we read adc-values 2 times and dismiss it
  delay(50);
  bme.readADCValues();
  return true;
}

bool Weather::initADS(AnemometerSettings &anSettings) {
  uint8_t adsAddr = 0x48;
  bool ret = false;
  for (adsAddr = 0x48; adsAddr <= 0x49; adsAddr++) {
    if (checkI2Caddr(adsAddr)) {
      _ADS1015 = ADS1015(adsAddr, pI2c);
      ret = _ADS1015.isConnected();
      if (ret) { break; }
    }
  }
  if (!ret) return false;
  log_i("found ADS1015 on adr 0x%X",adsAddr);
  _ADS1015.setMode(1);
  _ADS1015.readADC(0);
  _ADS1015.setGain(anSettings.AnemometerAdsGain);
  _ADS1015.readADC(0);
  _ADS1015.setDataRate(4);  // 7 is fastest, but more noise
  _ADS1015.readADC(0);
  return true;
}

bool Weather::begin(TwoWire *pi2c, SettingsData &setting, int8_t oneWirePin, int8_t windDirPin, int8_t windSpeedPin,int8_t rainPin){
  bool bRet = true;
  pI2c = pi2c;
  _height = setting.gs.alt;
  hasTempSensor = false;
  aneometerpulsecount = 0;
  bNewWeather = false;
  actHour = 0;
  actDay = 0;
  _bHasBME = setting.wd.mode.bits.hasBME;
  _bHasDS18B20 = setting.wd.mode.bits.ds18B20;
  _bHasSHT20 = setting.wd.mode.bits.SHT20;
  anSettings = setting.wd.anemometer;
  aneometerType = anSettings.AnemometerType;


  if (_bHasSHT20){
    sht = new SHT2x(pI2c);
    if (sht->begin()){
      uint8_t stat = sht->getStatus();
      log_i("found SHT20-Sensor stat=0X%02X",stat);
    }else{
      log_e("sht20 not connected");
    }
  }
  //log_i("onewire pin=%d",oneWirePin);
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
      sensors.requestTemperatures();//start first conversion
      hasTempSensor = true;
    }
  }
  _weather.bTemp = false;
  _weather.bHumidity = false;
  _weather.bPressure = false;

  if (_bHasBME){
    if (initBME280()){
    }else{
      log_i("no BME280 found");
      _bHasBME = false;
      bRet = false;
    }
  }
  //avgFactor = 128; //factor for avg-factor 
  bFirst = false;
  _weather.bWindDir = false; 
  _weather.bWindSpeed = false;
  if (aneometerType == eAnemometer::TX20){
    _weather.bWindSpeed = true;
    _weather.bWindDir = true;
    tx20_init(windSpeedPin);
  } else if (aneometerType == eAnemometer::ADS_A1015){
    if (initADS(anSettings)) {
      _weather.bWindDir = anSettings.AnemometerAdsWDirMaxVoltage != anSettings.AnemometerAdsWDirMinVoltage;
      _weather.bWindSpeed = anSettings.AnemometerAdsWSpeedMaxVoltage != anSettings.AnemometerAdsWSpeedMinVoltage;
      _bHasADS = true;
    }else{
      _bHasADS = false;
      log_i("no ADS1015 found");
    }
  } else if (aneometerType == eAnemometer::PEETBROS){
    _weather.bWindSpeed = true;
    _weather.bWindDir = true;    
    peetBros_init(windSpeedPin,windDirPin);
  } else if (aneometerType == eAnemometer::MISOL){
    //init-code for aneometer Misol
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
    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer, 2000000, true); //every 2 seconds
    timerAlarmEnable(timer);
  }
    #ifdef WS85_in
   else if (aneometerType == WS85) {
      ws85_init(windDirPin);  // RX pin used for WS85
      _weather.bWindSpeed = true;
      _weather.bWindDir = true;
      _weather.bTemp = true;
    }
    #endif
   else{
    //init-code for aneometer DAVIS6410
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
    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer, 2250000, true); //every 2.25 seconds
    timerAlarmEnable(timer);
  }
  //rain-pin
  if (rainPin >= 0){
    _weather.bRain = true;
    pinMode(rainPin, INPUT);
    rainDebounceTime = millis();
    attachInterrupt(digitalPinToInterrupt(rainPin), rainhandler, FALLING);
  }else{
    _weather.bRain = false;
  }
  return bRet;
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

bool Weather::getValues(weatherData *weather){
  *weather = _weather;
  bool bRet = bNewWeather;
  bNewWeather = false;
  return bRet;
}

void Weather::setTempOffset(float tempOffset){
  _tempOffset = tempOffset; //set temperature-offset
}

void Weather::setWindDirOffset(int16_t winddirOffset){
  _winddirOffset = winddirOffset;
}

float Weather::calcWindspeed(void){
  if (aneometerType == eAnemometer::MISOL){
    // 2.4km/h = 1 impuls per second 
    // --> 1.2km/h = 1 impuls in 2 seconds
    return (float)_actPulseCount * 1.2;
  }else{
    return (float)_actPulseCount * 1.609;
  }
}

uint8_t getMilosDirection(uint16_t vaneValue){
  uint16_t dirValues[16] = {793,400,458,76,85,58,176,117,278,235,629,599,995,835,910,702};
  uint8_t actIndex = 0;
  uint16_t maxOffset = 0xFFFF;
  if (vaneValue == 1023) return 0; //no wind-vane connected
  for (int i = 0;i < 16;i++){
    uint16_t actOffset = abs((int16_t)vaneValue - (int16_t)dirValues[i]);
    if (actOffset < maxOffset){
      maxOffset = actOffset;
      actIndex = i;
    }
  }
  return actIndex;
}

void Weather::checkAneometer(void){
  if (_weather.bWindDir){
    //VaneValue = analogRead(_windDirPin);
    analogRead(_windDirPin);//skip first measurement
    uint32_t adc_reading = 0;
    for (int i = 0; i < 4; i++) { //make 4 readings and create avg
      uint16_t thisReading = analogRead(_windDirPin);
      adc_reading += thisReading;
    }    
    VaneValue = adc_reading / 4;
    if (aneometerType == eAnemometer::MISOL){      
      //R2 = (Uout * R1)/(Uin-Uout)
      float Uout = (float)VaneValue * 3.3 / 1023;
      uint32_t resistor = uint32_t((Uout * 10000)/(3.3-Uout));
      winddir = (float)((((int16_t)getMilosDirection(VaneValue) * 225) + (_winddirOffset * 10)) % 3600) / 10.0;
      //log_i("analog-value of wind-vane:%d;resistor=%d;dir=%.2f",VaneValue,resistor,winddir);
      _weather.vaneValue = VaneValue;
      _weather.WindDir = winddir;  

    }else{
      _weather.vaneValue = VaneValue;
      winddir = (map(VaneValue, 0, 1023, 0, 359) + _winddirOffset) % 360;
      _weather.WindDir = winddir;  
    }
  }
  if (_weather.bWindSpeed){
    if (timerIrq){
      _actPulseCount = actPulseCount;
      timerIrq = 0;
      float wSpeed = calcWindspeed();
      _weather.WindSpeed = wSpeed;
      if (wSpeed > windgust) windgust =  wSpeed;
      _weather.WindGust = windgust;      
    }
  }
}

float Weather::getAdsVoltage(uint8_t pin, float vref) {
  float voltage, vdiv_r1, vdiv_r2;
  vdiv_r1 = anSettings.AnemometerAdsVDivR1;
  vdiv_r2 = anSettings.AnemometerAdsVDivR2;
  voltage = _ADS1015.toVoltage(_ADS1015.readADC(pin));
  voltage -= vref; // voltage divider is between in and vref
  voltage = voltage * ((vdiv_r1 + vdiv_r2) / vdiv_r2);
  voltage += vref; // add reference voltage again
  log_e("measured_voltage: %f",voltage);
  return voltage;
}

float Weather::calcAdsMeasurement(float measurement, float minVoltage, float maxVoltage, float minRange, float maxRange) {
  // use mV
  if ((measurement < minVoltage)) {
    return 0.0;
  }
  if (minVoltage == maxVoltage) {
    // avoid division by zero.
    return 0.0;
  }
  float minVoltageMV = 1000 * minVoltage;
  float maxVoltageMV = 1000 * maxVoltage;
  float measurementMV = 1000 * measurement;
  float range = maxRange - minRange;
  float rangePart = range / (maxVoltageMV - minVoltageMV);

  return (minRange + (rangePart * (measurementMV - minVoltageMV))); 
}

void Weather::checkAdsAneometer(void) {
  uint8_t speed_pin = (uint8_t) eAdsAneometerPin::ADS_WINDSPEED_PIN;
  uint8_t dir_pin = (uint8_t) eAdsAneometerPin::ADS_WINDDIR_PIN;
  uint8_t vref_pin = (uint8_t) eAdsAneometerPin::ADS_VREF_PIN;
  float vref = _ADS1015.toVoltage(_ADS1015.readADC(vref_pin));
  float measurement;
  if (_weather.bWindSpeed){
    measurement = getAdsVoltage(speed_pin, vref);
    measurement = calcAdsMeasurement(
      measurement,
      anSettings.AnemometerAdsWSpeedMinVoltage,
      anSettings.AnemometerAdsWSpeedMaxVoltage,
      anSettings.AnemometerAdsWSpeedMinSpeed,
      anSettings.AnemometerAdsWSpeedMaxSpeed
    );
    _weather.WindSpeed = measurement;
  }
  if (_weather.bWindDir){
    measurement = getAdsVoltage(dir_pin, vref);
    measurement = calcAdsMeasurement(
      measurement,
      anSettings.AnemometerAdsWDirMinVoltage,
      anSettings.AnemometerAdsWDirMaxVoltage,
      anSettings.AnemometerAdsWDirMinDir,
      anSettings.AnemometerAdsWDirMaxDir
    );
    measurement += _winddirOffset;
    // don't use fmod for a smaller footprint
    while (measurement < 0) { measurement += 360.0; }
    while (measurement > 360) { measurement -= 360.0; }
    _weather.WindDir = measurement;
  }
}

void Weather::checkRainSensor(void){
  // Skip tipping bucket logic if WS85 is the anemometer
  if (aneometerType == eAnemometer::WS85) return;
  time_t now;
  std::time(&now);
  //log_i("%04d-%02d-%02d %02d:%02d:%02d",year(now),month(now),day(now),hour(now),minute(now),second(now));
  uint8_t u8Hour = hour(now);
  if (actHour != u8Hour){
    rainTipCount1h = rainCount;
    //log_i("hour changed %d->%d %d",actHour,u8Hour,rainTipCount1h);
    actHour = u8Hour;
  }
  uint8_t u8Day = day(now);
  if (actDay != u8Day){
    rainTipCount1d = rainCount;
    //log_i("day changed %d->%d %d",actDay,u8Day,rainTipCount1d);
    actDay = u8Day;
  }
  _weather.rain1h = float(rainCount - rainTipCount1h) * Bucket_Size;
  _weather.rain1d = float(rainCount - rainTipCount1d) * Bucket_Size;
  //log_i("count= %d rain1h=%.1f %d rain1d=%.1f %d",rainCount, _weather.rain1h,rainTipCount1h,_weather.rain1d,rainTipCount1d);
}

void Weather::run(void){
  uint32_t tAct = millis();
  static uint32_t tOld = millis();
	float temp = 0;
	float rawPressure = 0;
  bool bReadOk = false;
  if (aneometerType == eAnemometer::PEETBROS){
    peetBrosRun();
  }
  if ((tAct - tOld) >= WEATHER_REFRESH){
    int i = 0;
    uint8_t bmeRet = 0;
    _weather.bTemp = false;
    _weather.bHumidity = false;
    _weather.bPressure = false;
    if (_bHasBME){ //BME280 sensor
      for (i = 0;i < 5;i++){
        bmeRet = bme.readADCValues();
        if (bmeRet == 0){
          if (!hasTempSensor){
            _weather.temp = ((float)bme.getTemp() / 100.) + _tempOffset; // in °C
            _weather.bTemp = true;
          }
          rawPressure = (float)bme.getPressure() / 100.;
          _weather.bHumidity = true;
          _weather.Humidity = (float)(bme.getHumidity()/ 100.); // in %
          _weather.bPressure = true;
          _weather.Pressure = calcPressure(rawPressure,temp,_height); // in mbar
          bReadOk = true; //we got the values !!            
          break; //ready with reading
        }
        delay(500);
        log_e("error reading bme280 %d",i);
      }
      if (!bReadOk){
        log_e("error reading bme280 ret=%d",bmeRet);
        initBME280(); //try to reinit BME
      }
    }
    if (hasTempSensor){ //one-wire Temp-sensor
      float actTemp = -127.0;
      bReadOk = false;
      for (int i = 0;i < 3;i++){
        if (sensors.isConnected(tempSensorAdr)){
          actTemp = sensors.getTempC(tempSensorAdr); //get temperature of sensor          
          sensors.requestTemperatures(); //start next temperature-conversion
          if (actTemp > -100.0){
            _weather.temp = actTemp + _tempOffset;
            _weather.bTemp = true;
            bReadOk = true;
            break;
          }else{
            delay(sensors.millisToWaitForConversion(sensors.getResolution()));
          }          
        }else{
          delay(100); //wait 100ms
        }
      }
      if (!bReadOk){
        log_e("error reading oneWire");
      }
    }
    if (_bHasSHT20){
      for (i = 0;i < 3;i++){
        if (sht->read()){
          _weather.temp = sht->getTemperature() + _tempOffset; // in °C
          _weather.bTemp = true;
          _weather.bHumidity = true;
          _weather.Humidity = sht->getHumidity(); // in %
          //log_i("t=%.2f;hum=%.2f",_weather.temp,_weather.Humidity);  
          break;
        }
        log_e("error reading sht20 %d",i);        
        delay(500);
      } 
    }
    if (aneometerType == eAnemometer::TX20){
      uint8_t Dir;
      uint16_t Speed;
      uint8_t ret = tx20getNewData(&Dir,&Speed);
      if (ret == 1){
        _weather.vaneValue = int16_t(float(Dir) * 22.5);
        _weather.WindDir = (float(Dir) * 22.5) + _winddirOffset;
        _weather.WindSpeed = float(Speed) / 10.0 * 3.6; //[1/10m/s] --> [km/h]
        if (_weather.WindSpeed > _weather.WindGust) _weather.WindGust = _weather.WindSpeed; 
      }
    } else if (aneometerType == eAnemometer::ADS_A1015 && _bHasADS) {
      checkAdsAneometer();
    } else if (aneometerType == eAnemometer::PEETBROS) {
      uint8_t ret = peetBrosgetNewData(&_weather.WindDir,&_weather.WindSpeed);
      if (ret == 1){
        _weather.WindDir += _winddirOffset; //we got new data --> add winddir-offset
      }      
      if (_weather.WindSpeed > _weather.WindGust) _weather.WindGust = _weather.WindSpeed; 
      //log_i("dir=%.1f,speed=%0.1f,ret=%d",_weather.WindDir,_weather.WindSpeed,ret);
    } else if (aneometerType == eAnemometer::MISOL){
      checkAneometer();
    } else if (aneometerType == eAnemometer::WS85) {
      #ifdef WS85_in
      //ws85_run(); // Now called in main loop more frequently
      float dir, speed, gust, temp, rain1h, batVolt, capVolt;
      if (ws85_getData(&dir, &speed, &gust, &temp, &rain1h, &batVolt, &capVolt)) {
        _weather.WindDir = dir;
        _weather.WindSpeed = speed * 3.6;
        if (gust * 3.6 > _weather.WindGust) _weather.WindGust = gust * 3.6;
        if(temp > -30.0 && temp < 60.0) { // Check for valid tempreature
          _weather.temp = temp + _tempOffset; // in °C
          _weather.bTemp = true;
        }
        _weather.rain1h = rain1h;
        _weather.bRain = true;

        log_i("WS85: WindDir=%.1f deg, WindSpeed=%.2f km/h, WindGust=%.2f km/h", dir, speed * 3.6, gust * 3.6);
        log_i("WS85: Temp=%.2f C, Rain1h=%.2f mm", temp, rain1h);
        log_i("WS85: BatVoltage=%.2f V, CapVoltage=%.2f V", batVolt, capVolt);
      }
      #endif
    } else {
      checkAneometer();
    }
    while (_weather.WindDir < 0) { _weather.WindDir += 360.0; }
    while (_weather.WindDir >= 360) { _weather.WindDir -= 360.0; }    
    checkRainSensor();
    bNewWeather = true;
    tOld = tAct;
  }
}