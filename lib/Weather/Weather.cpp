#include <Weather.h>

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


bool Weather::begin(TwoWire *pi2c, float height,uint8_t oneWirePin){
  pI2c = pi2c;
  _height = height;
  hasTempSensor = false;
  log_i("onewire pin=%d",oneWirePin);
  if (oneWirePin >= 0){
    oneWire.begin(oneWirePin);
    sensors.setOneWire(&oneWire);
    sensors.begin();
    oneWire.reset_search();
    if (oneWire.search(&tempSensorAdr[0])){      
      log_i("found onewire TempSensor with adr %X:%X:%X:%X:%X:%X:%X:%X ",tempSensorAdr[0],tempSensorAdr[1],tempSensorAdr[2],tempSensorAdr[3],tempSensorAdr[4],tempSensorAdr[5],tempSensorAdr[6],tempSensorAdr[7]);
      sensors.setResolution(&tempSensorAdr[0],12); //12 Bit resolution
      sensors.requestTemperaturesByAddress(&tempSensorAdr[0]);
      float temperatureC = sensors.getTempC(&tempSensorAdr[0]);
      log_i("temp of sensor 0 = %fC",temperatureC);
      //temperatureC = sensors.getTempCByIndex(0);
      //log_i("temp of sensor 0 = %fC",temperatureC);
      hasTempSensor = true;
    }
  }
  xMutex = xSemaphoreCreateMutex();
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
  return true;
}

void Weather::copyValues(void){
  xSemaphoreTake( xMutex, portMAX_DELAY );
  _weather.bTemp = true;
  _weather.bHumidity = true;
  _weather.bPressure = true;
  _weather.temp = dTemp;
  _weather.Humidity = dHumidity;
  _weather.Pressure = dPressure;
  xSemaphoreGive( xMutex );
}

float Weather::calcPressure(float p, float t, float h){
  if (h > 0){
      //return ((float)p / pow(1-(0.0065*h/288.15),5.255));
      return ((float)p * pow(1-(0.0065*h/(t + (0.0065*h) + 273.15)),-5.257));
  }else{
      return p;
  }

}

void Weather::getValues(weatherData *weather){
  xSemaphoreTake( xMutex, portMAX_DELAY );
  *weather = _weather;
  xSemaphoreGive( xMutex );

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
      sensors.requestTemperaturesByAddress(&tempSensorAdr[0]);
      dTemp = sensors.getTempC(&tempSensorAdr[0]);
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

void Weather::run(void){
  uint32_t tAct = millis();
  runBME280(tAct);
}