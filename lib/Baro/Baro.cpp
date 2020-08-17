#include <Baro.h>

Baro::Baro(){
  //FilterAlt(1.0f, 1.0f, 0.01f);
}

bool Baro::begin(TwoWire *pi2c){
  // Initialize MS5611 sensor
  // Ultra high resolution: MS5611_ULTRA_HIGH_RES
  // (default) High resolution: MS5611_HIGH_RES
  // Standard: MS5611_STANDARD
  // Low power: MS5611_LOW_POWER
  // Ultra low power: MS5611_ULTRA_LOW_POWER
  uint8_t initCount = 0;
  while(!ms5611.begin(pi2c,MS5611_ULTRA_HIGH_RES))
  {
    log_i("no sensor found");
    initCount++;
    if (initCount >= 5){
      return false; //no baro found
    }
    delay(500);
  }  
  //ms5611.doConversion();
  //accelgyro.initialize();
  xMutex = xSemaphoreCreateMutex();
  
  // initialize device
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();
  // reset offsets  
  accelgyro.setXAccelOffset(-1617);
  accelgyro.setYAccelOffset(-1551);
  accelgyro.setZAccelOffset(1147);
  accelgyro.setXGyroOffset(110);
  accelgyro.setYGyroOffset(48);
  accelgyro.setZGyroOffset(-14);
  accelgyro.setI2CMasterModeEnabled(false);
  accelgyro.setI2CBypassEnabled(true) ;
  accelgyro.setSleepEnabled(false);
  //accelgyro.setDMPEnabled();
  mag.initialize();
  Serial.println(mag.testConnection() ? "HMC5883L connection successful" : "HMC5883L connection failed");

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed"); 
  Serial.print("size of data:"); Serial.println(sizeof(logData));  
  memset(&logData,0,sizeof(logData));
  countReadings = 0;
  logData.newData = 0x80; //first measurement
  
  return true;
}

void Baro::calcClimbing(void){
  
  if (logData.newData == 0x80){
    //init alt-array
    //double test = logData.altitude;
    Kalmanvert.init(logData.altitude,
                    0.0,
                    POSITION_MEASURE_STANDARD_DEVIATION,
                    ACCELERATION_MEASURE_STANDARD_DEVIATION,
                    millis());
    //Serial.println("KalmanInit");
  }else{
    Kalmanvert.update( logData.altitude,
                       0.0,
                       millis());
    //Serial.println("KalmanUpdate");
  }
  logData.altitudeFiltered = Kalmanvert.getPosition();
  logData.climb = Kalmanvert.getVelocity();
  //Serial.print("alt:");Serial.print(logData.altitude);Serial.print("alt2:");Serial.print(logData.altitudeFiltered);Serial.print(";climb:");Serial.println(logData.climb);
}
float Baro::getHeading(void){
  float fRet;
  xSemaphoreTake( xMutex, portMAX_DELAY );
  fRet = logData.heading;
  xSemaphoreGive( xMutex );
  return fRet;
}

float Baro::getAlt(void){
  float fRet;
  xSemaphoreTake( xMutex, portMAX_DELAY );
  fRet = logData.altitudeFiltered;
  xSemaphoreGive( xMutex );
  return fRet;  
}

void Baro::copyValues(void){
  xSemaphoreTake( xMutex, portMAX_DELAY );
  fPressure = logData.pressure;
  fClimbRate = logData.climb;
  fAltitude = logData.altitudeFiltered;
  fTemp = logData.temp;
  xSemaphoreGive( xMutex );
}

bool Baro::isNewVAlues(){
  bool bRet = bNewValues;
  bNewValues = false;
  return bRet;
}

void Baro::getValues(float *pressure,float *alt,float *climb,float *temp){  
  xSemaphoreTake( xMutex, portMAX_DELAY );
  *pressure = fPressure;
  *climb = fClimbRate;
  *alt = fAltitude;
  *temp = fTemp;
  xSemaphoreGive( xMutex );

}

void Baro::run(void){
  static uint32_t tOld;  
  uint32_t tAct = millis();
  
  ms5611.run();
  if (ms5611.convFinished()){
    mag.getHeading(&logData.mx, &logData.my, &logData.mz);
    accelgyro.getMotion6(&logData.ax, &logData.ay, &logData.az, &logData.gx, &logData.gy, &logData.gz);
    if (countReadings < 10){
      countReadings++;
    }else{
      if (countReadings == 10){
        logData.newData = 0x80;
        countReadings++;
      }
      // To calculate heading in degrees. 0 degree indicates North
      logData.heading = atan2(logData.my, logData.mx);
      if(logData.heading < 0){
        logData.heading += 2 * M_PI;
      }      
      logData.heading  = logData.heading * 180/M_PI;

      logData.temp = ms5611.readTemperature(true);
      logData.pressure = ms5611.readPressure(true);
      logData.altitude = ms5611.getAltitude(logData.pressure);
      //logData.altitude = alt;
      //float altNew = FilterAlt.updateEstimate(alt);
      //logData.altitudeFiltered = FilterAlt.updateEstimate(logData.altitude);
      //logData.pressureFiltered = FilterPressure.updateEstimate(logData.pressure);  
      //logData.altitudeFiltered = logData.altitude;
      logData.loopTime = tAct - tOld;
      //logData.climb = (logData.altitudeFiltered - oldAlt) * (float)logData.loopTime / 1000.0   ;
      calcClimbing();
      if (logData.newData == 0x80){
        logData.newData = 0;
        bNewValues = false;
      }else{
        copyValues();        
        logData.newData = 1;
        bNewValues = true;
      }
      //Serial.print(logData.pressure,2);Serial.print(";");
      //Serial.print(logData.pressureFiltered,2);Serial.print(";");
      //Serial.print(logData.altitude,2);Serial.print(";");
      //Serial.print(altNew,2);Serial.print(";");
      //Serial.print(logData.loopTime);Serial.print(";");
      //Serial.println(logData.temp,2);
    }    
  }
  #ifdef BARO_DEBUG
  if ((WiFi.status() == WL_CONNECTED) && (logData.newData)){
    udp.beginPacket(BARO_DEBUG_IP,BARO_DEBUG_PORT);
    unsigned int bufferSize = sizeof(logData);
    udp.write((uint8_t *)&logData,bufferSize);
    udp.endPacket(); 
  }
  #endif 
  if (logData.newData){
    tOld = tAct;  
    logData.newData = 0;
  }
  /*
  ms5611.doConversion();
  logData.temp = ms5611.readTemperature(true);
  logData.pressure = ms5611.readPressure(true);
  logData.altitude = ms5611.getAltitude(logData.pressure);
  logData.altitudeFiltered = pFilterAlt->updateEstimate(logData.altitude);
  logData.pressureFiltered = pFilterPressure->updateEstimate((float)logData.pressure);  
  */

  /*
  // display tab-separated accel/gyro x/y/z values
  Serial.print("a/g:\t");
  Serial.print(logData.ax); Serial.print("\t");
  Serial.print(logData.ay); Serial.print("\t");
  Serial.print(logData.az); Serial.print("\t");
  Serial.print(logData.gx); Serial.print("\t");
  Serial.print(logData.gy); Serial.print("\t");
  Serial.print(logData.gz);Serial.print("\t");
   
  Serial.print("mag:\t");
  Serial.print(logData.mx); Serial.print("\t");
  Serial.print(logData.my); Serial.print("\t");
  Serial.print(logData.mz); Serial.print("\t");
   
// To calculate heading in degrees. 0 degree indicates North
  logData.heading = atan2(logData.my, logData.mx);
  if(logData.heading < 0)
    logData.heading += 2 * M_PI;
  logData.heading  = logData.heading * 180/M_PI;
  //Serial.print("heading:\t");
  //Serial.println(logData.heading);
  */

  //copy data for udp
  //xSemaphoreTake(data_mutex, portMAX_DELAY); 
  //stUdpData = logData; //copy data
  //xSemaphoreGive(data_mutex); 

  /*
  // Output
  Serial.print(logData.temp);
  Serial.print(":");
  Serial.print(logData.pressure);
  Serial.print(":");
  Serial.print(logData.altitude);
  Serial.print(":");
  Serial.print(tAct - tOld);
  Serial.println();
  */

}