#include <Baro.h>
//#include <MPU6050.h>
//#include "MPU6050_6Axis_MotionApps20.h"
#include "MPU6050_6Axis_MotionApps_V6_12.h"

//#define MAX2G 32768
#define MPU6050_2G_SENSITIVITY 		32768.0f 	// lsb per g
#define MAX2G 16384
//#define MPU6050_2G_SENSITIVITY 		16384.0f 	// lsb per g



MPU6050 mpu;

Baro::Baro(){
  //FilterAlt(1.0f, 1.0f, 0.01f);
}

bool Baro::initBMP3XX(void){
  uint8_t error;
  bool ret = false;
  for (sensorAdr = 0x76; sensorAdr <= 0x77; sensorAdr++)
  {
    log_i("check device at address 0x%X !",sensorAdr);
    pI2c->beginTransmission(sensorAdr);
    error = pI2c->endTransmission();
    if (error == 0){
      log_i("I2C device found at address 0x%X !",sensorAdr);
      ret = bmp3xx.begin_I2C(sensorAdr,pI2c);
      log_i("check sensor on adr 0x%X ret=%d",sensorAdr,ret);
      if (ret){
        log_i("found sensor BMP3XX on adr 0x%X",sensorAdr);
        break;
      }
    }
    else
    {
      log_i("Checking device at address 0x%X returned error %X", sensorAdr, error);
    }
  }

  if (!ret) return false;

  uint8_t chipId = bmp3xx.chipID();
  if ((chipId == BMP3_CHIP_ID) || (chipId == BMP390_CHIP_ID))
  log_i("found sensor BMP3xx on adr 0x%X",sensorAdr);
  sensorType = SENSORTYPE_BMP3XX; //init to no sensor connected
  //sensor found --> set sampling
  bmp3xx.setPressureOversampling(BMP3_OVERSAMPLING_8X);
  bmp3xx.setTemperatureOversampling(BMP3_NO_OVERSAMPLING);
  bmp3xx.setIIRFilterCoeff(BMP3_IIR_FILTER_DISABLE);
  return true;
}

bool Baro::initBME280(void){
  uint8_t error;
  sensorAdr = 0x76;
  bool ret = false;
  xSemaphoreTake( *xMutexI2C, portMAX_DELAY );
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
  xSemaphoreGive( *xMutexI2C );
  if (!ret) return false;
  
  char sensType ='P';
  if (bme.sensorID()==BME280CHIP_ID)
    sensType='E';    
  log_i("found sensor BM%C280 on adr 0x%X",sensType,sensorAdr);
  sensorType = SENSORTYPE_BME280; //init to no sensor connected
  //sensor found --> set sampling
  xSemaphoreTake( *xMutexI2C, portMAX_DELAY );
  bme.setSampling(Adafruit_BME280::MODE_NORMAL, // mode
                  Adafruit_BME280::SAMPLING_X1, // temperature
                  Adafruit_BME280::SAMPLING_X4, // pressure
                  Adafruit_BME280::SAMPLING_NONE, // humidity
                  Adafruit_BME280::FILTER_X16,  //filter
                  Adafruit_BME280::STANDBY_MS_0_5);  //duration
  xSemaphoreGive( *xMutexI2C );
  return true;
}

void Baro::getMPUValues(int16_t accel[3],int16_t gyro[3],float *acc_Z){
  for (int i = 0; i < 3;i++){
    accel[i] = logData.accel[i];
    gyro[i] = logData.gyro[i];
  }
  *acc_Z = logData.acc;
}

void Baro::meansensors() {
	long i, buff_ax, buff_ay, buff_az, buff_gx, buff_gy, buff_gz;
  int16_t ax, ay, az, gx, gy, gz;
	const int buffersize = 1000;     //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
  //int16_t az_values[buffersize];

	buff_ax = 0; buff_ay = 0; buff_az = 0; buff_gx = 0; buff_gy = 0; buff_gz = 0;
	i = 0;

	while (i < (buffersize)) {
		// read raw accel/gyro measurements from device
		mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
		//Serial.printf("az raw value %d\r\n", az);
		buff_ax += ax;
		buff_ay += ay;
		buff_az += az;
		//az_values[i] = az;
		buff_gx += gx;
		buff_gy += gy;
		buff_gz += gz;

		i++;
		delay(2); //Needed so we don't get repeated measures
	}
	mean_ax = buff_ax / buffersize;
	mean_ay = buff_ay / buffersize;
	mean_az = buff_az / buffersize;
	mean_gx = buff_gx / buffersize;
	mean_gy = buff_gy / buffersize;
	mean_gz = buff_gz / buffersize;

	Serial.println(mean_ax);
	Serial.println(mean_ay);
	Serial.println(mean_az);
	Serial.println(mean_gx);
	Serial.println(mean_gx);
	Serial.println(mean_gx);
}

bool Baro::calibrate(bool bInit,uint8_t* calibstate){
  int16_t ax, ay, az, gx, gy, gz;
  bool bRet = false;
  if (bInit){
    mpu.setDMPEnabled(false);
    /*
    mpu.setXAccelOffset(0);
    mpu.setYAccelOffset(0);
    mpu.setZAccelOffset(0);
    mpu.setXGyroOffset(0);
    mpu.setYGyroOffset(0);
    mpu.setZGyroOffset(0);
    */
    //First 100 measures are discarded
    axMin = 0;
    axMax = 0;
    ayMin = 0;
    ayMax = 0;
    azMin = 0;
    azMax = 0;
  }
  for (int j = 0; j < 100; j++) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    delay(2);
  }
  meansensors();  
  if ((abs(mean_ax) <= 5000) && (abs(mean_ay) <= 5000) && (mean_az > 10000)){
    *calibstate |= 1;
    bRet = true;
    azMax = mean_az;
    log_i("azMax=%d",azMax);
  }
  if ((abs(mean_ax) <= 5000) && (abs(mean_ay) <= 5000) && (mean_az < -10000)){
    *calibstate |= 2;
    bRet = true;
    azMin = mean_az;
    log_i("azMin=%d",azMin);
  }
  if ((abs(mean_ax) <= 5000) && (mean_ay > 10000) && (abs(mean_az) <= 5000)){
    *calibstate |= 4;
    bRet = true;
    ayMax = mean_ay;
    log_i("ayMax=%d",ayMax);
  }
  if ((abs(mean_ax) <= 5000) && (mean_ay < -10000) && (abs(mean_az) <= 5000)){
    *calibstate |= 8;
    bRet = true;
    ayMin = mean_ay;
    log_i("ayMin=%d",ayMin);
  }
  if ((mean_ax > 10000) && (abs(mean_ay) <= 5000) && (abs(mean_az) <= 5000)){
    *calibstate |= 16;
    bRet = true;
    axMax = mean_ax;
    log_i("axMax=%d",axMax);
  }
  if ((mean_ax < -10000) && (abs(mean_ay) <= 5000) && (abs(mean_az) <= 5000)){
    *calibstate |= 32;
    bRet = true;
    axMin = mean_ax;
    log_i("axMin=%d",axMin);
  }
  if ((axMin != 0) && (axMax != 0) && (ayMin != 0) && (ayMax != 0) && (azMin != 0) && (azMax != 0)){
    // we are ready !!
    ax_offset = (axMax + axMin) / 2 * -1;
    ay_offset = (ayMax + ayMin) / 2 * -1;
    az_offset = (azMax + azMin) / 2 * -1;
    
    //float aRange;
    ax_scale = 32768 / (float(axMax) - float(axMin));
    ay_scale = 32768 / (float(ayMax) - float(ayMin));
    az_scale = 32768 / (float(azMax) - float(azMin));
    log_i("write new offsets");
    log_i("scale ax_scale=%.02f,ay_scale=%.02f,az_scale=%.02f,",ax_scale,ay_scale,az_scale);
    log_i("acc offsets ax_offset=%d,ay_offset=%d,az_offset=%d",ax_offset,ay_offset,az_offset);
    Preferences preferences;
    preferences.begin("fastvario", false);
    preferences.putInt("axOffset", ax_offset);
    preferences.putInt("ayOffset", ay_offset);
    preferences.putInt("azOffset", az_offset);

    preferences.putFloat("axScale", ax_scale);
    preferences.putFloat("ayScale", ay_scale);
    preferences.putFloat("azScale", az_scale);
    
    logData.temp = getMpuTemp();
    log_i("temp=%.1f",logData.temp);
    preferences.putFloat("t[0]",logData.temp); //set temp from calibrating

    preferences.end();
    //return true;
  }
  return bRet;
}

bool Baro::calibration() {

  //int acel_deadzone = 8;			 //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
  int giro_deadzone = 1;           //Giro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)
	int numtries = 0;
  int16_t ax, ay, az, gx, gy, gz;
	mpu.setDMPEnabled(false);
  mpu.setXAccelOffset(0);
	mpu.setYAccelOffset(0);
	mpu.setZAccelOffset(0);
	mpu.setXGyroOffset(0);
	mpu.setYGyroOffset(0);
	mpu.setZGyroOffset(0);

	//First 100 measures are discarded
	for (int j = 0; j < 100; j++) {
		mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
		delay(2);
	}

	//The mpu6050 yaw figues drift and takes a little time to stablize ~15 secs with reading settling after ~30s

	meansensors();
	delay(1000);

	ax_offset = -mean_ax / 8;
	ay_offset = -mean_ay / 8;
	az_offset = (MAX2G - mean_az) / 8;

	gx_offset = -mean_gx / 4;
	gy_offset = -mean_gy / 4;
	gz_offset = -mean_gz / 4;

	while (1) {
		int ready = 0;
		mpu.setXAccelOffset(ax_offset);
		mpu.setYAccelOffset(ay_offset);
		mpu.setZAccelOffset(az_offset);

		mpu.setXGyroOffset(gx_offset);
		mpu.setYGyroOffset(gy_offset);
		mpu.setZGyroOffset(gz_offset);

		meansensors();
		Serial.printf("...\r\n");

		/*
    if (abs(mean_ax) <= acel_deadzone) ready++;
		else ax_offset = ax_offset - mean_ax / acel_deadzone;

		if (abs(mean_ay) <= acel_deadzone) ready++;
		else ay_offset = ay_offset - mean_ay / acel_deadzone;

		if (abs(MAX2G - mean_az) <= acel_deadzone) ready++;
		else az_offset = az_offset + (MAX2G - mean_az) / acel_deadzone;
    */

		if (abs(mean_gx) <= giro_deadzone) ready++;
		else gx_offset = gx_offset - mean_gx / (giro_deadzone + 1);

		if (abs(mean_gy) <= giro_deadzone) ready++;
		else gy_offset = gy_offset - mean_gy / (giro_deadzone + 1);

		if (abs(mean_gz) <= giro_deadzone) ready++;
		else gz_offset = gz_offset - mean_gz / (giro_deadzone + 1);

		numtries++;

		//if (ready == 6) {
    if (ready == 3){
      log_i("write new offsets");
      Preferences preferences;
      preferences.begin("fastvario", false);
      //preferences.putInt("axOffset", ax_offset);
      //preferences.putInt("ayOffset", ay_offset);
      //preferences.putInt("azOffset", az_offset);
      preferences.putInt("gxOffset", gx_offset);
      preferences.putInt("gyOffset", gy_offset);
      preferences.putInt("gzOffset", gz_offset);
      preferences.end();
      log_i("reboot");
      ESP.restart();      
			return true;
		}

		if (numtries > 20) return false;
	}

}


bool Baro::calibGyro(void){
  if (sensorType != SENSORTYPE_MS5611) return false; //sensortype different
  int giro_deadzone = 1;           //Giro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)
	int numtries = 0;
  int16_t ax, ay, az, gx, gy, gz;
	mpu.setDMPEnabled(false);
	mpu.setXGyroOffset(0);
	mpu.setYGyroOffset(0);
	mpu.setZGyroOffset(0);

	//First 100 measures are discarded
	for (int j = 0; j < 100; j++) {
		mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
		delay(2);
	}

	//The mpu6050 yaw figues drift and takes a little time to stablize ~15 secs with reading settling after ~30s

	meansensors();
	delay(1000);

	gx_offset = -mean_gx / 4;
	gy_offset = -mean_gy / 4;
	gz_offset = -mean_gz / 4;

	while (1) {
		int ready = 0;
		mpu.setXGyroOffset(gx_offset);
		mpu.setYGyroOffset(gy_offset);
		mpu.setZGyroOffset(gz_offset);

		meansensors();
		Serial.printf("...\r\n");

		if (abs(mean_gx) <= giro_deadzone) ready++;
		else gx_offset = gx_offset - mean_gx / (giro_deadzone + 1);

		if (abs(mean_gy) <= giro_deadzone) ready++;
		else gy_offset = gy_offset - mean_gy / (giro_deadzone + 1);

		if (abs(mean_gz) <= giro_deadzone) ready++;
		else gz_offset = gz_offset - mean_gz / (giro_deadzone + 1);

		numtries++;

		if (ready == 3) {
      log_i("write new offsets");
      Preferences preferences;
      preferences.begin("fastvario", false);
      preferences.putInt("gxOffset", gx_offset);
      preferences.putInt("gyOffset", gy_offset);
      preferences.putInt("gzOffset", gz_offset);
      preferences.end();
      //log_i("reboot");
      //ESP.restart();      
			return true;
		}

		if (numtries > 20) return false;
	}
  return true;
}

bool Baro::calibAcc(void){
  if (sensorType != SENSORTYPE_MS5611) return false; //sensortype different
  int acel_deadzone = 8;			 //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
	int numtries = 0;
  int16_t ax, ay, az, gx, gy, gz;
	mpu.setDMPEnabled(false);
  mpu.setXAccelOffset(0);
	mpu.setYAccelOffset(0);
	mpu.setZAccelOffset(0);

	//First 100 measures are discarded
	for (int j = 0; j < 100; j++) {
		mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
		delay(2);
	}

	//The mpu6050 yaw figues drift and takes a little time to stablize ~15 secs with reading settling after ~30s

	meansensors();
	delay(1000);

	ax_offset = -mean_ax / 8;
	ay_offset = -mean_ay / 8;
  az_offset = (MAX2G - mean_az) / 8;

	while (1) {
		int ready = 0;
		mpu.setXAccelOffset(ax_offset);
		mpu.setYAccelOffset(ay_offset);
		mpu.setZAccelOffset(az_offset);

		meansensors();
		Serial.printf("...\r\n");

		if (abs(mean_ax) <= acel_deadzone) ready++;
		else ax_offset = ax_offset - mean_ax / acel_deadzone;

		if (abs(mean_ay) <= acel_deadzone) ready++;
		else ay_offset = ay_offset - mean_ay / acel_deadzone;

		if (abs(MAX2G - mean_az) <= acel_deadzone) ready++;
		else az_offset = az_offset + (MAX2G - mean_az) / acel_deadzone;

		numtries++;

		if (ready == 3) {
      log_i("write new offsets");
      Preferences preferences;
      preferences.begin("fastvario", false);
      preferences.putInt("axOffset", ax_offset);
      preferences.putInt("ayOffset", ay_offset);
      preferences.putInt("azOffset", az_offset);
      preferences.end();
      //log_i("reboot");
      //ESP.restart();      
			return true;
		}

		if (numtries > 20) return false;
	}
  return true;
}


bool Baro::initMS5611(void){
  // Initialize MS5611 sensor
  // Ultra high resolution: MS5611_ULTRA_HIGH_RES
  // (default) High resolution: MS5611_HIGH_RES
  // Standard: MS5611_STANDARD
  // Low power: MS5611_LOW_POWER
  // Ultra low power: MS5611_ULTRA_LOW_POWER
  //uint8_t initCount = 0;
  //log_i("init ms5611");
  if (!ms5611.begin(pI2c,MS5611_ULTRA_HIGH_RES)){
    return false; //no baro found
  }
  log_i("found sensor MS5611");
  /*
  while(!ms5611.begin(pI2c,MS5611_ULTRA_HIGH_RES))
  {
    log_i("no sensor found");
    initCount++;
    if (initCount >= 5){
      return false; //no baro found
    }
    delay(500);
  }  
  */
  sensorType = SENSORTYPE_MS5611; //init to no sensor connected
  if (mpu.testConnection()){
    log_i("MPU6050 connection successful");
  }else{
    log_i("MPU6050 connection failed");
  }

  Preferences preferences;
	preferences.begin("fastvario", false);
	ax_offset = preferences.getInt("axOffset", 0);
	ay_offset = preferences.getInt("ayOffset", 0);
	az_offset = preferences.getInt("azOffset", 0);
	ax_scale = preferences.getFloat("axScale", 1.0);
	ay_scale = preferences.getFloat("ayScale", 1.0);
	az_scale = preferences.getFloat("azScale", 1.0);
	gx_offset = preferences.getInt("gxOffset", 0);
	gy_offset = preferences.getInt("gyOffset", 0);
	gz_offset = preferences.getInt("gzOffset", 0);
  tValues[0] = preferences.getFloat("t[0]",20.0);
  tValues[1] = preferences.getFloat("t[1]",0.0);
  zValues[0] = preferences.getFloat("z[0]",0.0);
  zValues[1] = preferences.getFloat("z[1]",0.0);  
  preferences.end();
  // initialize device
  mpu.initialize();

  //log_i("tempcorrection: t0=%.2f z0=%.2f t1=%.2f z1=%.2f",tValues[0],zValues[0],tValues[1],zValues[1]);
  log_i("tempcorrection: t0=%.2f z0=%.2f",tValues[0],zValues[0]);

	log_i("Initializing DMP...");
	devStatus = mpu.dmpInitialize();

  if ((gx_offset == 0) && (gy_offset == 0) && (gz_offset == 0)){
    //bUseAcc = false;
    gx_offset = mpu.getXGyroOffset();
    gy_offset = mpu.getYGyroOffset();
    gz_offset = mpu.getZGyroOffset();
    log_i("gyro not calibrated");
  }else{
    mpu.setXGyroOffset(gx_offset);
    mpu.setYGyroOffset(gy_offset);
    mpu.setZGyroOffset(gz_offset);
  }

  int actAxOffset = mpu.getXAccelOffset();
  int actAyOffset = mpu.getYAccelOffset();
  int actAzOffset = mpu.getZAccelOffset();
  log_i("acc offsets ax_offset=%d,ay_offset=%d,az_offset=%d",actAxOffset,actAyOffset,actAzOffset);
  /*
  if ((ax_offset == 0) && (ay_offset == 0) && (az_offset == 0)){
    //bUseAcc = false;
    ax_offset = mpu.getXAccelOffset();
    ay_offset = mpu.getYAccelOffset();
    az_offset = mpu.getZAccelOffset();
    log_i("accelerometer not calibrated");
  }else{
    mpu.setXAccelOffset(ax_offset);
    mpu.setYAccelOffset(ay_offset);
    mpu.setZAccelOffset(az_offset);
  }
  */

  log_i("scale ax_scale=%.02f,ay_scale=%.02f,az_scale=%.02f,",ax_scale,ay_scale,az_scale);
  log_i("acc offsets ax_offset=%d,ay_offset=%d,az_offset=%d",ax_offset,ay_offset,az_offset);
  log_i("gyro offsets gx_offset=%d,gy_offset=%d,gz_offset=%d",gx_offset,gy_offset,gz_offset);

  mpu.setDMPEnabled(true);

  // get expected DMP packet size for later comparison		
  packetSize = mpu.dmpGetFIFOPacketSize();

  mag.initialize(); //init mag
  return true;
}

uint8_t Baro::begin(TwoWire *pi2c,SemaphoreHandle_t *_xMutex){
  xMutexI2C = _xMutex;
  uint8_t ret = 0;
  pI2c = pi2c;
  sensorType = SENSORTYPE_NONE; //init to no sensor connected
  logData.vAcc = 0.0;
  logData.vOffset = 0.0;
  if (initBME280()){
    log_i("found BME280");
    ret = 1; //BME280;
  }else if (initMS5611()){
    log_i("found MS5611");
    ret = 2; //GY-86-Board
  }else if (initBMP3XX()){
    log_i("found BMP3xx");
    ret = 3; //BMP3xx;
  }else{
    return 0;
  }
  xMutex = xSemaphoreCreateMutex();
  //Serial.print("size of data:"); Serial.println(sizeof(logData));  
  memset(&logData,0,sizeof(logData));
  countReadings = 0;
  logData.newData = 0x80; //first measurement
  
  return ret;

}

void Baro::useMPU(bool bUseMPU){
  bUseAcc = bUseMPU;
}

void Baro::setKalmanSettings(float sigmaP,float sigmaA){
  _sigmaP = sigmaP;
  _sigmaA = sigmaA;
}

void Baro::calcClimbing(void){
  /*
  if ((logData.loopTime <= 50000) || (logData.newData == 0x80)){

  }else{
    //log_w("looptime to high %d",logData.loopTime);
    return;
  }
  */
  if (logData.newData == 0x80){
    //init alt-array
    //double test = logData.altitude;
    log_i("Kalman_settings sigmaP=%.04f,sigmaA=%.04f",_sigmaP,_sigmaA);
    //if (bUseAcc){
      Kalmanvert.init(logData.altitude,
                      0.0,
                      _sigmaP,
                      _sigmaA,
                      millis());
    /*
    }else{
      Kalmanvert.init(logData.altitude,
                      0.0,
                      _sigmaP,
                      _sigmaA,
                      millis());
    }
    */
    //Serial.println("KalmanInit");
  }else{    
    if (bUseAcc){
      Kalmanvert.update( logData.altitude,
                        logData.acc,
                        millis());
    }else{
      Kalmanvert.update( logData.altitude,
                        0.0,
                        millis());
    }
    //Serial.println("KalmanUpdate");
  }
  logData.altitudeFiltered = Kalmanvert.getPosition();
  logData.pos = Kalmanvert.getPosition();
  logData.velo = Kalmanvert.getVelocity();
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
  fClimbRate = logData.velo;
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

bool Baro::mpuDrdy(void){
  
  uint8_t mpuIntStatus = mpu.getIntStatus();
  uint16_t fifoCount = mpu.getFIFOCount();
  
  //if (!drdyFlag) return false; //wait for new Data
  //drdyFlag = false;
  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    //log_i("FIFO overflow! resetting..");
    //log_i("Enabling FIFO...");
    mpu.setFIFOEnabled(true);
    //log_i("Enabling DMP...");
    mpu.setDMPEnabled(true);
  }else if (mpuIntStatus & 0x02) {
    //log_i("new data from mpu");
    uint16_t fifoCount = mpu.getFIFOCount();
    if( fifoCount != packetSize){
      //log_i("wrong packet-length %d",fifoCount);
      // reset so we can continue cleanly
      mpu.resetFIFO();
      //log_i("FIFO overflow! resetting..");
      //log_i("Enabling FIFO...");
      mpu.setFIFOEnabled(true);
      //log_i("Enabling DMP...");
      mpu.setDMPEnabled(true);
    }else{
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      return true;
    }
    
  }
  return false;

}
void Baro::scaleAccel(VectorInt16 *accel,float temp){
  //static uint32_t tLog = millis();
  //y=mx+b; //linear function  
  float scale_x,scale_y,scale_z;
  float offset_x,offset_y,offset_z;  
  float tempOffsetZ;
  float tempdiff;
  /*
  scale_x = 1;
  scale_y = 1;
  scale_z = 1;
  offset_x = 0;
  offset_y = 0;
  offset_z = 0;
  */
  scale_x = ax_scale;
  scale_y = ay_scale;
  scale_z = az_scale;
  offset_x = ax_offset;
  offset_y = ay_offset;
  offset_z = az_offset;
  ;

  /*
  scale factor=((average couts at +1g)-(average counts at -1g))/(2g's)
  X +1g = 8265,164    -1g= -7989,700 = 8265,164 - (-7989,700) / 16384,0 = 0,99212 --> 1,0079
  Y +1g = 8092,989    -1g= -8167,884 = 8092,989 - (-8167,884) / 16384,0 = 0,99248 --> 1,0076
  Z +1g = 7804,699    -1g= -8779,637 = 7804,699 - (-8779,637) / 16384,0 = 1,0122  --> 0,9879


  Offset = - (average counts when axis is perpendicular to g)
  x1=223,229 x2=60,713 x3=150,532 x4=225,815 = 165,07225
  y1=-73,071 y2=76,648 y3=36,225 y4=-48,902 = -2,275
  z1=-1606,632 z2=159,373 z3=-556,161 z4=-1123,953 = -781,843

  */
  
  accel->x = (int16_t)round(scale_x * (float)accel->x + offset_x);//(offset_x * scale_x));
  accel->y = (int16_t)round(scale_y * (float)accel->y + offset_y);//(offset_y * scale_y));
  /*
  if ((millis() - tLog) >= 1000){
    tLog = millis();
    log_i("t=%.2f,offset_z=%.2f",temp,offset_z);
  }
  */
  //accel->z is drifting with temperature --> so we have to correct it.
  if (zValues[0] != 0){
    tempdiff = tValues[0] - temp;
    tempOffsetZ = tempdiff * zValues[0]; //calc temp-kompensation
  }else{
    tempOffsetZ = 0.0;
  }
  accel->z = (int16_t)round(scale_z * (float)accel->z + offset_z + tempOffsetZ);//(offset_z * scale_z));
}

float Baro::getGravityCompensatedAccel(float temp){
    // orientation/motion vars
    Quaternion q;           // [w, x, y, z]         quaternion container
    VectorInt16 aa;         // [x, y, z]            accel sensor measurements
    VectorInt16 gy;         // [x, y, z]            gyro sensor measurements
    VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
    VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
    VectorFloat gravity;    // [x, y, z]            gravity vector
    //float tempOffsetZAcc = 0.0;
    // get current FIFO count
    
    //uint16_t fifoCount = mpu.getFIFOCount();
    //if( fifoCount != packetSize){
    //  log_i("wrong packet-length %d",fifoCount);
    //}
    // wait for correct available data length, should be a VERY short wait
    //while (fifoCount < packetSize){
    //  fifoCount = mpu.getFIFOCount();
    //  delay(1);
    //} 
    //mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    //fifoCount -= packetSize;


    // display initial world-frame acceleration, adjusted to remove gravity
    // and rotated based on known orientation from quaternion
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGyro(&gy, fifoBuffer);
    scaleAccel(&aa,temp); //scale an offset to acceleration !!
    logData.accel[0] = aa.x;
    logData.accel[1] = aa.y;
    logData.accel[2] = aa.z;
    logData.gyro[0] = gy.x;
    logData.gyro[1] = gy.y;
    logData.gyro[2] = gy.z;
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
    logData.gravity[0] = gravity.x;
    logData.gravity[1] = gravity.y;
    logData.gravity[2] = gravity.z;
    logData.aaReal[0] = aaReal.x;
    logData.aaReal[1] = aaReal.y;
    logData.aaReal[2] = aaReal.z;
    logData.aaWorld[0] = aaWorld.x;
    logData.aaWorld[1] = aaWorld.y;
    logData.aaWorld[2] = aaWorld.z;
    /*
    if ((tValues[0] != tValues[1]) && (zValues[0] != zValues[1])){
      tempOffsetZAcc = interpolate.Linear(tValues,zValues,2,temp,false);
    }else{
      tempOffsetZAcc = 0.0; //no temp-Offset
    } 
    */   
    return float(aaWorld.z*(9.80665 / MPU6050_2G_SENSITIVITY));// + tempOffsetZAcc; //to get m/s
}

float Baro::getMpuTemp(void){
  return (float(mpu.getTemperature()) / 340) + 36.53;
}

uint32_t Baro::get2of3(uint32_t *press, uint32_t newPress){
  press[2] = press[1];
  press[1] = press[0];
  press[0] = newPress;
  int32_t diff[3];
  diff[0] = abs((int32_t)press[0] - (int32_t)press[2]);
  diff[1] = abs((int32_t)press[1] - (int32_t)press[0]);
  diff[2] = abs((int32_t)press[2] - (int32_t)press[1]);
  if ((diff[0] <= diff[1])  && (diff[0] <= diff[2])){
    return press[0];
  }else if ((diff[1] <= diff[2])  && (diff[1] <= diff[0])){
    return press[0];
  }else if ((diff[2] <= diff[1])  && (diff[2] <= diff[0])){
    return press[1];
  }
  return press[0];
}

void Baro::runMS5611(uint32_t tAct){
  static uint32_t tOld;
  static float press = 0.0f;
  static float temp = 0.0f;
  static uint8_t baroCount = 0;
  float lTemp = 0.0f;
  //static uint32_t tTemp = millis();

  static float acc = 0.0f;
  static uint8_t mpuCount = 0;
  ms5611.run();
  if (ms5611.convFinished()){
    uint32_t actPress = ms5611.readPressure(true);
    logData.pressmeasure = actPress;
    #ifdef newBaro
    logData.pressureFiltered = get2of3(&arPress[0],actPress);
    #endif
    press += actPress;
    //temp += ms5611.readTemperature(true);
    //log_i("rawTemp=%d,Temp=%.02f",ms5611.getRawTemperature(),ms5611.readTemperature(true));
    logData.mx++;
    baroCount++;
  }
  bool bReady = mpuDrdy();
  if (bReady){
    lTemp = getMpuTemp();
    temp += lTemp;
    acc += getGravityCompensatedAccel(lTemp);
		mpuCount++;
  }
  #ifdef newBaro
  if (mpuCount > 0){
    press = logData.pressureFiltered;
  #else
  if ((baroCount > 0) && (mpuCount > 0)){
    press = press / float(baroCount);
  #endif
    //temp = temp / float(baroCount);
    acc = acc / float(mpuCount);
    temp = temp / float(mpuCount);
    logData.acc = acc;
    logData.baroCount = baroCount;
    logData.mpuCount = mpuCount;    
    if (countReadings < 10){
      countReadings++;
    }else{
      if (countReadings == 10){
        logData.newData = 0x80;
        countReadings++;
      }
      // To calculate heading in degrees. 0 degree indicates North
      /*
      mag.getHeading(&logData.mx, &logData.my, &logData.mz);
      logData.heading = atan2(logData.my, logData.mx);
      if(logData.heading < 0){
        logData.heading += 2 * M_PI;
      }      
      logData.heading  = logData.heading * 180/M_PI;
      */

      logData.temp = temp;
      logData.pressure = press;
      //log_i("temp=%f pressure=%f",logData.temp,logData.pressure);

      logData.altitude = ms5611.getAltitude(logData.pressure);
      logData.baroPos = ms5611.getAltitude(logData.pressure);
      logData.loopTime = micros() - tOld;
      tOld = micros();
      logData.vAcc += (acc * ((float)logData.loopTime / 1000000.0));// + logData.vOffset;
      calcClimbing();
      #ifdef newBaro
      if ((logData.vAcc < 0) && (logData.vOffset < 0)){
        logData.vOffset = 0;
      }
      if ((logData.vAcc > 0) && (logData.vOffset > 0)){
        logData.vOffset = 0;
      }
      logData.vOffset += (logData.velo - logData.vAcc) * ((float)logData.loopTime/1000000.0) * 0.01;
      #endif
      
      if (logData.newData == 0x80){
        logData.newData = 0;
        bNewValues = false;
      }else{
        copyValues();        
        logData.newData = 1;
        bNewValues = true;
      }
    }
    press = 0.0f;
    temp = 0.0f;
    baroCount = 0;
    acc = 0.0f;
    mpuCount = 0;
  }

}

void Baro::runBME280(uint32_t tAct){
  static uint32_t tOld = millis();
  if ((tAct - tOld) >= 10){
    xSemaphoreTake( *xMutexI2C, portMAX_DELAY );
    bme.readADCValues();
    xSemaphoreGive( *xMutexI2C );
    if (countReadings < 10){
      countReadings++;
    }else{
      if (countReadings == 10){
        logData.newData = 0x80;
        countReadings++;
      }
      logData.temp = bme.getTemp()/100;
      logData.pressure = bme.getPressure();
      //log_i("temp=%f pressure=%f",logData.temp,logData.pressure);
      logData.altitude = ms5611.getAltitude(logData.pressure);
      logData.loopTime = tAct - tOld;
      calcClimbing();
      if (logData.newData == 0x80){
        logData.newData = 0;
        bNewValues = false;
      }else{
        copyValues();        
        logData.newData = 1;
        bNewValues = true;
      }
    }
  }
}

void Baro::runBMP3XX(uint32_t tAct){
  static uint32_t tOld = millis();
  //if ((tAct - tOld) >= 10)
  {
    if (!bmp3xx.performReading()) {
      log_e("Failed to perform reading :(");
      initBMP3XX();
      return;
    }
    if (countReadings < 10){
      countReadings++;
    }else{
      if (countReadings == 10){
        logData.newData = 0x80;
        countReadings++;
      }
      logData.temp = static_cast<float>(bmp3xx.temperature);
      logData.pressure = static_cast<float>(bmp3xx.pressure);
      logData.altitude = ms5611.getAltitude(bmp3xx.pressure);
      logData.loopTime = tAct - tOld;
      calcClimbing();
      if (logData.newData == 0x80){
        logData.newData = 0;
        bNewValues = false;
      }else{
        copyValues();        
        logData.newData = 1;
        bNewValues = true;
      }
    }
  }
}

void Baro::run(void){
  //static uint32_t tOld;  
  uint32_t tAct = millis();

  if (sensorType == SENSORTYPE_MS5611){
    runMS5611(tAct);
  }else if (sensorType == SENSORTYPE_BME280){
    runBME280(tAct);
  }else if (sensorType == SENSORTYPE_BMP3XX){
    runBMP3XX(tAct);
  }

  #ifdef BARO_DEBUG
  if (logData.newData){
    if ((WiFi.status() == WL_CONNECTED) && (logData.newData)){
      udp.beginPacket(BARO_DEBUG_IP,BARO_DEBUG_PORT);
      unsigned int bufferSize = sizeof(logData);
      udp.write((uint8_t *)&logData,bufferSize);
      udp.endPacket(); 
    }else{
      //log_i("acc_x=%d,acc_y=%d,acc_z=%d,gyro_x=%d,gyro_y=%d,gyro_z=%d,climd=%.2f",logData.accel[0],logData.accel[1],logData.accel[2],logData.gyro[0],logData.gyro[1],logData.gyro[2],logData.velo);
    }
  }
  #endif 
  if (logData.newData){
    //tOld = tAct;  
    logData.newData = 0;
  }
}

void Baro::end(void){
  if (sensorType == SENSORTYPE_MS5611){
    mpu.resetFIFO();
    mpu.setFIFOEnabled(false);
    mpu.setDMPEnabled(false);
    mpu.setSleepEnabled(true);
  }
}