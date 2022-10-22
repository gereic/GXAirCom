/*!
 * @file Adafruit_BME280.cpp
 *
 * @mainpage Adafruit BME280 humidity, temperature & pressure sensor
 *
 * @section intro_sec Introduction
 *
 *  Driver for the BME280 humidity, temperature & pressure sensor
 *
 * These sensors use I2C or SPI to communicate, 2 or 4 pins are required
 * to interface.
 *
 * Designed specifically to work with the Adafruit BME280 Breakout
 * ----> http://www.adafruit.com/products/2652
 *
 *  Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing
 *  products from Adafruit!
 *
 * @section author Author
 *
 * Written by Kevin "KTOWN" Townsend for Adafruit Industries.
 *
 * @section license License
 *
 * BSD license, all text here must be included in any redistribution.
 * See the LICENSE file for details.
 *
 */

#include "Adafruit_BME280.h"
#include "Arduino.h"
#include <SPI.h>
#include <Wire.h>

/*!
 *  @brief  class constructor
 */
Adafruit_BME280::Adafruit_BME280() : _cs(-1), _mosi(-1), _miso(-1), _sck(-1) {}

/*!
 *   @brief  class constructor if using hardware SPI
 *   @param  cspin the chip select pin to use
 *   @param  *theSPI
 *           optional SPI object
 */
Adafruit_BME280::Adafruit_BME280(int8_t cspin, SPIClass *theSPI) {
  _cs = cspin;
  _mosi = _miso = _sck = -1;
  _spi = theSPI;
}

/*!
 *   @brief  class constructor if using software SPI
 *   @param cspin the chip select pin to use
 *   @param mosipin the MOSI pin to use
 *   @param misopin the MISO pin to use
 *   @param sckpin the SCK pin to use
 */
Adafruit_BME280::Adafruit_BME280(int8_t cspin, int8_t mosipin, int8_t misopin,
                                 int8_t sckpin)
    : _cs(cspin), _mosi(mosipin), _miso(misopin), _sck(sckpin) {}

/*!
 *   @brief  Initialise sensor with given parameters / settings
 *   @param theWire the I2C object to use
 *   @returns true on success, false otherwise
 */
bool Adafruit_BME280::begin(TwoWire *theWire) {
  _wire = theWire;
  _i2caddr = BME280_ADDRESS;
  return init();
}

/*!
 *   @brief  Initialise sensor with given parameters / settings
 *   @param addr the I2C address the device can be found on
 *   @returns true on success, false otherwise
 */
bool Adafruit_BME280::begin(uint8_t addr) {
  _i2caddr = addr;
  _wire = &Wire;
  return init();
}

/*!
 *   @brief  Initialise sensor with given parameters / settings
 *   @param addr the I2C address the device can be found on
 *   @param theWire the I2C object to use
 *   @returns true on success, false otherwise
 */
bool Adafruit_BME280::begin(uint8_t addr, TwoWire *theWire) {
  _i2caddr = addr;
  _wire = theWire;
  return init();
}

/*!
 *   @brief  Initialise sensor with given parameters / settings
 *   @returns true on success, false otherwise
 */
bool Adafruit_BME280::begin(void) {
  bool status = false;
  _i2caddr = BME280_ADDRESS;
  _wire = &Wire;
  status = init();
  if (!status) {
    _i2caddr = BME280_ADDRESS_ALTERNATE;
    status = init();
  }
  return status;
}

/*!
 *   @brief  Initialise sensor with given parameters / settings
 *   @returns true on success, false otherwise
 */
bool Adafruit_BME280::init() {
  // init I2C or SPI sensor interface
  if (_cs == -1) {
    // I2C
    //_wire->begin(); //gereic 23.05.2022
  } else {
    digitalWrite(_cs, HIGH);
    pinMode(_cs, OUTPUT);
    if (_sck == -1) {
      // hardware SPI
      _spi->begin();
    } else {
      // software SPI
      pinMode(_sck, OUTPUT);
      pinMode(_mosi, OUTPUT);
      pinMode(_miso, INPUT);
    }
  }

  
  // check if sensor, i.e. the chip ID is correct
  _sensorID = read8(BME280_REGISTER_CHIPID);
  //log_i("sensorId=0x%X",_sensorID);
  if (_sensorID == BME280CHIP_ID){
    isBME280 = true;
  }else{
    isBME280 = false;
  }
  if ((_sensorID != BME280CHIP_ID) && (_sensorID != BMP280CHIP_ID)){
    //log_e("unknown-chip-id 0x%X",_sensorID);
    return false;
  }

  // reset the device using soft-reset
  // this makes sure the IIR is off, etc.
  write8(BME280_REGISTER_SOFTRESET, 0xB6);

  // wait for chip to wake up.
  delay(300);

  // if chip is still reading calibration, delay
  while (isReadingCalibration())
    delay(100);

  readCoefficients(); // read trimming parameters, see DS 4.2.2

  setSampling(); // use defaults

  delay(100);

  return true;
}

/*!
 *   @brief  setup sensor with given parameters / settings
 *
 *   This is simply a overload to the normal begin()-function, so SPI users
 *   don't get confused about the library requiring an address.
 *   @param mode the power mode to use for the sensor
 *   @param tempSampling the temp samping rate to use
 *   @param pressSampling the pressure sampling rate to use
 *   @param humSampling the humidity sampling rate to use
 *   @param filter the filter mode to use
 *   @param duration the standby duration to use
 */
void Adafruit_BME280::setSampling(sensor_mode mode,
                                  sensor_sampling tempSampling,
                                  sensor_sampling pressSampling,
                                  sensor_sampling humSampling,
                                  sensor_filter filter,
                                  standby_duration duration) {
  _measReg.mode = mode;
  _measReg.osrs_t = tempSampling;
  _measReg.osrs_p = pressSampling;

  _humReg.osrs_h = humSampling;
  _configReg.filter = filter;
  _configReg.t_sb = duration;

  // you must make sure to also set REGISTER_CONTROL after setting the
  // CONTROLHUMID register, otherwise the values won't be applied (see
  // DS 5.4.3)
  if (isBME280){
    write8(BME280_REGISTER_CONTROLHUMID, _humReg.get());
  }
  write8(BME280_REGISTER_CONFIG, _configReg.get());
  write8(BME280_REGISTER_CONTROL, _measReg.get());
}

/*!
 *   @brief  Encapsulate hardware and software SPI transfer into one
 * function
 *   @param x the data byte to transfer
 *   @returns the data byte read from the device
 */
uint8_t Adafruit_BME280::spixfer(uint8_t x) {
  // hardware SPI
  if (_sck == -1)
    return _spi->transfer(x);

  // software SPI
  uint8_t reply = 0;
  for (int i = 7; i >= 0; i--) {
    reply <<= 1;
    digitalWrite(_sck, LOW);
    digitalWrite(_mosi, x & (1 << i));
    digitalWrite(_sck, HIGH);
    if (digitalRead(_miso))
      reply |= 1;
  }
  return reply;
}

/*!
 *   @brief  Writes an 8 bit value over I2C or SPI
 *   @param reg the register address to write to
 *   @param value the value to write to the register
 */
void Adafruit_BME280::write8(byte reg, byte value) {
  if (_cs == -1) {
    _wire->beginTransmission((uint8_t)_i2caddr);
    _wire->write((uint8_t)reg);
    _wire->write((uint8_t)value);
    _wire->endTransmission();
  } else {
    if (_sck == -1)
      _spi->beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
    digitalWrite(_cs, LOW);
    spixfer(reg & ~0x80); // write, bit 7 low
    spixfer(value);
    digitalWrite(_cs, HIGH);
    if (_sck == -1)
      _spi->endTransaction(); // release the SPI bus
  }
}

/*!
 *   @brief  Reads an 8 bit value over I2C or SPI
 *   @param reg the register address to read from
 *   @returns the data byte read from the device
 */
uint8_t Adafruit_BME280::read8(byte reg) {
  uint8_t value;

  if (_cs == -1) {
    _wire->beginTransmission((uint8_t)_i2caddr);
    _wire->write((uint8_t)reg);
    _wire->endTransmission();
    _wire->requestFrom((uint8_t)_i2caddr, (byte)1);
    value = _wire->read();
  } else {
    if (_sck == -1)
      _spi->beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
    digitalWrite(_cs, LOW);
    spixfer(reg | 0x80); // read, bit 7 high
    value = spixfer(0);
    digitalWrite(_cs, HIGH);
    if (_sck == -1)
      _spi->endTransaction(); // release the SPI bus
  }
  return value;
}

/*!
 *   @brief  Reads a 16 bit value over I2C or SPI
 *   @param reg the register address to read from
 *   @returns the 16 bit data value read from the device
 */
uint16_t Adafruit_BME280::read16(byte reg) {
  uint16_t value;

  if (_cs == -1) {
    _wire->beginTransmission((uint8_t)_i2caddr);
    _wire->write((uint8_t)reg);
    _wire->endTransmission();
    _wire->requestFrom((uint8_t)_i2caddr, (byte)2);
    value = (_wire->read() << 8) | _wire->read();
  } else {
    if (_sck == -1)
      _spi->beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
    digitalWrite(_cs, LOW);
    spixfer(reg | 0x80); // read, bit 7 high
    value = (spixfer(0) << 8) | spixfer(0);
    digitalWrite(_cs, HIGH);
    if (_sck == -1)
      _spi->endTransaction(); // release the SPI bus
  }

  return value;
}

/*!
 *   @brief  Reads a signed 16 bit little endian value over I2C or SPI
 *   @param reg the register address to read from
 *   @returns the 16 bit data value read from the device
 */
uint16_t Adafruit_BME280::read16_LE(byte reg) {
  uint16_t temp = read16(reg);
  return (temp >> 8) | (temp << 8);
}

/*!
 *   @brief  Reads a signed 16 bit value over I2C or SPI
 *   @param reg the register address to read from
 *   @returns the 16 bit data value read from the device
 */
int16_t Adafruit_BME280::readS16(byte reg) { return (int16_t)read16(reg); }

/*!
 *   @brief  Reads a signed little endian 16 bit value over I2C or SPI
 *   @param reg the register address to read from
 *   @returns the 16 bit data value read from the device
 */
int16_t Adafruit_BME280::readS16_LE(byte reg) {
  return (int16_t)read16_LE(reg);
}

/*!
 *   @brief  Reads a 24 bit value over I2C
 *   @param reg the register address to read from
 *   @returns the 24 bit data value read from the device
 */
uint32_t Adafruit_BME280::read24(byte reg) {
  uint32_t value;

  if (_cs == -1) {
    _wire->beginTransmission((uint8_t)_i2caddr);
    _wire->write((uint8_t)reg);
    _wire->endTransmission();
    _wire->requestFrom((uint8_t)_i2caddr, (byte)3);

    value = _wire->read();
    value <<= 8;
    value |= _wire->read();
    value <<= 8;
    value |= _wire->read();
  } else {
    if (_sck == -1)
      _spi->beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
    digitalWrite(_cs, LOW);
    spixfer(reg | 0x80); // read, bit 7 high

    value = spixfer(0);
    value <<= 8;
    value |= spixfer(0);
    value <<= 8;
    value |= spixfer(0);

    digitalWrite(_cs, HIGH);
    if (_sck == -1)
      _spi->endTransaction(); // release the SPI bus
  }

  return value;
}

/*!
 *  @brief  Take a new measurement (only possible in forced mode)
 */
void Adafruit_BME280::takeForcedMeasurement() {
  // If we are in forced mode, the BME sensor goes back to sleep after each
  // measurement and we need to set it to forced mode once at this point, so
  // it will take the next measurement and then return to sleep again.
  // In normal mode simply does new measurements periodically.
  if (_measReg.mode == MODE_FORCED) {
    // set to forced mode, i.e. "take next measurement"
    write8(BME280_REGISTER_CONTROL, _measReg.get());
    // wait until measurement has been completed, otherwise we would read
    // the values from the last measurement
    while (read8(BME280_REGISTER_STATUS) & 0x08)
      delay(1);
  }
}

/*!
 *   @brief  Reads the factory-set coefficients
 */
void Adafruit_BME280::readCoefficients(void) {
  _bme280_calib.dig_T1 = read16_LE(BME280_REGISTER_DIG_T1);
  _bme280_calib.dig_T2 = readS16_LE(BME280_REGISTER_DIG_T2);
  _bme280_calib.dig_T3 = readS16_LE(BME280_REGISTER_DIG_T3);

  _bme280_calib.dig_P1 = read16_LE(BME280_REGISTER_DIG_P1);
  _bme280_calib.dig_P2 = readS16_LE(BME280_REGISTER_DIG_P2);
  _bme280_calib.dig_P3 = readS16_LE(BME280_REGISTER_DIG_P3);
  _bme280_calib.dig_P4 = readS16_LE(BME280_REGISTER_DIG_P4);
  _bme280_calib.dig_P5 = readS16_LE(BME280_REGISTER_DIG_P5);
  _bme280_calib.dig_P6 = readS16_LE(BME280_REGISTER_DIG_P6);
  _bme280_calib.dig_P7 = readS16_LE(BME280_REGISTER_DIG_P7);
  _bme280_calib.dig_P8 = readS16_LE(BME280_REGISTER_DIG_P8);
  _bme280_calib.dig_P9 = readS16_LE(BME280_REGISTER_DIG_P9);

  _bme280_calib.dig_H1 = read8(BME280_REGISTER_DIG_H1);
  _bme280_calib.dig_H2 = readS16_LE(BME280_REGISTER_DIG_H2);
  _bme280_calib.dig_H3 = read8(BME280_REGISTER_DIG_H3);
  _bme280_calib.dig_H4 = (read8(BME280_REGISTER_DIG_H4) << 4) |
                         (read8(BME280_REGISTER_DIG_H4 + 1) & 0xF);
  _bme280_calib.dig_H5 = (read8(BME280_REGISTER_DIG_H5 + 1) << 4) |
                         (read8(BME280_REGISTER_DIG_H5) >> 4);
  _bme280_calib.dig_H6 = (int8_t)read8(BME280_REGISTER_DIG_H6);
}

/*!
 *   @brief return true if chip is busy reading cal data
 *   @returns true if reading calibration, false otherwise
 */
bool Adafruit_BME280::isReadingCalibration(void) {
  uint8_t const rStatus = read8(BME280_REGISTER_STATUS);

  return (rStatus & (1 << 0)) != 0;
}

/*!
 *   @brief  Returns the temperature from the sensor
 *   @returns the temperature read from the device
 */
float Adafruit_BME280::readTemperature(void) {
  int32_t var1, var2;

  int32_t adc_T = read24(BME280_REGISTER_TEMPDATA);
  if (adc_T == 0x800000) // value in case temp measurement was disabled
    return NAN;
  adc_T >>= 4;

  var1 = ((((adc_T >> 3) - ((int32_t)_bme280_calib.dig_T1 << 1))) *
          ((int32_t)_bme280_calib.dig_T2)) >>
         11;

  var2 = (((((adc_T >> 4) - ((int32_t)_bme280_calib.dig_T1)) *
            ((adc_T >> 4) - ((int32_t)_bme280_calib.dig_T1))) >>
           12) *
          ((int32_t)_bme280_calib.dig_T3)) >>
         14;

  t_fine = var1 + var2;

  float T = (t_fine * 5 + 128) >> 8;
  return T / 100;
}

/*!
 *   @brief  Returns the pressure from the sensor
 *   @returns the pressure value (in Pascal) read from the device
 */
float Adafruit_BME280::readPressure(void) {
  int64_t var1, var2, p;

  readTemperature(); // must be done first to get t_fine

  int32_t adc_P = read24(BME280_REGISTER_PRESSUREDATA);
  if (adc_P == 0x800000) // value in case pressure measurement was disabled
    return NAN;
  adc_P >>= 4;

  var1 = ((int64_t)t_fine) - 128000;
  var2 = var1 * var1 * (int64_t)_bme280_calib.dig_P6;
  var2 = var2 + ((var1 * (int64_t)_bme280_calib.dig_P5) << 17);
  var2 = var2 + (((int64_t)_bme280_calib.dig_P4) << 35);
  var1 = ((var1 * var1 * (int64_t)_bme280_calib.dig_P3) >> 8) +
         ((var1 * (int64_t)_bme280_calib.dig_P2) << 12);
  var1 =
      (((((int64_t)1) << 47) + var1)) * ((int64_t)_bme280_calib.dig_P1) >> 33;

  if (var1 == 0) {
    return 0; // avoid exception caused by division by zero
  }
  p = 1048576 - adc_P;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = (((int64_t)_bme280_calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
  var2 = (((int64_t)_bme280_calib.dig_P8) * p) >> 19;

  p = ((p + var1 + var2) >> 8) + (((int64_t)_bme280_calib.dig_P7) << 4);
  return (float)p / 256;
}

/*!
 *  @brief  Returns the humidity from the sensor
 *  @returns the humidity value read from the device
 */
float Adafruit_BME280::readHumidity(void) {
  readTemperature(); // must be done first to get t_fine

  int32_t adc_H = read16(BME280_REGISTER_HUMIDDATA);
  if (adc_H == 0x8000) // value in case humidity measurement was disabled
    return NAN;

  int32_t v_x1_u32r;

  v_x1_u32r = (t_fine - ((int32_t)76800));

  v_x1_u32r = (((((adc_H << 14) - (((int32_t)_bme280_calib.dig_H4) << 20) -
                  (((int32_t)_bme280_calib.dig_H5) * v_x1_u32r)) +
                 ((int32_t)16384)) >>
                15) *
               (((((((v_x1_u32r * ((int32_t)_bme280_calib.dig_H6)) >> 10) *
                    (((v_x1_u32r * ((int32_t)_bme280_calib.dig_H3)) >> 11) +
                     ((int32_t)32768))) >>
                   10) +
                  ((int32_t)2097152)) *
                     ((int32_t)_bme280_calib.dig_H2) +
                 8192) >>
                14));

  v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
                             ((int32_t)_bme280_calib.dig_H1)) >>
                            4));

  v_x1_u32r = (v_x1_u32r < 0) ? 0 : v_x1_u32r;
  v_x1_u32r = (v_x1_u32r > 419430400) ? 419430400 : v_x1_u32r;
  float h = (v_x1_u32r >> 12);
  return h / 1024.0;
}

/*!
 *   Calculates the altitude (in meters) from the specified atmospheric
 *   pressure (in hPa), and sea-level pressure (in hPa).
 *   @param  seaLevel      Sea-level pressure in hPa
 *   @returns the altitude value read from the device
 */
float Adafruit_BME280::readAltitude(float seaLevel) {
  // Equation taken from BMP180 datasheet (page 16):
  //  http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf

  // Note that using the equation from wikipedia can give bad results
  // at high altitude. See this thread for more information:
  //  http://forums.adafruit.com/viewtopic.php?f=22&t=58064

  float atmospheric = readPressure() / 100.0F;
  return 44330.0 * (1.0 - pow(atmospheric / seaLevel, 0.1903));
}

/*!
 *   Calculates the pressure at sea level (in hPa) from the specified
 * altitude (in meters), and atmospheric pressure (in hPa).
 *   @param  altitude      Altitude in meters
 *   @param  atmospheric   Atmospheric pressure in hPa
 *   @returns the pressure at sea level (in hPa) from the specified altitude
 */
float Adafruit_BME280::seaLevelForAltitude(float altitude, float atmospheric) {
  // Equation taken from BMP180 datasheet (page 17):
  //  http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf

  // Note that using the equation from wikipedia can give bad results
  // at high altitude. See this thread for more information:
  //  http://forums.adafruit.com/viewtopic.php?f=22&t=58064

  return atmospheric / pow(1.0 - (altitude / 44330.0), 5.255);
}

/*!
 *   Returns Sensor ID found by init() for diagnostics
 *   @returns Sensor ID 0x60 for BME280, 0x56, 0x57, 0x58 BMP280
 */
uint32_t Adafruit_BME280::sensorID(void) { return _sensorID; }

/*
 * function reads adc-values and computes Temperature pressure and humidity
 */
uint8_t Adafruit_BME280::readADCValues(void){
  takeForcedMeasurement();
  if (_cs == -1) {
    //Serial.print(F("1"));
    _wire->beginTransmission((uint8_t)_i2caddr);
    //Serial.print(F("2"));
    _wire->write((uint8_t)BME280_REGISTER_PRESSUREDATA);
    //Serial.print(F("3"));
    _wire->endTransmission();
    //Serial.print(F("4"));
    _wire->requestFrom((uint8_t)_i2caddr, (byte)8);
    //Serial.print(F("5"));

    adc_P = _wire->read();
    adc_P <<= 8;
    adc_P |= _wire->read();
    adc_P <<= 8;
    adc_P |= _wire->read();
    adc_P >>= 4;
    adc_T = _wire->read();
    adc_T <<= 8;
    adc_T |= _wire->read();
    adc_T <<= 8;
    adc_T |= _wire->read();
    adc_T >>= 4;
    if (isBME280){
      adc_H = _wire->read();
      adc_H <<= 8;
      adc_H |= _wire->read();  
    }
  } else {
    if (_sck == -1)
      _spi->beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
    digitalWrite(_cs, LOW);
    spixfer(BME280_REGISTER_PRESSUREDATA | 0x80); // read, bit 7 high

    adc_P = spixfer(0);
    adc_P <<= 8;
    adc_P |= spixfer(0);
    adc_P <<= 8;
    adc_P |= spixfer(0);
    adc_P >>= 4;
    adc_T = spixfer(0);
    adc_T <<= 8;
    adc_T |= spixfer(0);
    adc_T <<= 8;
    adc_T |= spixfer(0);
    adc_T >>= 4;
    if (isBME280){
      adc_H = spixfer(0);
      adc_H <<= 8;
      adc_H |= spixfer(0);  
    }
    digitalWrite(_cs, HIGH);
    if (_sck == -1)
      _spi->endTransaction(); // release the SPI bus
  }
  if ((adc_P == 0) && (adc_T == 0) && (adc_H == 0)){
	  // no communication with device --> error
	  //Serial.println(F("10"));
    return 1;
  }else{
	  //all ok --> calc all values
    //Serial.print(F("6"));
	  if (_measReg.osrs_t != SAMPLING_NONE){
      if (adc_T == 0x80000){
        log_e("error reading Temp");
        return 2; //error calc Temp
      } 
      iTemp = calcTemperature();
      if ((iTemp > 8000) || (iTemp < -3500)){ // > 80degC or <-35degC
        log_e("temp exceeding range t=%d",iTemp);
        return 3; //error calc Temp
      }
    }else{
      iTemp = 0;
    }
    //Serial.print(F("7"));
    if (_measReg.osrs_p != SAMPLING_NONE){
      if (adc_P == 0x80000){
        log_e("error reading pressure");
        return 4; //error calc pressure
      } 
      uiPressure = calcPressure();
      if ((uiPressure > 109000) || (uiPressure < 35000)){ // >1090hPa or <350hPa
        log_e("pressure exceeding range t=%d",uiPressure);
        return 5; //error calc Temp
      }
    }else{
      uiPressure = 0;
    }
    //Serial.print(F("8"));
	  if ((isBME280) && (_humReg.osrs_h != SAMPLING_NONE)){
      if (adc_H == 0x8000){
        log_e("error reading humidity");
        return 6; //error calc Humidity
      } 
      uiHumidity = calcHumidity();
      if (uiHumidity > 11000){ // >110%rH
        log_e("humidity exceeding range t=%d",uiHumidity);
        return 5; //error calc Temp
      }
    }else{
      uiHumidity = 0;
    }
    //Serial.print(F("11"));
  	return 0;
  }
}
int32_t Adafruit_BME280::calcTemperature(void){
  if (adc_T == 0x80000) // value in case Temperature measurement was disabled
    return -1;
  int32_t var1, var2, T;
  var1 = ((((adc_T>>3) - ((int32_t)_bme280_calib.dig_T1<<1))) * ((int32_t)_bme280_calib.dig_T2)) >> 11;
  var2 = (((((adc_T>>4) - ((int32_t)_bme280_calib.dig_T1)) * ((adc_T>>4) - ((int32_t)_bme280_calib.dig_T1))) >> 12) *
  ((int32_t)_bme280_calib.dig_T3)) >> 14;
  t_fine = var1 + var2;
  T = (t_fine * 5 + 128) >> 8;
  return T;
}
uint32_t Adafruit_BME280::calcPressure(void){
  if (adc_P == 0x80000) // value in case pressure measurement was disabled
    return -1;
  BME280_S32_t var1, var2;
  BME280_U32_t p;
  var1 = (((BME280_S32_t)t_fine)>>1) - (BME280_S32_t)64000;
  var2 = (((var1>>2) * (var1>>2)) >> 11 ) * ((BME280_S32_t)_bme280_calib.dig_P6);
  var2 = var2 + ((var1*((BME280_S32_t)_bme280_calib.dig_P5))<<1);
  var2 = (var2>>2)+(((BME280_S32_t)_bme280_calib.dig_P4)<<16);
  var1 = (((_bme280_calib.dig_P3 * (((var1>>2) * (var1>>2)) >> 13 )) >> 3) + ((((BME280_S32_t)_bme280_calib.dig_P2) * var1)>>1))>>18;
  var1 =((((32768+var1))*((BME280_S32_t)_bme280_calib.dig_P1))>>15);
  if (var1 == 0){
    return 0; // avoid exception caused by division by zero
  }
  p = (((BME280_U32_t)(((BME280_S32_t)1048576)-adc_P)-(var2>>12)))*3125;
  if (p < 0x80000000){
    p = (p << 1) / ((BME280_U32_t)var1);
  }else{
    p = (p / (BME280_U32_t)var1) * 2;
  }
  var1 = (((BME280_S32_t)_bme280_calib.dig_P9) * ((BME280_S32_t)(((p>>3) * (p>>3))>>13)))>>12;
  var2 = (((BME280_S32_t)(p>>2)) * ((BME280_S32_t)_bme280_calib.dig_P8))>>13;
  p = (BME280_U32_t)((BME280_S32_t)p + ((var1 + var2 + _bme280_calib.dig_P7) >> 4));
  return p;
}
uint32_t Adafruit_BME280::calcHumidity(void){
  if (adc_H == 0x8000) // value in case Humidity measurement was disabled
    return -1;

  int32_t v_x1_u32r;
  v_x1_u32r = (t_fine - ((int32_t)76800));

  v_x1_u32r = (((((adc_H << 14) - (((int32_t)_bme280_calib.dig_H4) << 20) -
              (((int32_t)_bme280_calib.dig_H5) * v_x1_u32r)) + ((int32_t)16384)) >> 15) *
              (((((((v_x1_u32r * ((int32_t)_bme280_calib.dig_H6)) >> 10) *
              (((v_x1_u32r * ((int32_t)_bme280_calib.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) +
              ((int32_t)2097152)) * ((int32_t)_bme280_calib.dig_H2) + 8192) >> 14));

  v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
              ((int32_t)_bme280_calib.dig_H1)) >> 4));

  v_x1_u32r = (v_x1_u32r < 0) ? 0 : v_x1_u32r;
  v_x1_u32r = (v_x1_u32r > 419430400) ? 419430400 : v_x1_u32r;
  return  (uint32_t)((v_x1_u32r>>12) * 100 / 1024);
}  

int32_t Adafruit_BME280::getTemp(void){
  return iTemp;
}
uint32_t Adafruit_BME280::getPressure(void){
  return uiPressure;
}
uint32_t Adafruit_BME280::getHumidity(void){
  return uiHumidity;
}
