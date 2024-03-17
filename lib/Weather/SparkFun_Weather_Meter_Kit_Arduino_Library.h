#ifndef __SPARKFUN_WEATHER_METER_KIT_H__
#define __SPARKFUN_WEATHER_METER_KIT_H__

#include "Arduino.h"
#include "SparkFun_Weather_Meter_Kit_Constants.h"

// Calibration parameters for each sensor
struct SFEWeatherMeterKitCalibrationParams
{
    // Wind vane
    uint16_t vaneADCValues[WMK_NUM_ANGLES];

    // Wind speed
    uint32_t windSpeedMeasurementPeriodMillis;
    float kphPerCountPerSec;

    // Rainfall
    float mmPerRainfallCount;
    uint32_t minMillisPerRainfall;
};

class SFEWeatherMeterKit
{
public:
    // Constructor
    SFEWeatherMeterKit(uint8_t windDirectionPin, uint8_t windSpeedPin, uint8_t rainfallPin);
    static void begin();

    // Data collection
    static float getWindDirection();
    static float getWindSpeed();
    static float getTotalRainfall();

    // Sensor calibration params
    static SFEWeatherMeterKitCalibrationParams getCalibrationParams();
    static void setCalibrationParams(SFEWeatherMeterKitCalibrationParams params);

    // ADC resolution scaling
    static void setADCResolutionBits(uint8_t resolutionBits);

    // Helper functions. These can be helpful for sensor calibration
    static uint32_t getWindSpeedCounts();
    static uint32_t getRainfallCounts();
    static void resetWindSpeedFilter();
    static void resetTotalRainfall();

private:
    // Updates wind speed
    static void updateWindSpeed();

    // Interrupt handlers
    static void windSpeedInterrupt();
    static void rainfallInterrupt();

    // Pins for each sensor
    static uint8_t _windDirectionPin;
    static uint8_t _windSpeedPin;
    static uint8_t _rainfallPin;

    // Sensor calibration parameters
    static SFEWeatherMeterKitCalibrationParams _calibrationParams;

    // Variables to track measurements
    static uint32_t _windCounts;
    static uint32_t _windCountsPrevious;
    static uint32_t _rainfallCounts;
    static uint32_t _lastWindSpeedMillis;
    static uint32_t _lastRainfallMillis;
};

#endif