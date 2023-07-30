/*!
 * @file FlarmDataPort.h
 *
 *
 */

#ifndef __FLARMDATAPORT_H__
#define __FLARMDATAPORT_H__


#include <HardwareSerial.h>
#include <CalcTools.h>


enum class eFlarmAircraftType {
  UNKNOWN = 0,
  GLIDER_MOTOR_GLIDER = 1,
  TOW_PLANE = 2,
  HELICOPTER_ROTORCRAFT = 3,
  SKYDIVER = 4,
  DROP_PLANE_SKYDIVER = 5,
  HANG_GLIDER = 6,
  PARA_GLIDER = 7,
  AIRCRAFT_RECIPROCATING_ENGINE = 8,
  AIRCRAFT_JET_TURBO_ENGINE = 9,
  RESERVED10 = 10,
  BALLOON = 11,
  AIRSHIP = 12,
  UAV = 13,
  RESERVED14 = 14,
  STATIC_OBJECT = 15
};


typedef struct {
  String DevId;
  uint8_t addressType;
  float lat; //latitude
  float lon; //longitude
  uint16_t altitude; //altitude [m]
  eFlarmAircraftType aircraftType; //
  float speed; //km/h
  float climb; //m/s
  float heading; //deg
} FlarmtrackingData;


#define FLARM_NO_GPS 0
#define FLARM_GPS_FIX3d_GROUND 1
#define FLARM_GPS_FIX3d_AIR 2


class FlarmDataPort {
public:
    FlarmDataPort(); //constructor
    bool begin(void);
    void run(void); //has to be called cyclic
    uint8_t neighbors;
    uint8_t GPSState;    
    int writeFlarmData(char *buffer, size_t size,FlarmtrackingData *myData,FlarmtrackingData *movePilotData);
    int writeDataPort(char *buffer, size_t size);
    int writeVersion(char *buffer, size_t size);
    int writeSelfTestResult(char *buffer, size_t size);
    //String writeFlarmData(FlarmtrackingData *myData,FlarmtrackingData *movePilotData);
    //String writeDataPort(void);
    //String writeVersion(void);
    //String writeSelfTestResult(void);
    String addChecksum(String s);
    int addChecksum(char *buffer, size_t size);
protected:
    
    String getHexFromByte(uint8_t val);
    String getHexFromByte1(uint8_t val);
};
#endif