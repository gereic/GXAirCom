/*!
 * @file Logger.h
 *
 *
 */

extern struct SettingsData setting;
extern struct statusData status;



#ifndef __LOGGER_H__
#define __LOGGER_H__

#include <Arduino.h>
#include "FS.h"
#include "SD_MMC.h"
#include "main.h"

// Manufacturer code, use XXX if you don't have one. ABC is the unique code for this logger (serial no.?), rest of line can be anything you like
#define IGC_ROW1 "AXXXGXO "
// fix accuracy in m 
#define IGC_ROW2 "HFFXA035" 
// UTC date of flight, to be completed while saving headers eg "HFDTEDATE:081021"
#define IGC_ROW3 "HFDTEDATE:"
// Free-text name of the pilot
#define IGC_ROW4 "HFPLTPILOTINCHARGE:"
// Second pilot
#define IGC_ROW5 "HFCM2CREW2:"
// glider type eg. "HFGTYGLIDERTYPE:Nova Mentor 6"
#define IGC_ROW6 "HFGTYGLIDERTYPE:"
// competition class
#define IGC_ROW7 "HOCCLCOMPETITION CLASS:FAI-3"
// Glider ID to be set based on GxAir ID
#define IGC_ROW8 "HFGIDGLIDERID:"

// costant headers
#define IGC_ROW9 "HFDTM100GPSDATUM:WGS-84"
// Firmware version
#define IGC_ROW10 "HFRFWFIRMWAREVERSION:"
#define IGC_ROW11 "HFRHWHARDWAREVERSION:Lilygo T3 v2.1.6.1"
#define IGC_ROW12 "HFFTYFRTYPE:GxAirCom Logger by Gerald E."
// Manufacturer of the pressure sensor in the logger. Any text.
#define IGC_ROW13 "HFPRSPRESSALTSENSOR:BOSH,BMP280,max10000m"
// Manufacturer of the GPS receiver inside the logger. Do we really care? Any text will work
#define IGC_ROW14 "HFGPS:uBLOX Neo6"
//extra complusory lines
#define IGC_ROW15 "HFALG:GEO"
#define IGC_ROW16 "HOSITSite:?"
#define IGC_ROW17 "HFALPALTPRESSURE:ISA"


// File myFile;

class Logger{
  public:
    Logger(); //constructor
    bool begin();
    void end(void);
    void run(void); //has to be called cyclic
    void listFiles(fs::FS &fs, const char * dirname);
    void deleteFile(fs::FS &fs, const char * path);
    char igclist[1000];

  private:
    bool lInit;
    bool lStop;
    uint8_t ltest;

    uint32_t g_time;
    uint32_t g_latlon;
    uint32_t g_baroalt;
    uint32_t g_gpsalt;

    char igcPAth[32];
    uint32_t gotflytime;
    void doInitLogger(const char * trackFile);
    void updateLogger(void);
    void doStopLogger(void);
    void writeFile(fs::FS &fs, const char * path, const char * message);
    void appendFile(fs::FS &fs, const char * path, const char * message);
    char* igcHeaders();
};

#endif
