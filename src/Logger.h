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

// File myFile;

class Logger{
  public:
    Logger(); //constructor
    bool begin();
    void end(void);
    void run(void); //has to be called cyclic
    void listFiles(fs::FS &fs);

  private:
    bool lInit;
    bool lStop;
    bool ltest;
    char igcPAth[32];
    uint32_t gotflytime;
    void doInitLogger(const char * trackFile);
    void updateLogger(void);
    void doStopLogger(void);
    void writeFile(fs::FS &fs, const char * path, const char * message);
    void appendFile(fs::FS &fs, const char * path, const char * message);
};

#endif
