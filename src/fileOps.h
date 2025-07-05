#ifndef __FILEOPS_H__
#define __FILEOPS_H__


#include <Preferences.h>
#include "main.h"
#include "enums.h"

extern SettingsData setting;

void load_configFile(SettingsData* pSetting);
void write_configFile(SettingsData* pSetting);
void write_screenNumber(void);
void write_Volume(void);
void write_PilotName(void);
void write_AircraftType(void);
void write_AirMode(void);
void write_Mode(void);
void write_RFMode(void);
void write_OutputMode(void);
void write_LoraPower(void);
void write_fuelsensor(void);
void write_battOffset(void);
void write_battMinPerc(void);
void write_gpsBaud(void);
void write_wifiModeBits(void);
void write_CPUFrequency(void);
void write_FanetUploadInterval(void);
void write_windDirOffset(void);
void write_AnemometerType(void);
#endif