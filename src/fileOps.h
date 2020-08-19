#ifndef __FILEOPS_H__
#define __FILEOPS_H__


#include <Preferences.h>
#include "main.h"

extern SettingsData setting;

void load_configFile(void);
void write_configFile(void);
void write_screenNumber(void);
void write_Volume(void);
#endif