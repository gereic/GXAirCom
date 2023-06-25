/*!
 * @file gxUpdater.h
 *
 *
 */

#ifndef __UPDATER_H__
#define __UPDATER_H__

#include <ArduinoHttpClient.h>
#include <WiFi.h>
#include "tools.h"
#include <Update.h>

#define UPDATESERVER "gxaircom.getronix.at"
#define RELEASEDIR "/release/"
#define VERSIONFILE "version.txt"
#define NETWORKTIMEOUT 30000ul
#define NETWORKDELAY 1000ul

class gxUpdater {
public:
  typedef struct {
    uint8_t  major;
    uint8_t  minor;
    uint8_t  patch;
  } tVersion;
  gxUpdater(); //constructor
  void setClient(Client *_client);
  void setMutex(SemaphoreHandle_t *_xMutex);
  bool checkVersion(void);
  String getVersion(void);
  void setVersion(String s);
  bool checkVersionNewer(void);
  bool updateVersion(void);
private:
  Client *client;
  SemaphoreHandle_t *xMutex;    
  String _version;
  bool getVersion(tVersion *v,String s);
  bool doUpdate(bool spiffs);
  String getHeaderValue(String header, String headerName);
};

#endif