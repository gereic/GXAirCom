/*!
 * @file gxUpdater.cpp
 *
 *
 */

#include "gxUpdater.h"

gxUpdater::gxUpdater(){
    client = NULL;
    xMutex = NULL;
}

void gxUpdater::setClient(Client *_client){
    client = _client;
}

void gxUpdater::setMutex(SemaphoreHandle_t *_xMutex){
    xMutex = _xMutex;
}



bool gxUpdater::checkVersion(void){
  bool bRet = true;
  if (xMutex == NULL){ //create new Mutex
    xMutex = new SemaphoreHandle_t();
    *xMutex = xSemaphoreCreateMutex();
  }
  if (client == NULL){    
    client = new WiFiClient();
  }
  xSemaphoreTake( *xMutex, portMAX_DELAY );
  HttpClient http(*client, UPDATESERVER);  
  int httpResponseCode = http.get(RELEASEDIR VERSIONFILE);
  if (httpResponseCode == 0){
    httpResponseCode = http.responseStatusCode();
    if (httpResponseCode != 200){
      log_e("resp=%d",httpResponseCode);
      bRet = false;
    }else{
      int bodyLen = http.contentLength();
      log_i("response 200 --> ok %d",bodyLen);
      // Now we've got to the body, so we can print it out
      unsigned long timeoutStart = millis();
      char c;
      _version = "";
      // Whilst we haven't timed out & haven't reached the end of the body
      while ( (http.connected() || http.available()) &&
        (!http.endOfBodyReached()) &&
        ((millis() - timeoutStart) < NETWORKTIMEOUT) ){
        if (http.available()){
          c = http.read();
          _version += String(c);
          // Print out this character
          //Serial.print(c);
          // We read something, reset the timeout counter
          timeoutStart = millis();
        }else{
          // We haven't got any data, so let's pause to allow some to
          // arrive
          delay(NETWORKDELAY);
        }
      }

      //_version = http.responseBody();
      log_i("%s",_version.c_str());
      //log_i("body:%s",http.responseBody )
    } 
  }else{
    log_e("failed to connect=%d",httpResponseCode);
    bRet = false;
  }
  http.stop();
  xSemaphoreGive( *xMutex );
  return bRet;
}

String gxUpdater::getVersion(void){
  return _version;
}

bool gxUpdater::checkVersionNewer(void){
  tVersion vAct;
  tVersion vNew;
  getVersion(&vAct,VERSION);
  getVersion(&vNew,_version);
  if (vNew.major > vAct.major) return true;
  if (vNew.major == vAct.major){
    if (vNew.minor > vAct.minor) return true;
    if (vNew.minor == vAct.minor){
      if (vNew.patch > vAct.patch)return true;
    }
  }
  return false;
}

void gxUpdater::setVersion(String s){
  _version = s; //set version
}

bool gxUpdater::getVersion(tVersion *v,String s){
  bool bRet = false;
  //tVersion v;
  //version is v1.2.3 --> v255.255.255
  String sRet;
  int pos1 = 1;
  int pos2 = s.indexOf(".",pos1);  
  if (pos2 > 0){
    sRet = s.substring(pos1,pos2);
    v->major = atoi(sRet.c_str());
    pos1 = pos2+1;
    pos2 = s.indexOf(".",pos1); 
    if (pos2 > 0){      
      sRet = s.substring(pos1,pos2);
      v->minor = atoi(sRet.c_str());      
      pos1 = pos2+1;
      sRet = s.substring(pos1);
      v->patch = atoi(sRet.c_str());
      bRet = true;
    }
  }
  return bRet;
}

// used to extract header value from headers for ota update
String gxUpdater::getHeaderValue(String header, String headerName) {
  return header.substring(strlen(headerName.c_str()));
}

bool gxUpdater::doUpdate(bool spiffs){
  bool bRet = true;
  if (xMutex == NULL){ //create new Mutex
    xMutex = new SemaphoreHandle_t();
    *xMutex = xSemaphoreCreateMutex();
  }
  if (client == NULL){    
    client = new WiFiClient();
  }
  xSemaphoreTake( *xMutex, portMAX_DELAY );
  client->setTimeout(NETWORKTIMEOUT);
  HttpClient http(*client, UPDATESERVER); 
  String sFilename = "";
  int command = 0;
  if (spiffs){
    sFilename = RELEASEDIR "spiffs_" + _version + ".bin";
    log_i("updating spiffs:%s",sFilename.c_str());
    command = U_SPIFFS;
  } else{
    sFilename = RELEASEDIR "firmware_" + _version + "_" + ENV + ".bin";
    log_i("updating firmwawre:%s",sFilename.c_str());
    command = U_FLASH;
  }
  int httpResponseCode = http.get(sFilename);
  if (httpResponseCode == 0){
    httpResponseCode = http.responseStatusCode();
    if (httpResponseCode != 200){
      log_e("resp=%d",httpResponseCode);
      bRet = false;
    }else{
      int length = http.contentLength();
      log_i("response 200 --> ok %d",length);
      if (length > 0){
        if (!Update.begin(length,command)) {
          Update.printError(Serial);
          http.stop();
          client->flush();
          client->stop();
          xSemaphoreGive( *xMutex );
          return false;
        }

        uint32_t bytesRead = 0;
        // Whilst we haven't timed out & haven't reached the end of the body        
        //Serial.printf("%02X",client->read());
        bytesRead = Update.writeStream(*client);  
        log_i("bytes-read=%d",bytesRead);
        if (length == bytesRead){
          if (!Update.end(true)){
            Update.printError(Serial);
            bRet = false;
          } else {
            Serial.println("Update complete");              
          }
        }else{
          log_i("error length error %d!=%d",length,bytesRead);
          Serial.println("Error Occurred! \nError: " + String(Update.getError()) + " " + String(Update.errorString()) );
          Serial.println("Will try to update again at a different time.\n");

          bRet = false;
        }
      }
    } 
  }else{
    log_e("failed to connect=%d",httpResponseCode);
    bRet = false;
  }
  client->flush();
  http.stop();
  client->stop();
  xSemaphoreGive( *xMutex );
  return bRet;

}

bool gxUpdater::updateVersion(void){
  bool bRet = false;
  for (int i = 0;i < 3; i++){ //we try 3 times, because of gsm-modem
    if (doUpdate(false)){ //update firmware  
      bRet = true;
      break;
    }
    delay(500); //wait 500ms sec. before next
  }
  if (!bRet) return false;
  bRet = false;
  delay(500); //wait 500ms sec. before next
  for (int i = 0;i < 3; i++){ //we try 3 times, because of gsm-modem
    if (doUpdate(true)){  //update spiffs 
      bRet = true;
      break;
    }
    delay(500); //wait 500ms sec. before next
  }
  return bRet;
}