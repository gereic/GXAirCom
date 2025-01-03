#include <GxMqtt.h>

extern struct SettingsData setting;
extern struct statusData status;

GxMqtt::GxMqtt(){
  xMutex = NULL;
  updCmd[0] = 0;
  updstate[0] = 0;
}

void GxMqtt::callback(char* topic, byte* payload, unsigned int length) {
  //Serial.print(topic);
  //Serial.print("=");
  //Serial.printf("%s\n",(char *)&payload[0]);
  if (!strcmp(&cmdTopic[0],topic)){
    //Serial.println("!! CMD !!");
    if (length < sizeof(lastCmd)){ //check, if payload fits
      memcpy(&lastCmd[0],payload,length); //copy payload
      lastCmd[length] = 0; //set zero-termination
    } 
  }else if (!strcmp(&updTopic[0],topic)){
    memcpy(updCmd,payload,sizeof(updCmd));
    uint8_t cmd= payload[0] & 0x0F;
    uint8_t highNibble = (payload[0] & 0xF0) >> 4;  
    static uint8_t updateState = 0; 
    static uint8_t msgIndex = 0; 
    if (cmd == 0x01){
      //Serial.printf("upd start len=%d\n",length);
      WebUpdateRunning = true;
      Update.end(true); //end any update
      fileLen = ((uint32_t)payload[1] << 24) + ((uint32_t)payload[2] << 16) + ((uint32_t)payload[3] << 8) + (uint32_t)payload[4];
      rFileLen = 0;
      if (highNibble == 0x00){
        if (!Update.begin(0x30000,U_SPIFFS)) {
          Update.printError(Serial);
        }else{
          updateState = 1; //startOk
        }
        snprintf(updstate,sizeof(updstate),"start update spiffs len=%d",fileLen);
        Serial.println(updstate);
        //sendState(updstate);
      }else if (highNibble == 0x01){
        uint32_t free_space = (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;
        if (!Update.begin(free_space,U_FLASH)) {
          Update.printError(Serial);
        }else{
          updateState = 1; //startOk
        }       
        snprintf(updstate,sizeof(updstate),"start update flash len=%d",fileLen);
        Serial.println(updstate);
        //sendState(updstate);

      }
      //pPubSubClient->publish(&updStateTopic[0],payload,5,false);
      msgCnt = 1;
      msgIndex = 1;
    }else if (cmd == 0x02){
      //Serial.printf("upd %d len=%d\n",highNibble,length);
      if (updateState == 1){
        updateState = 2;
      }
      if (updateState == 2){
        if (msgIndex == highNibble){
          rFileLen+=length-2;
          if (Update.write(&payload[2], length-2) != (length-2)) {
            Update.printError(Serial);
          }
          msgIndex++;
          msgIndex = msgIndex & 0x0F; //only 4 Bits  
          msgCnt++;              
          snprintf(updstate,sizeof(updstate),"update %d%% finished",payload[1]);
          Serial.println(updstate);
        }else{
          snprintf(updstate,sizeof(updstate),"wrong msgindex %d=%d",msgIndex,highNibble);
          Serial.println(updstate);
        }
      }else{
        Serial.println("error, update not started");
      }
      //sendState(updstate);
      //pPubSubClient->publish(&updStateTopic[0],payload,2,false);
    }else if (cmd == 0x03){
      msgCnt++;
      if (updateState == 2){
        updateState = 3;
      }
      if (updateState == 3){
        uint32_t rMsgCnt = ((uint32_t)payload[1] << 24) + ((uint32_t)payload[2] << 16) + ((uint32_t)payload[3] << 8) + (uint32_t)payload[4];
        Serial.printf("upd end msgCnt=%d:%d,len=%d\n",msgCnt,rMsgCnt,length);
        if ((rMsgCnt == msgCnt) && (fileLen == rFileLen)){
          //pPubSubClient->publish(&updStateTopic[0],payload,1,false);
          if (Update.end(true)){
            Serial.printf("Update complete msgCnt=%d:%d len=%d:%d\n",msgCnt,rMsgCnt,fileLen,rFileLen);  
            restartNow = true;    
            snprintf(updstate,sizeof(updstate),"Update complete --> restarting");
            sendState(updstate);
          } else {
            Serial.printf("Update error\n");  
            Update.printError(Serial);
          }
        }else{
          snprintf(updstate,sizeof(updstate),"Update error msgCnt=%d:%d len=%d:%d",msgCnt,rMsgCnt,fileLen,rFileLen);
          //sendState(updstate);
          //pPubSubClient->publish(&updStateTopic[0],payload,5,false);
        }
      }else{
        Serial.println("error update was not started");
      }
    }
    
  } 
}



bool GxMqtt::begin(SemaphoreHandle_t *_xMutex,Client *_client){
  log_i("begin MQTT");
  lastCmd[0] = 0; //clear lastCmd
  if (_xMutex == NULL){ //create new Mutex
    log_i("create Mutex");
    xMutex = new SemaphoreHandle_t();
    *xMutex = xSemaphoreCreateMutex();
  }else{
    log_i("using exsiting Mutex");
    xMutex = _xMutex;
  }
  if (_client == NULL){
    pClient = new WiFiClient(); //we create our own client
    log_i("set wifi-client");
  }else{
    pClient = _client;
  }
  pPubSubClient = new PubSubClient(*pClient);
  pPubSubClient->setBufferSize(11000); //set packetsize to 11000Bytes
  pPubSubClient->setServer(setting.mqtt.server.c_str(), setting.mqtt.port);
  pPubSubClient->setCallback([this] (char* topic, byte* payload, unsigned int length) { this->callback(topic, payload, length); });
  pPubSubClient->setKeepAlive(900); //set timeout to 15min
  
  return true;
}

void GxMqtt::sendTopic(const char* topic,const char* payload, boolean retained){
  char sendTopic[100];
  snprintf(sendTopic,sizeof(sendTopic),"%s/%s",myTopic,topic);
  pPubSubClient->publish(sendTopic,payload,retained);
}

void GxMqtt::sendState(const char *c){
  sendTopic("state",c,false);
}

void GxMqtt::sendOnlineTopic(uint8_t state){
  char sMqttState[5];
  sprintf(sMqttState, "%d", state);
  sendTopic("online",sMqttState,true);
}

int16_t GxMqtt::getLastCmd(char *c,uint16_t len){
  int16_t ret = snprintf(c,len,lastCmd);
  lastCmd[0] = 0; //clear lastCmd
  return ret;
}

void GxMqtt::sendInfo(){
  static uint32_t tOld = millis() - 60000;
  uint32_t tAct = millis();
  uint32_t tDiff = tAct - tOld;
  if (tDiff >= 60000){
    StaticJsonDocument<500> doc; //Memory pool
    char msg_buf[500];
    doc.clear();
    doc["battPerc"] = status.battery.percent;
    doc["battV"] = float(status.battery.voltage)/1000.0;
    serializeJson(doc, msg_buf);
    sendTopic("info",msg_buf,false); //send topic info
    tOld = tAct;
  }
}

void GxMqtt::sendGPS(){
  StaticJsonDocument<500> doc; //Memory pool
  char msg_buf[500];
  doc.clear();
  doc["lat"] = status.gps.Lat;
  doc["lon"] = status.gps.Lon;
  doc["alt"] = status.gps.alt;
  serializeJson(doc, msg_buf);
  sendTopic("gps",msg_buf,true); //send topic gps
}

void GxMqtt::subscribe(){
    pPubSubClient->publish(willTopic,"1",true); //retained
    pPubSubClient->publish(stateTopic,"");
    sendTopic("name",setting.PilotName.c_str(),true); //send topic name
    StaticJsonDocument<500> doc; //Memory pool
    char msg_buf[500];
    doc.clear();
    doc["appVersion"] = VERSION;
    doc["buildDate"] = String(compile_date);
    doc["sdkVersion"] = String(ESP.getSdkVersion());
    doc["img"] = ENV;
    serializeJson(doc, msg_buf);
    sendTopic("version",msg_buf,true); //send topic version
    sendGPS();
    pPubSubClient->subscribe(cmdTopic);
    //subscribe to update topic    
    pPubSubClient->subscribe(updTopic);
    log_i("subscription finished");
    connState = 100;
}

 void GxMqtt::connect() {
  log_i("Attempting MQTT connection...");
  connState = 0;
  // Attempt to connect
  char myDevId[100];
  snprintf(myDevId,sizeof(myDevId),"GXAirCom-%s",setting.myDevId.c_str());
  snprintf(willTopic,sizeof(willTopic),"GXAirCom/%s/online",setting.myDevId.c_str());
  snprintf(stateTopic,sizeof(stateTopic),"GXAirCom/%s/state",setting.myDevId.c_str());
  snprintf(cmdTopic,sizeof(cmdTopic),"GXAirCom/%s/cmd",setting.myDevId.c_str());
  snprintf(myTopic,sizeof(myTopic),"GXAirCom/%s",setting.myDevId.c_str());
  snprintf(updTopic,sizeof(updTopic),"GXAirCom/%s/upd/cmd",setting.myDevId.c_str());
  snprintf(updStateTopic,sizeof(updStateTopic),"GXAirCom/%s/upd/state",setting.myDevId.c_str());
  snprintf(infoTopic,sizeof(infoTopic),"GXAirCom/%s/info",setting.myDevId.c_str());
  
  bool bRet;
  if (setting.mqtt.pw.length() == 0){
    //log_i("connect without user and password");
    bRet = pPubSubClient->connect(myDevId,willTopic,2,true,"0"); //connect without user and password
  }else{
    //log_i("connect with user and password");
    bRet = pPubSubClient->connect(myDevId,setting.myDevId.c_str(),setting.mqtt.pw.c_str(),willTopic,2,true,"0"); //connect with user and password
  }
  
  if (bRet) {
    log_i("connected as %s",myDevId);    
    connState = 10;
  } else {
    log_i("failed, rc=%d --> try again in 1min",pPubSubClient->state());
    pPubSubClient->disconnect();
  }
}

void GxMqtt::sendUpdateCmd(){
  if (updCmd[0]){
    pPubSubClient->publish(&updStateTopic[0],updCmd,5,false);
    if (updstate){
      sendState(updstate);
      updstate[0] = 0;
    }    
    updCmd[0] = 0;
  }
}

void GxMqtt::run(bool bNetworkOk){
  uint32_t tAct = millis();
  static uint32_t tOld = millis();
  static uint32_t tRestart = millis();
  static uint32_t tRestartModem = millis();
  if (bNetworkOk){
    //log_i("client connected");
    //log_i("take mutex");
    if (xSemaphoreTake( *xMutex, ( TickType_t ) 500 ) == pdTRUE ){
      if ((tAct - tOld) >= 5000){
        if (!pPubSubClient->connected()){
          connect(); //we try to connect
          tOld = millis();
        }else if (connState == 10){
          subscribe();
        }   
      }
      pPubSubClient->loop();
      if (connState == 100){ //full-connected
        sendUpdateCmd();
        sendInfo();
        tRestartModem = tAct;
      }
      //log_i("give mutex");
      xSemaphoreGive( *xMutex );
    }
    status.MqttStat = connState;
  }else{
    tOld = tAct;
    //tRestartModem = tAct;
    status.MqttStat = 0;
  }
  if ((tAct - tRestartModem) >= 600000){ //10min. no MQTT-connection, maybe no network-connection
    log_e("no MQTT-connection after 10min --> restart");
    restartNow = true; //we have not MQTT-connection --> restart ESP, because we need to reconnect wifi or modem
    tRestartModem = tAct;
  }  
  if (restartNow){
    if ((tAct - tRestart) >= 3000){
      ESP.restart();
    }    
  }else{
    tRestart = tAct;
  }
}

void GxMqtt::end(void){
}