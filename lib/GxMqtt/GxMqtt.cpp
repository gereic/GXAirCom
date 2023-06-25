#include <GxMqtt.h>

extern struct SettingsData setting;
extern struct statusData status;

GxMqtt::GxMqtt(){
  xMutex = NULL;
}

void GxMqtt::callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i=0;i<length;i++) {
    Serial.print((char)payload[i]);
    if (i < GXMQTTMAXSTRING){
      lastCmd[i] = (char)payload[i];
    }
  }
  if (length < GXMQTTMAXSTRING){
    lastCmd[length] = 0;
  }else{
    lastCmd[GXMQTTMAXSTRING-1] = 0;
  }
  
  Serial.println();
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
  pPubSubClient->setServer(setting.mqtt.server.c_str(), setting.mqtt.port);
  pPubSubClient->setCallback([this] (char* topic, byte* payload, unsigned int length) { this->callback(topic, payload, length); });
  pPubSubClient->setKeepAlive(900); //set timeout to 15min
  return true;
}

void GxMqtt::sendTopic(const char* topic,const char* payload, boolean retained){
  char sendTopic[100];
  snprintf(sendTopic,sizeof(sendTopic),"%s/%s",myTopic,topic);
  xSemaphoreTake( *xMutex, portMAX_DELAY );
  pPubSubClient->publish(sendTopic,payload,retained);
  xSemaphoreGive( *xMutex );
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

void GxMqtt::connect() {
  log_i("Attempting MQTT connection...");
  // Attempt to connect
  char myDevId[100];
  char willTopic[100];
  char cmdTopic[100];
  snprintf(myDevId,sizeof(myDevId),"GXAirCom-%s",setting.myDevId.c_str());
  snprintf(willTopic,sizeof(willTopic),"GXAirCom/%s/online",setting.myDevId.c_str());
  snprintf(cmdTopic,sizeof(cmdTopic),"GXAirCom/%s/cmd",setting.myDevId.c_str());
  snprintf(stateTopic,sizeof(stateTopic),"GXAirCom/%s/state",setting.myDevId.c_str());
  snprintf(myTopic,sizeof(myTopic),"GXAirCom/%s",setting.myDevId.c_str());
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
    delay(2000); //wait 2 seconds until will-topic is sent
    pPubSubClient->publish(willTopic,"1",true); //retained
    pPubSubClient->publish(stateTopic,"");
    //log_i("subscribe to topic %s",cmdTopic);
    pPubSubClient->subscribe(cmdTopic);
  } else {
    log_i("failed, rc=%d --> try again in 1min",pPubSubClient->state());
  }
}



void GxMqtt::run(bool bNetworkOk){
  uint32_t tAct = millis();
  static uint32_t tOld = millis() - 60000;
  if (bNetworkOk){
    //log_i("client connected");
    xSemaphoreTake( *xMutex, portMAX_DELAY );
    if ((tAct - tOld) >= 60000){
      if (!pPubSubClient->connected()){
        connect(); //we try to connect
        tOld = millis();
      }    
    }
    pPubSubClient->loop();
    //pClient->print("");  
    xSemaphoreGive( *xMutex );
  }else{
    tOld = tAct - 55000;
  }
}

void GxMqtt::end(void){
}