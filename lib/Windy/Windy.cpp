#include <Windy.h>


Windy::Windy(){
    client = NULL;
    xMutex = NULL;
    MyClient = NULL;
}

Windy::~Windy(){
  if (MyClient != NULL){
    delete MyClient;
  }
}

void Windy::setClient(Client *_client){
    client = _client;
}

void Windy::setMutex(SemaphoreHandle_t *_xMutex){
    xMutex = _xMutex;
}

bool  Windy::sendData(String ID,String APIKEY,wData *data){ //send Data to WU with Station-ID and Station-Key
  //time_t now;
  bool bRet = true;
  if (xMutex == NULL){ //create new Mutex
      xMutex = new SemaphoreHandle_t();
      *xMutex = xSemaphoreCreateMutex();
  }
  //time(&now);
  char msg[1024]; //make this big enough to hold the resulting string
  char msg2[255]; //make this big enough to hold the resulting string
  float dewpoint = dewPointFast(data->temp,data->humidity);
  //sprintf(msg,"http://weatherstation.wunderground.com/weatherstation/updateweatherstation.php?ID=%s&PASSWORD=%s&dateutc=%04d-%02d-%02d+%02d%%3A%02d%%3A%02d",
  snprintf(msg,sizeof(msg),"/pws/update/%s?stationId=%s&dateutc=%04d-%02d-%02d+%02d%%3A%02d%%3A%02d",
          APIKEY.c_str(),
          ID.c_str(),          
          year(),
          month(),
          day(),
          hour(),
          minute(),
          second()
          );
  if (data->bWind){
    snprintf(msg2,sizeof(msg2),"&winddir=%.2f&windspeedmph=%.2f&windgustmph=%.2f",
          data->winddir,
          kmh2mph(data->windspeed),
          kmh2mph(data->windgust)
          );
    strcat(msg,msg2);
  }
  if (data->bHum){
    snprintf(msg2,sizeof(msg2),"&humidity=%.2f",
          data->humidity
          );
    strcat(msg,msg2);
  }
  if (data->bTemp){
    snprintf(msg2,sizeof(msg2),"&tempf=%.2f",
          deg2f(data->temp)
          );
    strcat(msg,msg2);
  }
  if ((data->bTemp) && (data->bHum)){
    snprintf(msg2,sizeof(msg2),"&dewptf=%.2f",
          deg2f(dewpoint)
          );
    strcat(msg,msg2);
  }
  if (data->bTemp){
    snprintf(msg2,sizeof(msg2),"&baromin=%.2f",
          data->pressure * 0.029529983071445
          );
    strcat(msg,msg2);
  }
  if (data->bRain){
    snprintf(msg2,sizeof(msg2),"&rainin=%.2f",
            data->rain1h
            );
    strcat(msg,msg2);
  }  
  //log_i("%s len=%d",msg,strlen(msg));
  
  uint16_t serverPort = 80;
  if (client == NULL){   
    #ifdef SSLCONNECTION
    MyClient = new WiFiClientSecure();
    MyClient->setInsecure();
    serverPort = 443;
    #else
    MyClient = new WiFiClient();
    #endif
    client = MyClient;
    
  }
  xSemaphoreTake( *xMutex, portMAX_DELAY );
  HttpClient http(*client, "stations.windy.com",serverPort);  
  int httpResponseCode = http.get(msg);
  if (httpResponseCode == 0){
    httpResponseCode = http.responseStatusCode();
    if (httpResponseCode != 200){
      log_e("resp=%d",httpResponseCode);
      bRet = false;
    } 
  }else{
    log_e("failed to connect=%d",httpResponseCode);
    bRet = false;
  }
  http.stop();
  xSemaphoreGive( *xMutex );
  return bRet;
}