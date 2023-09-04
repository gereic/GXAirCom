#include <Holfuy.h>


Holfuy::Holfuy(){
    client = NULL;
    MyClient = NULL;
    xMutex = NULL;
}

Holfuy::~Holfuy(){
  if (MyClient != NULL){
    delete MyClient;
  }
}

void Holfuy::setClient(Client *_client){
    client = _client;
}

void Holfuy::setMutex(SemaphoreHandle_t *_xMutex){
    xMutex = _xMutex;
}


String Holfuy::httpRequest(const char *serverName, const char *path) {
  String payload = "";
  uint16_t serverPort = 80;
  if (xMutex == NULL){ //create new Mutex
      xMutex = new SemaphoreHandle_t();
      *xMutex = xSemaphoreCreateMutex();
  }
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
  HttpClient http(*client, serverName,serverPort);  
  int httpResponseCode = http.get(path);
  if (httpResponseCode == 0){
    httpResponseCode = http.responseStatusCode();
    if (httpResponseCode == 200){
      int length = http.contentLength();
      if (length >= 0) {
        payload = http.responseBody();
      }else{
        log_e("content length <= 0");
      }
    }else{
      log_e("resp=%d",httpResponseCode);
    } 
  }else{
    log_e("failed to connect=%d",httpResponseCode);
  }
  http.stop();
  xSemaphoreGive( *xMutex );

  return payload;

}

bool Holfuy::getData(String ID,String KEY,weatherUndergroundData *data){ //get Data from holfuy with Station-ID and API-Key
  char path[1024]; //make this big enough to hold the resulting string

  sprintf(path,"/live/?s=%s&pw=%s&m=JSON&tu=C&su=km/h&loc&avg=0&daily",
          ID.c_str(),
          KEY.c_str()
          );
  
  char serverName[1024] = "api.holfuy.com";
  
  String payload = Holfuy::httpRequest(serverName, path);
  //log_e("%s", payload.c_str());
  if (payload == "") return false;

  DynamicJsonDocument doc(2048);
  deserializeJson(doc, payload);
  String stationID = doc["stationId"].as<String>();
  if (stationID != ID){
    log_e("stationid not equal %s != %s",ID.c_str(),stationID.c_str());
    return false;
  }
  if (doc["location"]["latitude"]) {
    data->lat = doc["location"]["latitude"].as<float>();
  } else {
    log_e("no clue about the holfuy latitude!");
    return false;
  }
  if (doc["location"]["longitude"]) {
    data->lon = doc["location"]["longitude"].as<float>();
  } else {
    log_e("no clue about the holfuy longitude!");
    return false;
  }
  if (doc["location"]["altitude"]) {
    data->height = doc["location"]["altitude"].as<float>();
  } else {
    log_e("no clue about the holfuy altitude!");
    return false;
  }
  if (doc["temperature"]){
    data->bTemp = true;
    data->temp = doc["temperature"].as<float>();
  }else{
    data->bTemp = false;
  }
  if (doc["humidity"]){
    data->humidity = doc["humidity"].as<float>();
    data->bHum = true;
  } else {
    data->bHum = false;
  }
  String s = doc["wind"]["unit"].as<String>();
  if (s != "null"){
    data->bWind = true;
    data->winddir = doc["wind"]["direction"].as<float>();
    data->windspeed = doc["wind"]["speed"].as<float>();    
    data->windgust = doc["wind"]["gust"].as<float>();
  }else{
    log_i("no wind-data");
    data->bWind = false;
    data->winddir = 0.0;
    data->windspeed = 0.0;
    data->windgust = 0.0;
  }
  
  
  if (doc["pressure"]){
    data->bPress = true;
    data->pressure = doc["pressure"].as<float>();
  }else{
    data->bPress = false;
  }


  // s = doc["observations"][0]["metric"]["precipRate"].as<String>();
  // if (s != "null"){
  //   data->bRain = true;
  //   data->rain1h = doc["observations"][0]["metric"]["precipRate"].as<float>();
  //   data->raindaily = doc["observations"][0]["metric"]["precipTotal"].as<float>();
  // }else{
  log_i("rain data from holfuy unknown yet - need to calculate 1h values!?");
  data->bRain = false;
  data->rain1h = 0.0;
  data->raindaily = 0.0;
  //}
  return true;
}

