#include <WeatherUnderground.h>


WeatherUnderground::WeatherUnderground(){
}

void WeatherUnderground::setSettings(bool windsensor,bool rainsensor){
  _windsensor = windsensor;
  _rainsensor = rainsensor;
}

bool  WeatherUnderground::sendData(String ID,String KEY,float winddir,float windspeed,float windgust, float humidity, float temp, float baro,float rain1h, float raindaily){
  time_t now;
  time(&now);
  char msg[1024]; //make this big enough to hold the resulting string
  char msg2[255]; //make this big enough to hold the resulting string
  float dewpoint = dewPointFast(temp,humidity);
  sprintf(msg,"http://weatherstation.wunderground.com/weatherstation/updateweatherstation.php?ID=%s&PASSWORD=%s&dateutc=%04d-%02d-%02d+%02d%%3A%02d%%3A%02d",
          ID.c_str(),
          KEY.c_str(),
          year(now),
          month(now),
          day(now),
          hour(now),
          minute(now),
          second(now)
          );
  if (_windsensor){
    sprintf(msg2,"&winddir=%.2f&windspeedmph=%.2f&windgustmph=%.2f",
          winddir,
          kmh2mph(windspeed),
          kmh2mph(windgust)
          );
    strcat(msg,msg2);
  }
  sprintf(msg2,"&humidity=%.2f&tempf=%.2f&baromin=%.2f&dewptf=%.2f",
          humidity,
          deg2f(temp),
          baro * 0.029529983071445,
          deg2f(dewpoint)
          );
  strcat(msg,msg2);
  if (_rainsensor){
    sprintf(msg2,"&rainin=%.2f&dailyrainin=%.2f",
            rain1h,
            raindaily
            );
    strcat(msg,msg2);
  }  
  sprintf(msg2,"&softwaretype=GXAircom-%s&action=updateraw",
          VERSION
          );
  strcat(msg,msg2);
  //log_i("T1=%f h=%f p1=%f dp=%f",temp,humidity,baro,dewpoint);
  log_i("%s len=%d",msg,strlen(msg));
  http.begin(msg);
  int httpResponseCode = http.GET();
  if (httpResponseCode != 200){
    log_e("resp=%d %s",httpResponseCode,http.getString().c_str());
    return false;
  }
  return true;
}