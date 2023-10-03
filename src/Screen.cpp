/*!
 * @file Screen.cpp
 *
 *
 */

#include "Screen.h"
#include "tools.h"
#include <icons.h>


//GxEPD2_BW<GxEPD2_290, GxEPD2_290::HEIGHT> e_ink(GxEPD2_290(EINK_CS, EINK_DC, EINK_RST, EINK_BUSY));
//GxEPD2_BW<GxEPD2_290_T94, GxEPD2_290_T94::HEIGHT> e_ink2(GxEPD2_290_T94(EINK_CS, EINK_DC, EINK_RST, EINK_BUSY));

Screen::Screen(){
}

bool Screen::begin(uint8_t type,int8_t cs,int8_t dc,int8_t rst,int8_t busy,int8_t clk, int8_t din){
  bInit = false;
  log_i("starting E-Ink type=%d;cs=%d,dc=%d,rst=%d,busy=%d,clk=%d,din=%d",type,cs,dc,rst,busy,clk,din);
  if (type == 1){
    //GxEPD2_BW<GxEPD2_290_T94, GxEPD2_290_T94::HEIGHT> *e_ink = new GxEPD2_BW<GxEPD2_290_T94, GxEPD2_290_T94::HEIGHT>(GxEPD2_290_T94(cs, dc, rst, busy));
    GxEPD2_BW<GxEPD2_290_T94_V2, GxEPD2_290_T94_V2::HEIGHT> *e_ink = new GxEPD2_BW<GxEPD2_290_T94_V2, GxEPD2_290_T94_V2::HEIGHT>(GxEPD2_290_T94_V2(cs, dc, rst, busy));
    e_ink->epd2.init(clk, din, 0, true, false); // define or replace SW_SCK, SW_MOSI)
    e_ink->init(0);
    pEInk = e_ink;
    //e_ink2.epd2.init(EINK_CLK, EINK_DIN, 0, true, false); // define or replace SW_SCK, SW_MOSI
    //e_ink2.init(0); // needed to init upper level
    //pEInk = &e_ink2;
    log_i("display-type 2.9 V2");
  }else{
    GxEPD2_BW<GxEPD2_290, GxEPD2_290::HEIGHT> *e_ink = new GxEPD2_BW<GxEPD2_290, GxEPD2_290::HEIGHT>(GxEPD2_290(cs, dc, rst, busy));
    e_ink->epd2.init(clk, din, 0, true, false); // define or replace SW_SCK, SW_MOSI)
    e_ink->init(0);
    pEInk = e_ink;
    //e_ink.epd2.init(EINK_CLK, EINK_DIN, 0, true, false); // define or replace SW_SCK, SW_MOSI
    //e_ink.init(0); // needed to init upper level
    //pEInk = &e_ink;
    log_i("display-type 2.9 V1");
  }
  //e_ink3 = new GxEPD2_290_T94(EINK_CS, EINK_DC, EINK_RST, EINK_BUSY);
  //e_ink3 = new GxEPD2_BW<GxEPD2_290_T94, GxEPD2_290_T94::HEIGHT>(GxEPD2_290_T94(EINK_CS, EINK_DC, EINK_RST, EINK_BUSY));
  return true;
}

void Screen::end(void){
  if (setting.displayRotation <= 1){
    pEInk->setRotation(1);
  }else{
    pEInk->setRotation(3);
  }  
  pEInk->setFont(&FreeMonoBold9pt7b);
  pEInk->setTextColor(GxEPD_BLACK);
  pEInk->setFullWindow();
  pEInk->firstPage();
  do
  {
    pEInk->fillScreen(GxEPD_WHITE);
    pEInk->drawXBitmap(84,34,G_Logo_bits,G_Logo_width,G_Logo_height,GxEPD_BLACK);
    pEInk->drawXBitmap(114,34,X_Logo_bits,X_Logo_width,X_Logo_height,GxEPD_BLACK);
    pEInk->drawXBitmap(153,34,AirCom_Logo_bits,AirCom_Logo_width,AirCom_Logo_height,GxEPD_BLACK);
    pEInk->setCursor(169,92);
    pEInk->print(VERSION);
    pEInk->setCursor(80,92);
    pEInk->print(setting.myDevId);
  }
  while (pEInk->nextPage());
  bInit = false;
  //pEInk->powerOff();
  pEInk->hibernate();

}

void Screen::run(void){
  //bInit = true;
  if (!bInit){
    doInitScreen();
  }else{
    if (setting.Mode == eMode::GROUND_STATION){
      drawWeatherScreen();
    }else{
      drawMainScreen();
    }
  }
}

void Screen::doInitScreen(void){
  static uint32_t tTimeStamp = millis();
  static uint8_t step = 0;
  uint32_t tAct = millis();
  switch (step)
  {
  case 0:
    if (setting.displayRotation <= 1){
      pEInk->setRotation(1);
    }else{
      pEInk->setRotation(3);
    }  
    pEInk->setFont(&FreeMonoBold9pt7b);
    pEInk->setTextColor(GxEPD_BLACK);
    pEInk->firstPage();
    do
    {
        pEInk->fillScreen(GxEPD_WHITE);
        pEInk->drawXBitmap(84,34,G_Logo_bits,G_Logo_width,G_Logo_height,GxEPD_BLACK);
    }
    while (pEInk->nextPage());
    tTimeStamp = millis();
    step ++;
    break;
  case 1:
    if (timeOver(tAct,tTimeStamp,500)){
      pEInk->setPartialWindow(0,0,pEInk->width(),pEInk->height());
      do
      {
        pEInk->fillScreen(GxEPD_WHITE);
        pEInk->drawXBitmap(84,34,G_Logo_bits,G_Logo_width,G_Logo_height,GxEPD_BLACK);
        pEInk->drawXBitmap(114,34,X_Logo_bits,X_Logo_width,X_Logo_height,GxEPD_BLACK);
      }
      while (pEInk->nextPage());
      tTimeStamp = millis();
      step ++;
    }
    break;
  case 2:
    if (timeOver(tAct,tTimeStamp,500)){
      pEInk->setPartialWindow(0,0,pEInk->width(),pEInk->height());
      //pEInk->firstPage();
      do
      {
        pEInk->fillScreen(GxEPD_WHITE);
        pEInk->drawXBitmap(84,34,G_Logo_bits,G_Logo_width,G_Logo_height,GxEPD_BLACK);
        pEInk->drawXBitmap(114,34,X_Logo_bits,X_Logo_width,X_Logo_height,GxEPD_BLACK);
        pEInk->drawXBitmap(153,34,AirCom_Logo_bits,AirCom_Logo_width,AirCom_Logo_height,GxEPD_BLACK);
        pEInk->setCursor(169,92);
        pEInk->print(VERSION);
        pEInk->setCursor(80,92);
        pEInk->print(setting.myDevId);
      }
      while (pEInk->nextPage());
      tTimeStamp = millis();
      step ++;
    }
    break;
  case 3:
    if (timeOver(tAct,tTimeStamp,2000)){
      tTimeStamp = millis();
      step ++;
    }
    break;
  default:
    step = 0;
    stepCount = 0;
    bInit = true;
    break;
  }
}

String Screen::getWDir(float dir){
    uint8_t uDir = uint8_t(dir/22.5);
    switch (uDir)
    {
    case 1:
        return "NNE";
        break;
    case 2:
        return "NE";
        break;
    case 3:
        return "ENE";
        break;
    case 4:
        return "E";
        break;
    case 5:
        return "ESE";
        break;
    case 6:
        return "SE";
        break;
    case 7:
        return "SSE";
        break;
    case 8:
        return "S";
        break;
    case 9:
        return "SSW";
        break;
    case 10:
        return "SW";
        break;
    case 11:
        return "WSW";
        break;
    case 12:
        return "W";
        break;
    case 13:
        return "WNW";
        break;
    case 14:
        return "NW";
        break;
    case 15:
        return "NNW";
        break;

    default:
        return "N";
        break;
    }
}

void Screen::drawCompass(int16_t x, int16_t y, int16_t width, int16_t height,float value){
  String sText = String(value,0);
  String sDir = getWDir(value);
  int16_t tbx, tby, posx, posy; uint16_t tbw, tbh;
  pEInk->getTextBounds(sText.c_str(), 0, height, &tbx, &tby, &tbw, &tbh);
  posx = x+width-tbw-tbx;
  posy = y+height-1;
  pEInk->setCursor(x,posy);
  pEInk->print(sDir.c_str());
  pEInk->setCursor(posx,posy);
  pEInk->print(sText.c_str());
}

void Screen::drawValue(int16_t x, int16_t y, int16_t width, int16_t height,float value,unsigned int decimals){
  String sText = String(value,decimals);
  //log_i("%.2f = %s",value,sText.c_str());
  int16_t tbx, tby, posx, posy; uint16_t tbw, tbh;
  pEInk->getTextBounds(sText.c_str(), 0, height, &tbx, &tby, &tbw, &tbh);
  posx = x+width-tbw-tbx;
  posy = y+height-1;
  //log_i("%d %d %d %d %d %d",tbx, tby, tbw, tbh,posx,posy);
  pEInk->setCursor(posx,posy);
  pEInk->print(sText.c_str());
}
void Screen::drawBatt(int16_t x, int16_t y, int16_t width, int16_t height,uint8_t value){
  static uint8_t DrawValue = 0;
  //log_i("%d",value);
  if (value == 255){
    DrawValue = (DrawValue + 1) %5;
  }else{
    DrawValue = value;
  }
  switch (DrawValue)
  {
  case 1:
    pEInk->drawInvertedBitmap(x, y, bat1icons, width, height, GxEPD_BLACK);   //GxEPD_BLACK);
    break;
  case 2:
    pEInk->drawInvertedBitmap(x, y, bat2icons, width, height, GxEPD_BLACK);   //GxEPD_BLACK);
    break;
  case 3:
    pEInk->drawInvertedBitmap(x, y, bat3icons, width, height, GxEPD_BLACK);   //GxEPD_BLACK);
    break;
  case 4:
    pEInk->drawInvertedBitmap(x, y, bat4icons, width, height, GxEPD_BLACK);   //GxEPD_BLACK);
    break;
  default:
    pEInk->drawInvertedBitmap(x, y, bat0icons, width, height, GxEPD_BLACK);   //GxEPD_BLACK);
    break;
  }
}
void Screen::drawSatCount(int16_t x, int16_t y, int16_t width, int16_t height,uint8_t value){
  pEInk->setFont(&FreeSansBold9pt7b);
  if (value == 0){
    pEInk->drawXBitmap(x, y,gpsoff_bits,  16, 16, GxEPD_BLACK);
  }else{
    pEInk->drawXBitmap(x, y,gpsOn_bits,  16, 16, GxEPD_BLACK);
    pEInk->setCursor(x+17,y+height-2);
    pEInk->print(String(value));
  }
}

void Screen::getTextPositions(int16_t *posx, int16_t *posy,int16_t x, int16_t y, int16_t width, int16_t height,String sText){
  *posx = 0;
  *posy = 0;
  int16_t tbx, tby;
  uint16_t tbw, tbh;
  pEInk->getTextBounds(sText.c_str(), 0, 0, &tbx, &tby, &tbw, &tbh);
  *posx = x+(width-tbw-tbx) / 2;
  *posy = y+height-1;
  //log_i("%s x=%d,y=%d,w=%d,h=%d,tbx=%d,tby=%d,tbw=%d,tbh=%d,posx=%d,posy=%d",sText.c_str(),x,y,width,height,tbx, tby, tbw, tbh,*posx,*posy);
}

void Screen::drawWeatherScreen(void){
  uint32_t tAct = millis();
  //static uint32_t tOldUpate = millis();
  int16_t posx = 0;
  int16_t posy = 0;
  static weatherStatus oldWeather;
  static bool bForceUpdate = false;
  bool UpdateScreen = false;
  static bool bFullUpdate = false;
  static uint32_t tCharging = millis();
  static uint32_t tRun = millis();
  static uint8_t oldBattPercent;
  uint8_t battPercent;

  tAct = millis();
  battPercent = (status.battery.charging) ? 255 : status.battery.percent / 25;    
  switch (stepCount)
  {
  case 0:
    //bForceUpdate = true;
    bFullUpdate = true;
    stepCount++;
    break;
  case 1:
    if ((tAct - tRun) >= EINK_FULL_UPDATE){
      tRun = tAct;
      bFullUpdate = true;
    }
    if ((oldBattPercent != battPercent) || (bForceUpdate)){
      oldBattPercent = battPercent;
      //log_i("update Batt");
      UpdateScreen = true;
    }else if (oldBattPercent == 255){
      if (timeOver(tAct,tCharging,1000)){
        tCharging = tAct;
        //log_i("update Charging");
        UpdateScreen = true;
      }
    }
    if (checkValueDiff(oldWeather.WindSpeed,status.weather.WindSpeed,0) || (bForceUpdate)){
      oldWeather.WindSpeed = status.weather.WindSpeed;
      UpdateScreen = true;
    }
    if (checkValueDiff(oldWeather.WindGust,status.weather.WindGust,0) || (bForceUpdate)){
      oldWeather.WindGust = status.weather.WindGust;
      UpdateScreen = true;
    }
    if (checkValueDiff(oldWeather.WindDir,status.weather.WindDir,0) || (bForceUpdate)){
      oldWeather.WindDir = status.weather.WindDir;
      UpdateScreen = true;
    }
    if (checkValueDiff(oldWeather.temp,status.weather.temp,1) || (bForceUpdate)){
      oldWeather.temp = status.weather.temp;
      UpdateScreen = true;
    }
    if (checkValueDiff(oldWeather.Humidity,status.weather.Humidity,1) || (bForceUpdate)){
      oldWeather.Humidity = status.weather.Humidity;
      UpdateScreen = true;
    }
    if (checkValueDiff(oldWeather.Pressure,status.weather.Pressure,2) || (bForceUpdate)){
      oldWeather.Pressure = status.weather.Pressure;
      UpdateScreen = true;
    }
    if ((!bFullUpdate) && (!UpdateScreen)){
      break;
    }
    pEInk->setTextColor(GxEPD_BLACK);
    if ((setting.displayRotation == 0) || (setting.displayRotation == 2)){
      if (setting.displayRotation == 0){
        pEInk->setRotation(0);
      }else{
        pEInk->setRotation(2);
      }      
      pEInk->setTextSize(1);
      if (bFullUpdate){
        pEInk->setFullWindow();
        pEInk->firstPage();
        countFullRefresh = 0;
      }else{
        pEInk->setPartialWindow(0,0,pEInk->width(),pEInk->height());
        countFullRefresh++;
      }
      pEInk->setFont(&NotoSansBold6pt7b);
      getTextPositions(&posx,&posy,0,15,128,10,setting.PilotName);
      do
      {
        pEInk->fillScreen(GxEPD_WHITE);
        pEInk->setFont(&NotoSans6pt7b);
        pEInk->setCursor(40,10);
        pEInk->print(setting.myDevId);
        drawBatt(111,4,17,8,oldBattPercent);
        pEInk->setFont(&NotoSansBold6pt7b);
        pEInk->setCursor(posx,posy);
        pEInk->print(setting.PilotName);
        pEInk->drawFastHLine(0,26,pEInk->width(),GxEPD_BLACK);
        
        //Wind direction
        pEInk->setFont(&NotoSans6pt7b);
        pEInk->setCursor(5,37);
        pEInk->print("wind direction");
        pEInk->setFont(&gnuvarioe18pt7b);
        //drawCompass(0,42,128,31,oldWeather.WindDir);
        String sDir = getWDir(oldWeather.WindDir);
        getTextPositions(&posx,&posy,0,42,128,31,sDir);
        pEInk->setCursor(posx,posy);
        pEInk->print(sDir);
        pEInk->drawFastHLine(0,80,pEInk->width(),GxEPD_BLACK);


        //Wind-Speed /gust
        pEInk->setFont(&NotoSans6pt7b);
        pEInk->setCursor(5,91);
        pEInk->print("wind speed [km/h]");
        pEInk->setFont(&gnuvarioe23pt7b);
        drawValue(0,98,64,34,oldWeather.WindSpeed,0);
        drawValue(63,98,62,34,oldWeather.WindGust,0);
        pEInk->drawFastHLine(0,134,pEInk->width(),GxEPD_BLACK);
        pEInk->drawFastVLine(65,94,40,GxEPD_BLACK);

        //Temperature
        pEInk->setFont(&NotoSans6pt7b);
        pEInk->setCursor(5,145);
        pEInk->print("temperature [");
        pEInk->print(char(42));
        pEInk->print("C]");
        pEInk->setFont(&gnuvarioe23pt7b);
        drawValue(0,153,126,34,oldWeather.temp,1);
        pEInk->drawFastHLine(0,188,pEInk->width(),GxEPD_BLACK);

        //Pressure
        pEInk->setFont(&NotoSans6pt7b);
        pEInk->setCursor(5,199);
        pEInk->print("pressure [hPa]");
        pEInk->setFont(&gnuvarioe18pt7b);
        drawValue(0,205,126,34,oldWeather.Pressure,2);
        pEInk->drawFastHLine(0,242,pEInk->width(),GxEPD_BLACK);

        //Humidity
        pEInk->setFont(&NotoSans6pt7b);
        pEInk->setCursor(5,253);
        pEInk->print("humidity [");
        pEInk->print(char(37));
        pEInk->print("rH]");
        pEInk->setFont(&gnuvarioe18pt7b);
        drawValue(0,255,126,34,oldWeather.Humidity,1);
      }
      while (pEInk->nextPage());      
    }else{
      if (setting.displayRotation == 1){
        pEInk->setRotation(1);
      }else{
        pEInk->setRotation(3);
      }      
      pEInk->setTextSize(1);
      if (bFullUpdate){
        pEInk->setFullWindow();
        pEInk->firstPage();
        countFullRefresh = 0;
      }else{
        pEInk->setPartialWindow(0,0,pEInk->width(),pEInk->height());
        countFullRefresh++;
      }
      pEInk->setFont(&NotoSansBold6pt7b);
      getTextPositions(&posx,&posy,196,15,98,10,setting.PilotName);
      do
      {
        pEInk->fillScreen(GxEPD_WHITE);
        pEInk->setFont(&NotoSans6pt7b);

        pEInk->drawFastHLine(0,64,pEInk->width(),GxEPD_BLACK);
        pEInk->drawFastVLine(98,0,pEInk->height(),GxEPD_BLACK);
        pEInk->drawFastVLine(196,0,pEInk->height(),GxEPD_BLACK);

        pEInk->setCursor(230,10);
        pEInk->print(setting.myDevId);
        drawBatt(278,4,17,8,oldBattPercent);
        pEInk->setFont(&NotoSansBold6pt7b);
        pEInk->setCursor(posx,posy);
        pEInk->print(setting.PilotName);
        
        //Wind direction
        pEInk->setFont(&NotoSans6pt7b);
        pEInk->setCursor(5,10);
        pEInk->print("direction");
        pEInk->setFont(&gnuvarioe18pt7b);
        String sDir = getWDir(oldWeather.WindDir);
        getTextPositions(&posx,&posy,0,5,97,50,sDir);
        pEInk->setCursor(posx,posy);
        pEInk->print(sDir);

        //Wind-Speed /gust
        pEInk->setFont(&NotoSans6pt7b);
        pEInk->setCursor(103,10);
        pEInk->print("speed [km/h]");
        pEInk->setFont(&gnuvarioe18pt7b);
        drawValue(98,5,48,50,oldWeather.WindSpeed,0);
        pEInk->drawFastVLine(147,20,44,GxEPD_BLACK);
        drawValue(147,5,48,50,oldWeather.WindGust,0);

        //Temperature
        pEInk->setFont(&NotoSans6pt7b);
        pEInk->setCursor(5,74);
        pEInk->print("temp [");
        pEInk->print(char(42));
        pEInk->print("C]");
        pEInk->setFont(&gnuvarioe18pt7b);
        drawValue(0,69,92,50,oldWeather.temp,1);

        //Pressure
        pEInk->setFont(&NotoSans6pt7b);
        pEInk->setCursor(103,74);
        pEInk->print("pressure [hPa]");
        pEInk->setFont(&gnuvarioe14pt7b);
        drawValue(98,69,92,50,oldWeather.Pressure,2);

        //Humidity
        pEInk->setFont(&NotoSans6pt7b);
        pEInk->setCursor(201,74);
        pEInk->print("humidity [");
        pEInk->print(char(37));
        pEInk->print("rH]");
        pEInk->setFont(&gnuvarioe18pt7b);
        drawValue(196,69,92,50,oldWeather.Humidity,1);
      }
      while (pEInk->nextPage());      
    }  
    bFullUpdate = false;
    UpdateScreen = false;
    break;
  default:
    break;
  }
}

void Screen::drawMainScreen(void){
  uint32_t tAct = millis();
  //static uint32_t tOldUpate = millis();
  int16_t posx = 0;
  int16_t posy = 0;
  static screenMainData data;
  screenMainData actData;
  static bool bForceUpdate = false;
  bool UpdateScreen = false;
  static bool bFullUpdate = false;
  static uint32_t tCharging = millis();
  static uint32_t tRun = millis();

  tAct = millis();
  //copy values
  actData.alt = (status.gps.Fix) ? status.gps.alt : status.vario.alt;
  //log_i("%d,%.01f,%.01f",status.gps.Fix,status.gps.alt,status.vario.alt);
  actData.vario = status.vario.ClimbRate;
  actData.speed = status.gps.speed;
  //actData.compass = (status.gps.speed <= 5.0) ? status.vario.Heading : status.gps.course ;
  actData.compass = status.gps.course ;
  actData.battPercent = (status.battery.charging) ? 255 : status.battery.percent / 25;
  actData.SatCount = (status.gps.Fix) ? status.gps.NumSat : 0;
  if (actData.SatCount > 9) actData.SatCount = 9;
  //actData.SatCount = 9;
  actData.flightTime = status.flightTime;
  actData.flying = status.flying;
  actData.wifi = (status.wifiSTA.state) ? true : false;
  actData.bluetooth = status.bluetoothStat;
  if (status.bMuting){
      actData.volume = 0;
  }else{
      if (setting.vario.volume == LOWVOLUME){
          actData.volume = 1;
      }else if (setting.vario.volume == MIDVOLUME){
          actData.volume = 2;
      }else{
          actData.volume = 3;
      }
  }
  switch (stepCount)
  {
  case 0:
      //bForceUpdate = true;
      bFullUpdate = true;
      stepCount++;
      break;
  case 1:
      if ((tAct - tRun) >= EINK_FULL_UPDATE){
          tRun = tAct;
          bFullUpdate = true;
      }
      if ((data.SatCount != actData.SatCount) || (bForceUpdate)){
          data.SatCount = actData.SatCount;
          //log_i("update SatCount");
          UpdateScreen = true;
      }
      if ((data.battPercent != actData.battPercent) || (bForceUpdate)){
          data.battPercent = actData.battPercent;
          //log_i("update Batt");
          UpdateScreen = true;
      }else if (data.battPercent == 255){
          if (timeOver(tAct,tCharging,1000)){
              tCharging = tAct;
              //log_i("update Charging");
              UpdateScreen = true;
          }
      }
      if (checkValueDiff(data.alt,actData.alt,0) || (bForceUpdate)){
          data.alt = actData.alt;
          UpdateScreen = true;
      }
      //if ((abs(data.vario - actData.vario) >= 0.1) || (bForceUpdate)){
      if (checkValueDiff(data.vario,actData.vario,1) || (bForceUpdate)){
          data.vario = actData.vario;
          //log_i("update Vario");
          UpdateScreen = true;
      }
      if (checkValueDiff(data.speed,actData.speed,0) || (bForceUpdate)){
          data.speed = actData.speed;
          //log_i("update Speed");
          UpdateScreen = true;
      }
      if ((abs((int32_t)data.flightTime - (int32_t)actData.flightTime) >= 60) || (bForceUpdate)){
          data.flightTime = (actData.flightTime / 60) * 60; //only fixed minutes
          //log_i("update flightTime");
          UpdateScreen = true;
      }
      if (checkValueDiff(data.compass,actData.compass,0) || (bForceUpdate)){
          data.compass = actData.compass;
          //log_i("update Compass");
          UpdateScreen = true;
      }
      if ((data.volume != actData.volume) || (bForceUpdate)){
          data.volume = actData.volume;
          //log_i("update Volume");
          UpdateScreen = true;
      }
      if ((data.flying != actData.flying) || (bForceUpdate)){
          data.flying = actData.flying;
          //log_i("update status flying");
          UpdateScreen = true;
      }
      if ((data.wifi != actData.wifi) || (bForceUpdate)){
          data.wifi = actData.wifi;
          //log_i("update wifi status");
          UpdateScreen = true;
      }
      if ((data.bluetooth != actData.bluetooth) || (bForceUpdate)){
          data.bluetooth = actData.bluetooth;
          //log_i("update bluetooth status");
          UpdateScreen = true;
      }
      if ((!bFullUpdate) && (!UpdateScreen)){
          break;
      }
      pEInk->setTextColor(GxEPD_BLACK);
      if ((setting.displayRotation == 0) || (setting.displayRotation == 2)){
        if (setting.displayRotation == 0){
          pEInk->setRotation(0);
        }else{
          pEInk->setRotation(2);
        }          
        pEInk->setTextSize(1);
        if (bFullUpdate){
            pEInk->setFullWindow();
            pEInk->firstPage();
            countFullRefresh = 0;
        }else{
            pEInk->setPartialWindow(0,0,pEInk->width(),pEInk->height());
            countFullRefresh++;
        }
        pEInk->setFont(&NotoSansBold6pt7b);
        getTextPositions(&posx,&posy,0,35,128,10,setting.PilotName);
        do
        {
          pEInk->fillScreen(GxEPD_WHITE);
          drawspeaker(49,0,16,16,data.volume);
          drawflying(67,0,16,16,data.flying);
          if (data.wifi) pEInk->drawXBitmap(85, 4,WIFI_bits,  14, 8, GxEPD_BLACK);
          if (data.bluetooth == 1){
              pEInk->drawXBitmap(101, 3,BT_bits,  8, 10, GxEPD_BLACK);
          }else if (data.bluetooth == 2){
              pEInk->fillRect(101, 3, 8, 10, GxEPD_BLACK);
              pEInk->drawXBitmap(101, 3,BT_bits,  8, 10, GxEPD_WHITE);
          }
          drawBatt(111,4,17,8,data.battPercent);
          switch (setting.AircraftType)
          {
          case FanetLora::paraglider :
              pEInk->drawXBitmap(0, 0, Paraglider16_bits, 16, 16, GxEPD_BLACK);   //GxEPD_BLACK);
              break;
          case FanetLora::hangglider :
              pEInk->drawXBitmap(0, 0, Hangglider16_bits, 16, 16, GxEPD_BLACK);   //GxEPD_BLACK);
              break;
          case FanetLora::balloon :
              pEInk->drawXBitmap(0, 0, Ballon16_bits, 16, 16, GxEPD_BLACK);   //GxEPD_BLACK);
              break;
          case FanetLora::glider :
              pEInk->drawXBitmap(0, 0, Sailplane16_bits, 16, 16, GxEPD_BLACK);   //GxEPD_BLACK);
              break;
          case FanetLora::poweredAircraft :
              pEInk->drawXBitmap(0, 0, Airplane16_bits, 16, 16, GxEPD_BLACK);   //GxEPD_BLACK);
              break;
          case FanetLora::helicopter :
              pEInk->drawXBitmap(0, 0, Helicopter16_bits, 16, 16, GxEPD_BLACK);   //GxEPD_BLACK);
              break;
          case FanetLora::uav :
              pEInk->drawXBitmap(0, 0, UAV16_bits, 16, 16, GxEPD_BLACK);   //GxEPD_BLACK);
              break;

          default:
              pEInk->drawXBitmap(0, 0, UFO16_bits, 16, 16, GxEPD_BLACK);   //GxEPD_BLACK);
              break;
          }
          drawSatCount(18,0,26,16,data.SatCount);
          pEInk->drawFastHLine(0,52,pEInk->width(),GxEPD_BLACK);
          pEInk->setFont(&gnuvarioe23pt7b);
          drawValue(0,65,96,34,data.alt,0);
          pEInk->drawFastHLine(0,101,pEInk->width(),GxEPD_BLACK);
          pEInk->setFont(&gnuvarioe23pt7b);
          drawValue(0,114,96,34,data.vario,1);
          pEInk->drawFastHLine(0,150,pEInk->width(),GxEPD_BLACK);
          pEInk->setFont(&gnuvarioe23pt7b);
          drawValue(0,163,96,34,data.speed,0);
          pEInk->drawFastHLine(0,198,pEInk->width(),GxEPD_BLACK);
          drawFlightTime(0,213,128,34,data.flightTime);
          pEInk->drawFastHLine(0,248,pEInk->width(),GxEPD_BLACK);
          pEInk->setFont(&gnuvarioe18pt7b);
          drawCompass(0,261,128,31,data.compass);
          //pEInk->setFont(&FreeSansBold9pt7b);
          pEInk->setFont(&NotoSansBold6pt7b);
          pEInk->setCursor(posx,posy);
          pEInk->print(setting.PilotName);
          pEInk->setFont(&NotoSans6pt7b);
          pEInk->setCursor(40,30);
          pEInk->print(setting.myDevId);
          pEInk->setCursor(5,62);
          pEInk->print("Altitude");
          pEInk->setCursor(98, 97);
          pEInk->print('m');
          pEInk->setCursor(5,111);
          pEInk->print("Vario");
          pEInk->drawBitmap(98, 120, msicons, 24, 24, GxEPD_BLACK);   //GxEPD_BLACK);
          pEInk->setCursor(5,160);
          pEInk->print("Speed");
          pEInk->drawBitmap(98, 170, kmhicons, 24, 24, GxEPD_BLACK);   //GxEPD_BLACK);
          pEInk->setCursor(5,208);
          pEInk->print("Flight time");
          pEInk->setCursor(5,258);
          pEInk->print("Compass");
        }
        while (pEInk->nextPage());
      }else{
        if (setting.displayRotation == 1){
          pEInk->setRotation(1);
        }else{
          pEInk->setRotation(3);
        }          
        pEInk->setTextSize(1);
        if (bFullUpdate){
            pEInk->setFullWindow();
            pEInk->firstPage();
            countFullRefresh = 0;
        }else{
            pEInk->setPartialWindow(0,0,pEInk->width(),pEInk->height());
            countFullRefresh++;
        }
        pEInk->setFont(&NotoSansBold6pt7b);
        getTextPositions(&posx,&posy,196,54,95,7,setting.PilotName);
        do
        {
          pEInk->fillScreen(GxEPD_WHITE);
          pEInk->drawFastHLine(0,64,pEInk->width(),GxEPD_BLACK);
          pEInk->drawFastVLine(98,0,pEInk->height(),GxEPD_BLACK);
          pEInk->drawFastVLine(196,0,pEInk->height(),GxEPD_BLACK);

          drawBatt(278,4,17,8,data.battPercent);

          pEInk->setCursor(230,45);
          pEInk->setFont(&NotoSans6pt7b);
          pEInk->print(setting.myDevId);
          pEInk->setFont(&NotoSansBold6pt7b);
          pEInk->setCursor(posx,posy);
          pEInk->print(setting.PilotName);
          switch (setting.AircraftType)
          {
          case FanetLora::paraglider :
              pEInk->drawXBitmap(197, 0, Paraglider16_bits, 16, 16, GxEPD_BLACK);   //GxEPD_BLACK);
              break;
          case FanetLora::hangglider :
              pEInk->drawXBitmap(197, 0, Hangglider16_bits, 16, 16, GxEPD_BLACK);   //GxEPD_BLACK);
              break;
          case FanetLora::balloon :
              pEInk->drawXBitmap(197, 0, Ballon16_bits, 16, 16, GxEPD_BLACK);   //GxEPD_BLACK);
              break;
          case FanetLora::glider :
              pEInk->drawXBitmap(197, 0, Sailplane16_bits, 16, 16, GxEPD_BLACK);   //GxEPD_BLACK);
              break;
          case FanetLora::poweredAircraft :
              pEInk->drawXBitmap(197, 0, Airplane16_bits, 16, 16, GxEPD_BLACK);   //GxEPD_BLACK);
              break;
          case FanetLora::helicopter :
              pEInk->drawXBitmap(197, 0, Helicopter16_bits, 16, 16, GxEPD_BLACK);   //GxEPD_BLACK);
              break;
          case FanetLora::uav :
              pEInk->drawXBitmap(197, 0, UAV16_bits, 16, 16, GxEPD_BLACK);   //GxEPD_BLACK);
              break;

          default:
              pEInk->drawXBitmap(197, 0, UFO16_bits, 16, 16, GxEPD_BLACK);   //GxEPD_BLACK);
              break;
          }
          drawSatCount(220,0,26,16,data.SatCount);

          drawspeaker(197,17,16,16,data.volume);
          drawflying(220,17,16,16,data.flying);
          if (data.wifi) pEInk->drawXBitmap(255, 4,WIFI_bits,  14, 8, GxEPD_BLACK);
          if (data.bluetooth == 1){
              pEInk->drawXBitmap(255, 17,BT_bits,  8, 10, GxEPD_BLACK);
          }else if (data.bluetooth == 2){
              pEInk->fillRect(255, 17, 8, 10, GxEPD_BLACK);
              pEInk->drawXBitmap(255, 17,BT_bits,  8, 10, GxEPD_WHITE);
          }
          pEInk->setFont(&NotoSans6pt7b);
          pEInk->setCursor(5,10);
          pEInk->print("Vario [m/s]");
          pEInk->setFont(&gnuvarioe23pt7b);
          drawValue(0,10,97,50,data.vario,1);

          pEInk->setFont(&NotoSans6pt7b);
          pEInk->setCursor(103,10);
          pEInk->print("Altitude [m]");
          pEInk->setFont(&gnuvarioe23pt7b);
          drawValue(98,10,97,50,data.alt,0);

          pEInk->setFont(&NotoSans6pt7b);
          pEInk->setCursor(5,74);
          pEInk->print("speed [km/h]");
          pEInk->setFont(&gnuvarioe23pt7b);
          drawValue(0,74,90,50,data.speed,0);

          pEInk->setFont(&NotoSans6pt7b);
          pEInk->setCursor(103,74);
          pEInk->print("Flight time");
          drawFlightTime(98,85,98,30,data.flightTime);

          pEInk->setFont(&NotoSans6pt7b);
          pEInk->setCursor(201,74);
          pEInk->print("Compass");
          pEInk->setFont(&gnuvarioe18pt7b);
          String sDir = getWDir(data.compass);
          getTextPositions(&posx,&posy,196,78,98,40,sDir);
          pEInk->setCursor(posx,posy);
          pEInk->print(sDir);
        }
        while (pEInk->nextPage());
      }
      //log_i("update e-ink %dms",millis() - tAct);
      bFullUpdate = false;
      UpdateScreen = false;
      break;
  default:
      break;
  }
}

void Screen::drawflying(int16_t x, int16_t y, int16_t width, int16_t height,bool flying){
    if (flying){
        pEInk->drawXBitmap(x, y,flying_bits,  width, height, GxEPD_BLACK);
    }else{
        pEInk->drawXBitmap(x, y,not_flying_bits,  width, height, GxEPD_BLACK);
    }
}

void Screen::drawspeaker(int16_t x, int16_t y, int16_t width, int16_t height,uint8_t volume){
    switch (volume)
    {
    case 1:
        pEInk->drawXBitmap(x, y,speakerlow_bits,  16, 16, GxEPD_BLACK);
        break;
    case 2:
        pEInk->drawXBitmap(x, y,speakermid_bits,  16, 16, GxEPD_BLACK);
        break;
    case 3:
        pEInk->drawXBitmap(x, y,speakerhigh_bits,  16, 16, GxEPD_BLACK);
        break;
    default:
        pEInk->drawXBitmap(x, y,speakeroff_bits,  16, 16, GxEPD_BLACK);
        break;
    }
}

void Screen::drawFlightTime(int16_t x, int16_t y, int16_t width, int16_t height,uint32_t tTime){
    uint8_t min = (tTime / 60) % 60;
    uint8_t hours =  tTime / 3600;
    if (width < 120){
      pEInk->setFont(&gnuvarioe18pt7b);
      pEInk->drawBitmap(x + (width / 2) - 8, y + (height / 2) - 8, hicons, 16, 16, GxEPD_BLACK);   //GxEPD_BLACK);
      pEInk->setCursor(x + (width / 2) - 45,y + height -1);
      pEInk->printf("%02d",hours);
      pEInk->setCursor(x + (width / 2) + 8,y + height -1);
      pEInk->printf("%02d",min);
    }else{
      pEInk->setFont(&gnuvarioe23pt7b);
      pEInk->drawBitmap(x + (width / 2) - 8, y + (height / 2) - 8, hicons, 16, 16, GxEPD_BLACK);   //GxEPD_BLACK);
      pEInk->setCursor(x + (width / 2) - 54,y + height -1);
      pEInk->printf("%02d",hours);
      pEInk->setCursor(x + (width / 2) + 8,y + height -1);
      pEInk->printf("%02d",min);
    }
}

void Screen::webUpdate(void){
    if (setting.displayRotation <= 1){
      pEInk->setRotation(1);
    }else{
      pEInk->setRotation(3);
    }  
    pEInk->setFullWindow();
    pEInk->setFont(&FreeMonoBold24pt7b);
    pEInk->setTextColor(GxEPD_BLACK);
    pEInk->firstPage();
    do
    {
        pEInk->setCursor(10,40);
        pEInk->print("FW-UPDATE");
        pEInk->setCursor(10,90);
        pEInk->print("wait...");
    }
    while (pEInk->nextPage());

}