#include "oled.h"
#include "tools.h"
#include "icons.h"
#include "icons2.h"
#include <FanetLora.h>

Oled::Oled(){
}

bool Oled::begin(TwoWire *pi2c,int8_t rst,SemaphoreHandle_t *_xMutex){
  xMutex = _xMutex;
  pinRst = rst;
  oldScreenNumber = 0;
  if (display == NULL){
    #ifdef SH1106G
    display = new Adafruit_SH1106G(128, 64, pi2c);
    #else
    display = new Adafruit_SSD1306(128, 64, pi2c);
    #endif
  }
  xSemaphoreTake( *xMutex, portMAX_DELAY );
  PowerOn();
  display->setTextColor(WHITE);
  display->clearDisplay();
  display->display();
  display->drawXBitmap(0,2,G_Logo_bits,G_Logo_width,G_Logo_height,WHITE);
  display->display();
  delay(1000);
  display->drawXBitmap(30,2,X_Logo_bits,X_Logo_width,X_Logo_height,WHITE);
  display->display();
  delay(1000);
  display->drawXBitmap(69,2,AirCom_Logo_bits,AirCom_Logo_width,AirCom_Logo_height,WHITE);
  display->setTextSize(1);
  display->setCursor(85,55);
  display->print(VERSION);
  /*
  display->display();
  while (setting.myDevId.length() == 0){ //waiting for dev-ID
    delay(100);
  }
  */
  display->setTextColor(WHITE);
  display->setTextSize(1);
  display->setCursor(0,55);
  display->print(setting.myDevId.c_str());
  display->display();
  delay(1000);
  if (setting.gs.SreenOption == eScreenOption::ALWAYS_OFF){
    PowerOff();
  }else{
    display->clearDisplay();
    display->display();
  }  
  xSemaphoreGive( *xMutex );
  return true;
}

void Oled::PowerOn(void){
  if (bDisplayOn){
    return; //already on
  } 
  //reset OLED display via software
  if (pinRst >= 0){
    log_i("Heltec-board");
    pinMode(pinRst, OUTPUT);
    digitalWrite(pinRst, LOW);
    delay(100);
    digitalWrite(pinRst, HIGH);
    delay(100);
  }  
  //initialize OLED
  #ifdef SH1106G
  if(!display->begin(0x3C,false)) { // Address 0x3C for 128x64
  #else
  if(!display->begin(SSD1306_SWITCHCAPVCC, 0x3C, false, false)) { // Address 0x3C for 128x64
  #endif
    log_e("display allocation failed");
    for(;;); // Don't proceed, loop forever
  }
  //set display to full contrast
  #ifndef SH1106G
  display->ssd1306_command(SSD1306_SETCONTRAST);
  display->ssd1306_command(0xFF);
  #endif
  //added the possibility to invert BW .. whould be nice to put it in the settings TODO
  display->invertDisplay(0);
  bDisplayOn = true;
}

void Oled::PowerOff(void){
  if (!bDisplayOn){ 
    return; //already off
  }
  display->clearDisplay();
  display->display();
  log_i("set display to off");
  #ifndef SH1106G
  display->ssd1306_command(0x8D); //into charger pump set mode
  display->ssd1306_command(0x10); //turn off charger pump
  display->ssd1306_command(0xAE); //set OLED sleep  
  #endif
  bDisplayOn = false;
}

void Oled::drawWifiStat(eConnectionState wifiStat)
{
  if (wifiStat!=IDLE) 
  {
    
    WIFI_bits[2]=0xC4;
    WIFI_bits[6]=0xC9;
    display->drawXBitmap(85,0,WIFI_bits,WIFI_width,WIFI_height,WHITE);
  }
  if (wifiStat==FULL_CONNECTED) 
  { 
    WIFI_bits[2]=0x4;
    WIFI_bits[6]=0x9;
    display->drawXBitmap(85,0,WIFI_bits,WIFI_width,WIFI_height,WHITE);
  }
}

void Oled::webUpdate(void){
  xSemaphoreTake( *xMutex, portMAX_DELAY );
  display->clearDisplay();
  display->setTextSize(2);
  display->setCursor(10,5);
  display->print("FW-UPDATE");
  display->setCursor(10,30);
  display->print("wait...");
  display->display();
  xSemaphoreGive( *xMutex );
}

void Oled::drawAircraftType(int16_t x, int16_t y, uint8_t AircraftType){
  switch (AircraftType)
  {
  case FanetLora::paraglider :
  case FanetLora::leg_para_glider :
      display->drawXBitmap(x,y, Paraglider16_bits, 16, 16, WHITE);   //GxEPD_BLACK);
      break;
  case FanetLora::hangglider :
  case FanetLora::leg_hang_glider :
      display->drawXBitmap(x,y, Hangglider16_bits, 16, 16, WHITE);   //GxEPD_BLACK);
      break;
  case FanetLora::balloon :
  case FanetLora::leg_balloon :
      display->drawXBitmap(x,y, Ballon16_bits, 16, 16, WHITE);   //GxEPD_BLACK);
      break;
  case FanetLora::glider :
  case FanetLora::leg_glider_motor_glider :
      display->drawXBitmap(x,y, Sailplane16_bits, 16, 16, WHITE);   //GxEPD_BLACK);
      break;
  case FanetLora::poweredAircraft :
  case FanetLora::leg_aircraft_reciprocating_engine :
      display->drawXBitmap(x,y, Airplane16_bits, 16, 16, WHITE);   //GxEPD_BLACK);
      break;
  case FanetLora::helicopter :
  case FanetLora::leg_helicopter_rotorcraft :
      display->drawXBitmap(x,y, Helicopter16_bits, 16, 16, WHITE);   //GxEPD_BLACK);
      break;
  case FanetLora::uav :
  case FanetLora::leg_uav :
      display->drawXBitmap(x,y, UAV16_bits, 16, 16, WHITE);   //GxEPD_BLACK);
      break;
  
  default:
      display->drawXBitmap(x,y, UFO16_bits, 16, 16, WHITE);   //GxEPD_BLACK);
      break;
  }
}

void Oled::drawSatCount(int16_t x, int16_t y,uint8_t value){
    display->setTextSize(1);
    if (value == 0){
        display->drawXBitmap(x, y,gpsoff_bits,  16, 16, WHITE);
    }else{
        display->drawXBitmap(x, y,gpsOn_bits,  16, 16, WHITE);
        display->setCursor(x+18,y+4);
        display->print(String(value));
    }

}


void Oled::drawspeaker(int16_t x, int16_t y){
    uint8_t volume = 0;
    if (status.bMuting){
        volume = 0;
    }else{
        if (setting.vario.volume == LOWVOLUME){
            volume = 1;
        }else if (setting.vario.volume == MIDVOLUME){
            volume = 2;
        }else{
            volume = 3;
        }
    }
    switch (volume)
    {
    case 1:
        display->drawXBitmap(x, y,speakerlow_bits,  16, 16, WHITE);
        break;
    case 2:
        display->drawXBitmap(x, y,speakermid_bits,  16, 16, WHITE);
        break;
    case 3:
        display->drawXBitmap(x, y,speakerhigh_bits,  16, 16, WHITE);
        break;        
    default:
        display->drawXBitmap(x, y,speakeroff_bits,  16, 16, WHITE);
        break;
    }
}

void Oled::drawflying(int16_t x, int16_t y, bool flying){
    if (flying){
        display->drawXBitmap(x, y,flying_bits,  16, 16, WHITE);
    }else{
        display->drawXBitmap(x, y,not_flying_bits,  16, 16, WHITE);
    }
}

void Oled::drawBluetoothstat(int16_t x, int16_t y){
    if (status.bluetoothStat == 1){
     display->drawXBitmap(x,y,BT_bits,8,10,WHITE);
    }else if (status.bluetoothStat == 2){
      display->fillRect(x,y,8,10,WHITE);
      display->drawXBitmap(x,y,BT_bits,8,10,BLACK);
    }
}

void Oled::drawBatt(int16_t x, int16_t y,uint8_t value){
  static uint8_t DrawValue = 0;
  if (value == 255){
      DrawValue = (DrawValue + 1) %5; 
  }else{
      DrawValue = value / 25;
  }
  //log_i("%d",DrawValue);
  display->fillRect(x,y,17,8,WHITE);
  switch (DrawValue)
  {
  case 1:
      display->drawBitmap(x, y, bat1icons, 17, 8, BLACK);   //GxEPD_BLACK);    
      break;
  case 2:
      display->drawBitmap(x, y, bat2icons, 17, 8, BLACK);   //GxEPD_BLACK);    
      break;
  case 3:
      display->drawBitmap(x, y, bat3icons, 17, 8, BLACK);   //GxEPD_BLACK);    
      break;
  case 4:
      display->drawBitmap(x, y, bat4icons, 17, 8, BLACK);   //GxEPD_BLACK);    
      break;
  default:
      display->drawBitmap(x, y, bat0icons, 17, 8, BLACK);   //GxEPD_BLACK);    
      break;
  }
}

void Oled::printGPSData(uint32_t tAct){
  String s = "";
  display->clearDisplay();
  display->setTextSize(2);
  display->setCursor(0,0);
  drawAircraftType(0,0,setting.AircraftType);
  
  drawSatCount(18,0,(status.gps.NumSat > 9) ? 9 : status.gps.NumSat);
  drawspeaker(47,0);
  drawflying(67,0,status.flying);
  drawWifiStat(status.wifiSTA.state);
  drawBluetoothstat(101,0);
  drawBatt(111, 0,(status.battery.charging) ? 255 : status.battery.percent);

  display->setTextSize(3);

  display->setCursor(0,20);
  display->print(setStringSize(String(status.vario.ClimbRate,1) + "ms",7));

  display->setTextSize(2);

  display->setCursor(0,46);
  display->print(setStringSize(String(status.gps.alt,0) + "m",4));

  display->setCursor(65,46);
  display->print(setStringSize(String(status.gps.speed,0) + "kh",5));

  display->display();

}

String Oled::setStringSize(String s,uint8_t sLen){
  uint8_t actLen = (uint8_t)s.length();
  String sRet = "";
  for (uint8_t i = actLen;i < sLen;i++){
    sRet += " ";
  }
  sRet += s;
  return sRet;
}

void Oled::DrawAngleLine(int16_t x,int16_t y,int16_t length,float deg){
  int16_t xStart;
  int16_t yStart;
  int16_t xEnd;
  int16_t yEnd;
  float rads;
  rads = deg2rad(deg);
  xStart=(int)roundf(((sin(rads) * length/2) * 1) + x);
  yStart=(int)roundf(((cos(rads) * length/2) * -1) + y);
  xEnd=(int)roundf(((sin(rads) * length/2) * -1) + x);
  yEnd=(int)roundf(((cos(rads) * length/2) * 1) + y);  
  display->drawLine(xStart,yStart,xEnd,yEnd,WHITE);
  //log_i("x=%i,y=%i,deg=%0.1f,X-Start=%i,Y-Start=%i,X-End=%i,Y-End=%i",x,y,deg,xStart,yStart,xEnd,yEnd);
}
void Oled::DrawRadarPilot(uint8_t neighborIndex){
  float pilotDistance = 0.0;
  int bearing = 0;
  float rads;
  int relNorth;
  int relEast;
  display->setCursor(68,16);
  if (fanet.neighbours[neighborIndex].name.length() > 0){
    display->print(fanet.neighbours[neighborIndex].name.substring(0,10)); //max. 10 signs
  }else{
    display->print(fanet.getDevId(fanet.neighbours[neighborIndex].devId));
  }
  pilotDistance = distance(fanet._myData.lat, fanet._myData.lon,fanet.neighbours[neighborIndex].lat,fanet.neighbours[neighborIndex].lon, 'K') * 1000 ;
  bearing = CalcBearingA( fanet._myData.lat, fanet._myData.lon,fanet.neighbours[neighborIndex].lat,fanet.neighbours[neighborIndex].lon);
  rads = deg2rad(bearing + (fanet._myData.heading * -1));
  relEast=(int)((sin(rads) * 16) + RADAR_SCREEN_CENTER_X-8);
  relNorth=(int)(((cos(rads) * 16) * -1) + RADAR_SCREEN_CENTER_Y-8);
  //log_i("bearing=%i",bearing);
  //log_i("relNorth=%i",relNorth);
  //log_i("relEast=%i",relEast);
  switch (fanet.neighbours[neighborIndex].aircraftType)
  {
  case FanetLora::aircraft_t ::paraglider :
  case FanetLora::leg_para_glider :
    display->drawXBitmap(relEast, relNorth, Paraglider16_bits,16, 16,WHITE);      
    break;
  case FanetLora::aircraft_t::hangglider :
  case FanetLora::leg_hang_glider :
    display->drawXBitmap(relEast, relNorth, Hangglider16_bits,16, 16,WHITE);      
    break;
  case FanetLora::aircraft_t::balloon :
  case FanetLora::leg_balloon :
    display->drawXBitmap(relEast, relNorth, Ballon16_bits,16, 16,WHITE);      
    break;
  case FanetLora::aircraft_t::glider :
  case FanetLora::leg_glider_motor_glider :
    display->drawXBitmap(relEast, relNorth, Sailplane16_bits,16, 16,WHITE);      
    break;
  case FanetLora::aircraft_t::poweredAircraft :
  case FanetLora::leg_aircraft_reciprocating_engine :
    display->drawXBitmap(relEast, relNorth, Airplane16_bits,16, 16,WHITE);      
    break;
  case FanetLora::aircraft_t::helicopter :
  case FanetLora::leg_helicopter_rotorcraft :
    display->drawXBitmap(relEast, relNorth, Helicopter16_bits,16, 16,WHITE);      
    break;
  case FanetLora::aircraft_t::uav:
  case FanetLora::leg_uav :
    display->drawXBitmap(relEast, relNorth, UAV16_bits,16, 16,WHITE);      
    break;
  
  default:
    display->drawXBitmap(relEast, relNorth, UFO16_bits,16, 16,WHITE);      
    break;
  }
  if (pilotDistance > 1000){
    pilotDistance /= 1000;
    display->setCursor(68,28);
    display->printf("%skm",setStringSize(String(pilotDistance,1),5).c_str());
  }else{
    display->setCursor(75,28);
    display->printf("%sm",setStringSize(String(pilotDistance,0),5).c_str());
  }
  display->setCursor(75,40); //display relative alt
  display->printf("%sm",setStringSize(String(fanet.neighbours[neighborIndex].altitude - fanet._myData.altitude,0),5).c_str());
  display->setCursor(75,52); //display climbing
  display->printf("%sms",setStringSize(String(fanet.neighbours[neighborIndex].climb,1),5).c_str());
}



void Oled::DrawRadarScreen(uint32_t tAct,eRadarDispMode mode){
  static uint8_t neighborIndex = 0;
  int index;
  int16_t xStart;
  int16_t yStart;
  float rads;
  
  String s = "";
  display->clearDisplay();
  drawAircraftType(0,0,setting.AircraftType);
  drawSatCount(18,0,(status.gps.NumSat > 9) ? 9 : status.gps.NumSat);
  drawWifiStat(status.wifiSTA.state);
  drawBluetoothstat(101,0);
  drawBatt(111, 0,(status.battery.charging) ? 255 : status.battery.percent);

  
  display->setTextSize(1);
  display->drawCircle(RADAR_SCREEN_CENTER_X,RADAR_SCREEN_CENTER_Y,24,WHITE);

  DrawAngleLine(RADAR_SCREEN_CENTER_X,RADAR_SCREEN_CENTER_Y,30,fanet._myData.heading * -1);
  DrawAngleLine(RADAR_SCREEN_CENTER_X,RADAR_SCREEN_CENTER_Y,6,(fanet._myData.heading + 90) * -1);
  rads = deg2rad(fanet._myData.heading * -1);
  xStart=(int)(((sin(rads) * 19) * 1) + RADAR_SCREEN_CENTER_X);
  yStart=(int)(((cos(rads) * 19) * -1) + RADAR_SCREEN_CENTER_Y);
  display->setCursor(xStart-2,yStart-3);
  display->print("N");
  display->setTextSize(1);
  display->setCursor(42,0);
  switch (mode)
  {
  case eRadarDispMode::CLOSEST:
    display->print("CLOSEST");
    if (status.gps.Fix == 0){
      display->setCursor(60,16);
      display->print("NO GPS-FIX");
      break;
    } 
    index = fanet.getNearestNeighborIndex();
    //log_i("index %i",index);
    if (index < 0) break;
    neighborIndex = index;
    DrawRadarPilot(neighborIndex);
    break;
  case eRadarDispMode::LIST:
    display->print("LIST");
    if (status.gps.Fix == 0){
      display->setCursor(60,16);
      display->print("NO GPS-FIX");
      break;
    } 
    index = fanet.getNextNeighbor(neighborIndex);
    if (index < 0) break;
    neighborIndex = index;
    DrawRadarPilot(neighborIndex);
    break;
  case eRadarDispMode::FRIENDS:
    display->print("FRIENDS");
    if (status.gps.Fix == 0){
      display->setCursor(50,16);
      display->print("NO GPS-FIX");
      break;
    } 
    break;    
  default:
    break;
  }
  display->display();
}

void Oled::drawSignal(int16_t x, int16_t y,uint8_t strength) {
  if ((strength <= 9) && (strength >= 3)){
      display->drawBitmap(x,y,signal_1, SIGNALWIDTH, SIGNALHEIGHT, BLACK,WHITE);
  }else if ((strength >= 10) && (strength <= 14)){
      display->drawBitmap(x,y,signal_2, SIGNALWIDTH, SIGNALHEIGHT, BLACK,WHITE);
  }else if ((strength >= 15) && (strength <= 19)){
      display->drawBitmap(x,y,signal_3, SIGNALWIDTH, SIGNALHEIGHT, BLACK,WHITE);
  }else if ((strength >= 19) && (strength <= 30)){
      display->drawBitmap(x,y,signal_4, SIGNALWIDTH, SIGNALHEIGHT, BLACK,WHITE);
  }else{
      display->drawBitmap(x,y,signal_0, SIGNALWIDTH, SIGNALHEIGHT, BLACK,WHITE);
  }
}

void Oled::printWeather(uint32_t tAct){
  static uint32_t tRefresh = millis();
  static uint8_t screen = 0;
  if (timeOver(tAct,tRefresh,3000)){
    tRefresh = tAct;
    String s = "";
    display->clearDisplay();
    display->setTextSize(1);
    display->setCursor(0,0);
    display->print(setting.PilotName);
    drawWifiStat(status.wifiSTA.state);
    drawBluetoothstat(101,0);
    drawBatt(111, 0,(status.battery.charging) ? 255 : status.battery.percent);
    if (status.gsm.bHasGSM){
      drawSignal(60,0,status.gsm.SignalQuality);
    }    
    display->setTextSize(3); //set Textsize
    display->setCursor(0,21);
    switch(screen)
    {
      case 0: //battery-voltage
        s = String(status.battery.voltage / 1000.,2) + "V";
        break;
      case 1: //wind dir
        s = "  " + getWDir(status.weather.WindDir);
        break;
      case 2: //speed and gust
        s = String(round(status.weather.WindSpeed),0) + "|" + String(round(status.weather.WindGust),0); // + "kh";
        break;
      case 3: //temp
        s = " " + String(status.weather.temp,1) + "C";
        break;
      case 4: //pressure
        display->setCursor(0,26);
        display->setTextSize(2); //set Textsize
        s = " " + String(status.weather.Pressure,1) + "hPa";
        break;
      case 5: //humidity
        s = " " + String(status.weather.Humidity,0) + "%";
        break;
    }
    screen++;
    if (screen > 5) screen = 0;
    display->print(s);
    display->setTextSize(1);
    display->setCursor(0,55);
    display->print(setting.myDevId);
    display->setCursor(85,55);
    display->print(VERSION);
    display->display();
  }
}

void Oled::printScanning(uint32_t tAct){
  static uint8_t icon = 0;
  if (setting.gs.SreenOption == eScreenOption::ON_WHEN_TRAFFIC){
    PowerOff();
    return;
  }
  PowerOn();
  display->clearDisplay();
  drawWifiStat(status.wifiSTA.state);
  drawBluetoothstat(101,0);
  drawBatt(111, 0,(status.battery.charging) ? 255 : status.battery.percent);
  switch (icon)
  {
  case 0: 
    display->drawXBitmap(1,10,Antenna_bits,Antenna_width,Antenna_height,WHITE);
    break;
  case 1: 
    display->drawXBitmap(1,10,Antenna_bits,Antenna_width,Antenna_height,WHITE);
    display->drawXBitmap(42,10,WFTX_bits,WFTX_width,WFTX_height,WHITE);
    break;
  case 2: 
    display->drawXBitmap(1,10,Antenna_bits,Antenna_width,Antenna_height,WHITE);
    break;
  case 3: 
    display->drawXBitmap(1,10,Antenna_bits,Antenna_width,Antenna_height,WHITE);
    display->drawXBitmap(42,10,WFTX_bits,WFTX_width,WFTX_height,WHITE);
    display->drawXBitmap(66, 34, WFRX_bits,WFRX_width, WFRX_height,WHITE );
    display->drawXBitmap(88, 10, PGRX_bits,PGRX_width, PGRX_height,WHITE);      
    break;
  case 4: 
    display->drawXBitmap(1,10,Antenna_bits,Antenna_width,Antenna_height,WHITE);
    break;
  case 5: 
    display->drawXBitmap(1,10,Antenna_bits,Antenna_width,Antenna_height,WHITE);
    display->drawXBitmap(42,10,WFTX_bits,WFTX_width,WFTX_height,WHITE);
    display->drawXBitmap(66, 34, WFRX_bits,WFRX_width, WFRX_height,WHITE );
    display->drawXBitmap(88, 10, HGRX_bits,HGRX_width, HGRX_height,WHITE);      
    break;
  case 6: 
    display->drawXBitmap(1,10,Antenna_bits,Antenna_width,Antenna_height,WHITE);
    break;
  case 7: 
    display->drawXBitmap(1,10,Antenna_bits,Antenna_width,Antenna_height,WHITE);
    display->drawXBitmap(42,10,WFTX_bits,WFTX_width,WFTX_height,WHITE);
    display->drawXBitmap(66, 34, WFRX_bits,WFRX_width, WFRX_height,WHITE );
    display->drawXBitmap(88, 10, BLRX_bits,BLRX_width, BLRX_height,WHITE);      
    break;
  case 8: 
    display->drawXBitmap(1,10,Antenna_bits,Antenna_width,Antenna_height,WHITE);
    break;
  case 9: 
    display->drawXBitmap(1,10,Antenna_bits,Antenna_width,Antenna_height,WHITE);
    display->drawXBitmap(42,10,WFTX_bits,WFTX_width,WFTX_height,WHITE);
    display->drawXBitmap(66, 34, WFRX_bits,WFRX_width, WFRX_height,WHITE );
    display->drawXBitmap(88, 10, SPRX_bits,SPRX_width, SPRX_height,WHITE);      
    break;
  case 10: 
    display->drawXBitmap(1,10,Antenna_bits,Antenna_width,Antenna_height,WHITE);
    break;
  case 11: 
    display->drawXBitmap(1,10,Antenna_bits,Antenna_width,Antenna_height,WHITE);
    display->drawXBitmap(42,10,WFTX_bits,WFTX_width,WFTX_height,WHITE);
    display->drawXBitmap(66, 34, WFRX_bits,WFRX_width, WFRX_height,WHITE );
    display->drawXBitmap(88, 10, Airplane40_bits,Airplane40_width, Airplane40_height,WHITE);      
    break;
  case 12: 
    display->drawXBitmap(1,10,Antenna_bits,Antenna_width,Antenna_height,WHITE);
    break;
  case 13: 
    display->drawXBitmap(1,10,Antenna_bits,Antenna_width,Antenna_height,WHITE);
    display->drawXBitmap(42,10,WFTX_bits,WFTX_width,WFTX_height,WHITE);
    display->drawXBitmap(66, 34, WFRX_bits,WFRX_width, WFRX_height,WHITE );
    display->drawXBitmap(88, 10, Helicopter40_bits,Helicopter40_width, Helicopter40_height,WHITE);      
    break;
  case 14: 
    display->drawXBitmap(1,10,Antenna_bits,Antenna_width,Antenna_height,WHITE);
    break;
  case 15: 
    display->drawXBitmap(1,10,Antenna_bits,Antenna_width,Antenna_height,WHITE);
    display->drawXBitmap(42,10,WFTX_bits,WFTX_width,WFTX_height,WHITE);
    display->drawXBitmap(66, 34, WFRX_bits,WFRX_width, WFRX_height,WHITE );
    display->drawXBitmap(88, 10, UAVRX_bits,UAVRX_width, UAVRX_height,WHITE);      
    break;
  case 16: 
    display->drawXBitmap(1,10,Antenna_bits,Antenna_width,Antenna_height,WHITE);
    break;
  case 17: 
    display->drawXBitmap(1,10,Antenna_bits,Antenna_width,Antenna_height,WHITE);
    display->drawXBitmap(42,10,WFTX_bits,WFTX_width,WFTX_height,WHITE);
    display->drawXBitmap(66, 34, WFRX_bits,WFRX_width, WFRX_height,WHITE );
    display->drawXBitmap(88, 10, UFORX_bits,UFORX_width, UFORX_height,WHITE);      
    break;



  
  default:
    break;
  }
  icon++;
  if (icon > 17) icon = 2;

  display->setTextSize(1);
  display->setCursor(0,54);
  display->print("Scanning the skyes...");

  display->display();
}

void Oled::printGSData(uint32_t tAct){
  static uint8_t index = 0;
  static uint16_t fanetRx = 0;
  static uint16_t legacyRx = 0;
  static uint8_t received = 0;
  char buf[10];
  PowerOn();
  display->clearDisplay();
  drawWifiStat(status.wifiSTA.state);
  drawBluetoothstat(101,0);
  drawBatt(111, 0,(status.battery.charging) ? 255 : status.battery.percent);


  //show rx-count
  display->setTextSize(1);

  display->setCursor(78,0);
  if (fanetRx != status.fanetRx){
    fanetRx = status.fanetRx;
    received++;
  }
  if (legacyRx != status.legRx){
    legacyRx = status.legRx;
    received++;
  }

  sprintf(buf, "%d", received % 10);
  display->print(buf);


  //get next index
  for (int i = 0;i < MAXNEIGHBOURS;i++){
    if (fanet.neighbours[index].devId) break;
    index ++;
    if (index >= MAXNEIGHBOURS) index = 0;
  }
  if (!fanet.neighbours[index].devId){
    //no more neighbours --> return false
    display->display();
    return;
  } 



  display->setCursor(0,0);
  display->print(fanet.getDevId(fanet.neighbours[index].devId));
  display->setCursor(35,0);
  sprintf(buf, "%4ddb", fanet.neighbours[index].rssi);
  display->print(buf);

  display->setCursor(0,10);
  display->print(fanet.getNeighbourName(fanet.neighbours[index].devId));

  switch (fanet.neighbours[index].aircraftType)
  {
  case FanetLora::aircraft_t ::paraglider :
  case FanetLora::leg_para_glider :
    display->drawXBitmap(88, 12, PGRX_bits,PGRX_width, PGRX_height,WHITE);      
    break;
  case FanetLora::aircraft_t::hangglider :
  case FanetLora::leg_hang_glider :
    display->drawXBitmap(88, 12, HGRX_bits,HGRX_width, HGRX_height,WHITE);      
    break;
  case FanetLora::aircraft_t::balloon :
  case FanetLora::leg_balloon :
    display->drawXBitmap(88, 12, BLRX_bits,BLRX_width, BLRX_height,WHITE);      
    break;
  case FanetLora::aircraft_t::glider :
  case FanetLora::leg_glider_motor_glider :
    display->drawXBitmap(88, 12, SPRX_bits,SPRX_width, SPRX_height,WHITE);      
    break;
  case FanetLora::aircraft_t::poweredAircraft :
  case FanetLora::leg_aircraft_reciprocating_engine :
    display->drawXBitmap(88, 12, Airplane40_bits,Airplane40_width, Airplane40_height,WHITE);      
    break;
  case FanetLora::aircraft_t::helicopter :
  case FanetLora::leg_helicopter_rotorcraft :
    display->drawXBitmap(88, 10, Helicopter40_bits,Helicopter40_width, Helicopter40_height,WHITE);      
    break;
  case FanetLora::aircraft_t::uav:
  case FanetLora::leg_uav :
    display->drawXBitmap(88, 12, UAVRX_bits,UAVRX_width, UAVRX_height,WHITE);      
    break;
  
  default:
    display->drawXBitmap(88, 12, UFORX_bits,UFORX_width, UFORX_height,WHITE);      
    break;
  }
  
  display->setCursor(0,20);
  display->print("alt:");
  display->print(fanet.neighbours[index].altitude,0);
  display->print("m");

  display->setCursor(0,32);
  display->print("speed:");
  display->print(fanet.neighbours[index].speed,0);
  display->print("kmh");

  display->setCursor(0,44);
  display->print(fanet.neighbours[index].climb,1);
  display->print("m/s");

  display->setCursor(50,44);
  display->print(fanet.neighbours[index].heading,0);
  display->print("deg");

  display->setCursor(0,56);
  display->print(fanet.neighbours[index].lat,6);
  display->setCursor(64,56);
  display->print(fanet.neighbours[index].lon,6);


  display->display();
  index ++;
  if (index >= MAXNEIGHBOURS) index = 0;
}


void Oled::run(void){
  xSemaphoreTake( *xMutex, portMAX_DELAY );
  uint32_t tAct = millis();
  static uint32_t tDisplay = millis();  
  #ifdef GSMODULE
  if (setting.Mode == eMode::GROUND_STATION){
    if (setting.gs.SreenOption == eScreenOption::WEATHER_DATA){
      printWeather(tAct);
    }else if (setting.gs.SreenOption != eScreenOption::ALWAYS_OFF){
      if (fanet.getNeighboursCount() == 0){
        if (timeOver(tAct,tDisplay,500)){
          tDisplay = tAct;
          printScanning(tAct);
        }
      }else{
        if (timeOver(tAct,tDisplay,DISPLAY_UPDATE_RATE_GS)){
          tDisplay = tAct;
          printGSData(tAct);
        }
      }
    }
  }
  #endif
  #ifdef AIRMODULE
  if (setting.Mode == eMode::AIR_MODULE){
    switch (setting.screenNumber)
    {
    case 0: //main-Display
      if ((timeOver(tAct,tDisplay,DISPLAY_UPDATE_RATE)) || (oldScreenNumber != setting.screenNumber)){
        tDisplay = tAct;
        printGPSData(tAct);          
      }
      break;
    case 1: //radar-screen with list
      if ((timeOver(tAct,tDisplay,DISPLAY_UPDATE_RATE2)) || (oldScreenNumber != setting.screenNumber)){
        tDisplay = tAct;              
        DrawRadarScreen(tAct,eRadarDispMode::LIST);
      }
      break;
    case 2: //radar-screen with closest
      if ((timeOver(tAct,tDisplay,DISPLAY_UPDATE_RATE)) || (oldScreenNumber != setting.screenNumber)){
        tDisplay = tAct;
        DrawRadarScreen(tAct,eRadarDispMode::CLOSEST);
      }
      break;
    case 3: //list aircrafts
      if ((timeOver(tAct,tDisplay,DISPLAY_UPDATE_RATE)) || (oldScreenNumber != setting.screenNumber)){
        tDisplay = tAct;
      }
      break;
    default:
      break;
    }
    oldScreenNumber = setting.screenNumber;
  }
  #endif
  xSemaphoreGive( *xMutex );
}

void Oled::end(void){
  xSemaphoreTake( *xMutex, portMAX_DELAY );
  display->setTextColor(WHITE);
  display->clearDisplay();
  display->drawXBitmap(0,2,G_Logo_bits,G_Logo_width,G_Logo_height,WHITE);
  display->drawXBitmap(30,2,X_Logo_bits,X_Logo_width,X_Logo_height,WHITE);
  display->drawXBitmap(69,2,AirCom_Logo_bits,AirCom_Logo_width,AirCom_Logo_height,WHITE);
  display->setTextSize(1);
  display->setCursor(85,55);
  display->print(VERSION);
  display->setCursor(0,55);
  display->print(setting.myDevId);
  display->display();
  delay(1000);
  display->clearDisplay();
  display->drawXBitmap(0,2,G_Logo_bits,G_Logo_width,G_Logo_height,WHITE);
  display->drawXBitmap(30,2,X_Logo_bits,X_Logo_width,X_Logo_height,WHITE);
  display->display();
  delay(1000);
  display->clearDisplay();
  display->drawXBitmap(0,2,G_Logo_bits,G_Logo_width,G_Logo_height,WHITE);
  display->display();
  delay(1000);
  PowerOff();  
  xSemaphoreGive( *xMutex );
}