/*!
 * @file Screen.cpp
 *
 *
 */

#include "Screen.h"

GxEPD2_BW<GxEPD2_290, GxEPD2_290::HEIGHT> e_ink(GxEPD2_290(EINK_CS, EINK_DC, EINK_RST, EINK_BUSY));

Screen::Screen(){
}

bool Screen::begin(void){
    bInit = false;
    e_ink.epd2.init(EINK_CLK, EINK_DIN, 0, true, false); // define or replace SW_SCK, SW_MOSI
    e_ink.init(0); // needed to init upper level
    return true;
}

void Screen::end(void){
    e_ink.setRotation(1);
    e_ink.setFont(&FreeMonoBold9pt7b);
    e_ink.setTextColor(GxEPD_BLACK);
    e_ink.setFullWindow();
    e_ink.firstPage();
    do
    {
        e_ink.fillScreen(GxEPD_WHITE);
        e_ink.drawXBitmap(84,34,G_Logo_bits,G_Logo_width,G_Logo_height,GxEPD_BLACK);
        e_ink.drawXBitmap(114,34,X_Logo_bits,X_Logo_width,X_Logo_height,GxEPD_BLACK);
        e_ink.drawXBitmap(153,34,AirCom_Logo_bits,AirCom_Logo_width,AirCom_Logo_height,GxEPD_BLACK);
        e_ink.setCursor(169,92);
        e_ink.print(VERSION);
        e_ink.setCursor(80,92);
        e_ink.print(setting.myDevId);
    }
    while (e_ink.nextPage());
    bInit = false;

}

void Screen::run(void){
    //bInit = true;
    if (!bInit){
        doInitScreen();
    }else{
        drawMainScreen();
    }
}

void Screen::doInitScreen(void){
    static uint32_t tTimeStamp = millis();
    static uint8_t step = 0;
    uint32_t tAct = millis();
    switch (step)
    {
    case 0:
        e_ink.setRotation(1);
        e_ink.setFont(&FreeMonoBold9pt7b);
        e_ink.setTextColor(GxEPD_BLACK);
        e_ink.firstPage();
        do
        {
            e_ink.fillScreen(GxEPD_WHITE);
            e_ink.drawXBitmap(84,34,G_Logo_bits,G_Logo_width,G_Logo_height,GxEPD_BLACK);
        }
        while (e_ink.nextPage());
        tTimeStamp = millis();
        step ++;
        break;
    case 1:
        if (timeOver(tAct,tTimeStamp,500)){
            e_ink.setPartialWindow(0,0,e_ink.width(),e_ink.height());
            //e_ink.firstPage();
            do
            {
                e_ink.fillScreen(GxEPD_WHITE);
                e_ink.drawXBitmap(84,34,G_Logo_bits,G_Logo_width,G_Logo_height,GxEPD_BLACK);
                e_ink.drawXBitmap(114,34,X_Logo_bits,X_Logo_width,X_Logo_height,GxEPD_BLACK);
            }
            while (e_ink.nextPage());
            tTimeStamp = millis();
            step ++;
        }
        break;
    case 2:
        if (timeOver(tAct,tTimeStamp,500)){
            e_ink.setPartialWindow(0,0,e_ink.width(),e_ink.height());
            //e_ink.firstPage();
            do
            {
                e_ink.fillScreen(GxEPD_WHITE);
                e_ink.drawXBitmap(84,34,G_Logo_bits,G_Logo_width,G_Logo_height,GxEPD_BLACK);
                e_ink.drawXBitmap(114,34,X_Logo_bits,X_Logo_width,X_Logo_height,GxEPD_BLACK);
                e_ink.drawXBitmap(153,34,AirCom_Logo_bits,AirCom_Logo_width,AirCom_Logo_height,GxEPD_BLACK);
                e_ink.setCursor(169,92);
                e_ink.print(VERSION);
                e_ink.setCursor(80,92);
                e_ink.print(setting.myDevId);
            }
            while (e_ink.nextPage());
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
    //log_i("%.2f = %s",value,sText.c_str());
    int16_t tbx, tby, posx, posy; uint16_t tbw, tbh;
    e_ink.setPartialWindow(x,y,width,height);
    e_ink.getTextBounds(sText.c_str(), 0, height, &tbx, &tby, &tbw, &tbh);
    posx = x+width-tbw-tbx;
    posy = y+height-1;
    //log_i("%d %d %d %d %d %d",tbx, tby, tbw, tbh,posx,posy);
    do
    {
        e_ink.fillScreen(GxEPD_WHITE);
        e_ink.setCursor(x,posy);
        e_ink.print(sDir.c_str());
        e_ink.setCursor(posx,posy);
        e_ink.print(sText.c_str());
    }
    while (e_ink.nextPage());

}

void Screen::drawValue(int16_t x, int16_t y, int16_t width, int16_t height,float value,uint8_t decimals){
    String sText = String(value,decimals);
    //log_i("%.2f = %s",value,sText.c_str());
    int16_t tbx, tby, posx, posy; uint16_t tbw, tbh;
    e_ink.setPartialWindow(x,y,width,height);
    e_ink.getTextBounds(sText.c_str(), 0, height, &tbx, &tby, &tbw, &tbh);
    posx = x+width-tbw-tbx;
    posy = y+height-1;
    //log_i("%d %d %d %d %d %d",tbx, tby, tbw, tbh,posx,posy);
    do
    {
        e_ink.fillScreen(GxEPD_WHITE);
        e_ink.setCursor(posx,posy);
        e_ink.print(sText.c_str());
    }
    while (e_ink.nextPage());
}
void Screen::drawBatt(int16_t x, int16_t y, int16_t width, int16_t height,uint8_t value){
    static uint32_t tCharging = millis();
    static uint8_t DrawValue = 0;
    uint32_t tAct = millis();
    uint16_t posx,posy;
    
    posx = 104;
    posy = 4;
    //log_i("%d",value);
    if (value == 255){
        if (timeOver(tAct,tCharging,1000)){
            tCharging = tAct;
            DrawValue = (DrawValue + 1) %5; 
            //log_i("charging");
        }else{
            return;
        }
    }else{
        DrawValue = value;
    }
    //log_i("%d",DrawValue);
    e_ink.setPartialWindow(x,y,width,height);
    do
    {
        e_ink.fillScreen(GxEPD_WHITE);
        switch (DrawValue)
        {
        case 1:
            e_ink.drawInvertedBitmap(posx, posy, bat1icons, 17, 8, GxEPD_BLACK);   //GxEPD_BLACK);    
            break;
        case 2:
            e_ink.drawInvertedBitmap(posx, posy, bat2icons, 17, 8, GxEPD_BLACK);   //GxEPD_BLACK);    
            break;
        case 3:
            e_ink.drawInvertedBitmap(posx, posy, bat3icons, 17, 8, GxEPD_BLACK);   //GxEPD_BLACK);    
            break;
        case 4:
            e_ink.drawInvertedBitmap(posx, posy, bat4icons, 17, 8, GxEPD_BLACK);   //GxEPD_BLACK);    
            break;
        default:
            e_ink.drawInvertedBitmap(posx, posy, bat0icons, 17, 8, GxEPD_BLACK);   //GxEPD_BLACK);    
            break;
        }
        
    }
    while (e_ink.nextPage());

}
void Screen::drawSatCount(int16_t x, int16_t y, int16_t width, int16_t height,uint8_t value){
    e_ink.setFont(&FreeSansBold9pt7b);
    e_ink.setPartialWindow(x,y,width,height);
    do
    {
        e_ink.fillScreen(GxEPD_WHITE);
        if (value == 0){
            e_ink.drawXBitmap(x, y,gpsoff_bits,  16, 16, GxEPD_BLACK);
        }else{
            e_ink.drawXBitmap(x, y,gpsOn_bits,  16, 16, GxEPD_BLACK);
            e_ink.setCursor(x+17,y+height-2);
            e_ink.print(String(value));
        }
    }
    while (e_ink.nextPage());
}

void Screen::getTextPositions(int16_t *posx, int16_t *posy,int16_t x, int16_t y, int16_t width, int16_t height,String sText){
    *posx = 0;
    *posy = 0;
    int16_t tbx, tby; 
    uint16_t tbw, tbh;
    e_ink.getTextBounds(sText.c_str(),x, y, &tbx, &tby, &tbw, &tbh);
    *posx = x + (width-(tbx + tbw)) / 2;
    *posy = y + height-1;
    //log_i("%s x=%d,y=%d,w=%d,h=%d,tbx=%d,tby=%d,tbw=%d,tbh=%d,posx=%d,posy=%d",sText.c_str(),x,y,width,height,tbx, tby, tbw, tbh,*posx,*posy);
}

void Screen::drawMainScreen(void){
    int16_t posx = 0;
    int16_t posy = 0;
    static screenMainData data;
    screenMainData actData;
    static bool bForceUpdate = false;
    //copy values
    actData.alt = (status.GPS_Fix) ? status.GPS_alt : status.varioAlt;
    actData.vario = status.ClimbRate;
    actData.speed = status.GPS_speed;
    actData.compass = (status.GPS_speed <= 5.0) ? status.varioHeading : status.GPS_course ;
    actData.battPercent = (status.BattCharging) ? 255 : status.BattPerc / 25;
    actData.SatCount = (status.GPS_Fix) ? status.GPS_NumSat : 0;
    actData.flightTime = status.flightTime;
    switch (stepCount)
    {
    case 0:
        //if (status.GPS_fix)
        e_ink.setTextColor(GxEPD_BLACK);
        e_ink.setRotation(0); 
        e_ink.setTextSize(1);       
        e_ink.setFullWindow();
        e_ink.firstPage();
        e_ink.setFont(&NotoSansBold6pt7b);
        getTextPositions(&posx,&posy,0,35,128,10,setting.PilotName);        
        do
        {

           switch (setting.AircraftType)
           {
           case FanetLora::paraglider :
               e_ink.drawXBitmap(0, 0, Paraglider16_bits, 16, 16, GxEPD_BLACK);   //GxEPD_BLACK);
               break;
           case FanetLora::hangglider :
               e_ink.drawXBitmap(0, 0, Hangglider16_bits, 16, 16, GxEPD_BLACK);   //GxEPD_BLACK);
               break;
           case FanetLora::balloon :
               e_ink.drawXBitmap(0, 0, Ballon16_bits, 16, 16, GxEPD_BLACK);   //GxEPD_BLACK);
               break;
           case FanetLora::glider :
               e_ink.drawXBitmap(0, 0, Sailplane16_bits, 16, 16, GxEPD_BLACK);   //GxEPD_BLACK);
               break;
           case FanetLora::poweredAircraft :
               e_ink.drawXBitmap(0, 0, Airplane16_bits, 16, 16, GxEPD_BLACK);   //GxEPD_BLACK);
               break;
           case FanetLora::helicopter :
               e_ink.drawXBitmap(0, 0, Helicopter16_bits, 16, 16, GxEPD_BLACK);   //GxEPD_BLACK);
               break;
           case FanetLora::uav :
               e_ink.drawXBitmap(0, 0, UAV16_bits, 16, 16, GxEPD_BLACK);   //GxEPD_BLACK);
               break;
           
           default:
               e_ink.drawXBitmap(0, 0, UFO16_bits, 16, 16, GxEPD_BLACK);   //GxEPD_BLACK);
               break;
           }
            e_ink.drawFastHLine(0,52,e_ink.width(),GxEPD_BLACK);
            e_ink.drawFastHLine(0,101,e_ink.width(),GxEPD_BLACK);
            e_ink.drawFastHLine(0,150,e_ink.width(),GxEPD_BLACK);
            e_ink.drawFastHLine(0,198,e_ink.width(),GxEPD_BLACK);
            e_ink.drawFastHLine(0,248,e_ink.width(),GxEPD_BLACK);
            e_ink.setFont(&FreeSansBold9pt7b);            
            e_ink.setFont(&NotoSansBold6pt7b);
            e_ink.setCursor(posx,posy);
            e_ink.print(setting.PilotName);
            e_ink.setFont(&NotoSans6pt7b);
            e_ink.setCursor(50,11);
            e_ink.print(setting.myDevId);
            e_ink.setCursor(5,62);
            e_ink.print("Altitude");
            e_ink.setCursor(98, 97);
            e_ink.print('m');
            e_ink.setCursor(5,111);
            e_ink.print("Vario");
            e_ink.drawBitmap(98, 120, msicons, 24, 24, GxEPD_BLACK);   //GxEPD_BLACK);
            e_ink.setCursor(5,160);
            e_ink.print("Speed");
            e_ink.drawBitmap(98, 170, kmhicons, 24, 24, GxEPD_BLACK);   //GxEPD_BLACK);
            e_ink.setCursor(5,208);
            e_ink.print("Flight time");
            e_ink.setCursor(5,258);
            e_ink.print("Compass");

        }
        while (e_ink.nextPage());
        data.SatCount = 0;
        bForceUpdate = true;
        e_ink.setFont(&gnuvarioe23pt7b);
        e_ink.setTextColor(GxEPD_BLACK);
        stepCount++;
        break;
    case 1:        
        if ((abs(data.SatCount - actData.SatCount) >= 1) || (bForceUpdate)){
            data.SatCount = actData.SatCount;
            drawSatCount(18,0,32,16,data.SatCount);
        }
        if ((abs(data.battPercent - actData.battPercent) >= 1) || (bForceUpdate) || (actData.battPercent == 255)){
            data.battPercent = actData.battPercent;
            drawBatt(104,0,24,16,data.battPercent);
        }
        if ((abs(data.alt - actData.alt) >= 1.0) || (bForceUpdate)){
            data.alt = actData.alt;
            e_ink.setFont(&gnuvarioe23pt7b);
            drawValue(0,65,96,34,data.alt,0);
        }
        if ((abs(data.vario - actData.vario) >= 0.1) || (bForceUpdate)){
            data.vario = actData.vario;
            e_ink.setFont(&gnuvarioe23pt7b);
            drawValue(0,114,96,34,data.vario,1);
        }
        if ((abs(data.speed - actData.speed) >= 1.0) || (bForceUpdate)){
            data.speed = actData.speed;
            e_ink.setFont(&gnuvarioe23pt7b);
            drawValue(0,163,96,34,data.speed,0);
        }
        if ((abs(data.flightTime - actData.flightTime) >= 1000) || (bForceUpdate)){
            data.flightTime = actData.flightTime;
            drawFlightTime(0,213,128,34,actData.flightTime);
        }
        if ((abs(data.compass - actData.compass) >= 1.0) || (bForceUpdate)){
            data.compass = actData.compass;
            e_ink.setFont(&gnuvarioe18pt7b);
            drawCompass(0,261,128,31,data.compass);
        }
        bForceUpdate = false;

    default:
        break;
    }
}

void Screen::drawFlightTime(int16_t x, int16_t y, int16_t width, int16_t height,uint32_t tTime){
    e_ink.setFont(&gnuvarioe23pt7b);
    e_ink.setPartialWindow(x,y,width,height);
    uint8_t min = (tTime / 60) % 60;
    uint8_t hours =  tTime / 3600;
    do
    {
        e_ink.fillScreen(GxEPD_WHITE);
        e_ink.drawBitmap(x + (width / 2) - 8, y + (height / 2) - 8, hicons, 16, 16, GxEPD_BLACK);   //GxEPD_BLACK);    
        e_ink.setCursor(x + (width / 2) - 54,y + height -1);
        e_ink.printf("%02d",min);
        e_ink.setCursor(x + (width / 2) + 8,y + height -1);
        e_ink.printf("%02d",hours);

    }
    while (e_ink.nextPage());
}

void Screen::webUpdate(void){
    e_ink.setRotation(1);
    e_ink.setFullWindow();
    e_ink.setFont(&FreeMonoBold24pt7b);
    e_ink.setTextColor(GxEPD_BLACK);
    e_ink.firstPage();
    do
    {
        e_ink.setCursor(10,40);
        e_ink.print("FW-UPDATE");
        e_ink.setCursor(10,90);
        e_ink.print("wait...");
    }
    while (e_ink.nextPage());

}