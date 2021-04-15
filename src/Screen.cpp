/*!
 * @file Screen.cpp
 *
 *
 */

#include "Screen.h"

GxEPD2_BW<GxEPD2_290, GxEPD2_290::HEIGHT> e_ink(GxEPD2_290(EINK_CS, EINK_DC, EINK_RST, EINK_BUSY));
//GxEPD2_BW<GxEPD2_290_T94, GxEPD2_290_T94::HEIGHT> e_ink(GxEPD2_290_T94(EINK_CS, EINK_DC, EINK_RST, EINK_BUSY));

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
    int16_t tbx, tby, posx, posy; uint16_t tbw, tbh;
    e_ink.getTextBounds(sText.c_str(), 0, height, &tbx, &tby, &tbw, &tbh);
    posx = x+width-tbw-tbx;
    posy = y+height-1;
    e_ink.setCursor(x,posy);
    e_ink.print(sDir.c_str());
    e_ink.setCursor(posx,posy);
    e_ink.print(sText.c_str());
}

void Screen::drawValue(int16_t x, int16_t y, int16_t width, int16_t height,float value,uint8_t decimals){
    String sText = String(value,decimals);
    //log_i("%.2f = %s",value,sText.c_str());
    int16_t tbx, tby, posx, posy; uint16_t tbw, tbh;
    e_ink.getTextBounds(sText.c_str(), 0, height, &tbx, &tby, &tbw, &tbh);
    posx = x+width-tbw-tbx;
    posy = y+height-1;
    //log_i("%d %d %d %d %d %d",tbx, tby, tbw, tbh,posx,posy);
    e_ink.setCursor(posx,posy);
    e_ink.print(sText.c_str());
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
        e_ink.drawInvertedBitmap(x, y, bat1icons, width, height, GxEPD_BLACK);   //GxEPD_BLACK);    
        break;
    case 2:
        e_ink.drawInvertedBitmap(x, y, bat2icons, width, height, GxEPD_BLACK);   //GxEPD_BLACK);    
        break;
    case 3:
        e_ink.drawInvertedBitmap(x, y, bat3icons, width, height, GxEPD_BLACK);   //GxEPD_BLACK);    
        break;
    case 4:
        e_ink.drawInvertedBitmap(x, y, bat4icons, width, height, GxEPD_BLACK);   //GxEPD_BLACK);    
        break;
    default:
        e_ink.drawInvertedBitmap(x, y, bat0icons, width, height, GxEPD_BLACK);   //GxEPD_BLACK);    
        break;
    }
}
void Screen::drawSatCount(int16_t x, int16_t y, int16_t width, int16_t height,uint8_t value){
    e_ink.setFont(&FreeSansBold9pt7b);
    if (value == 0){
        e_ink.drawXBitmap(x, y,gpsoff_bits,  16, 16, GxEPD_BLACK);
    }else{
        e_ink.drawXBitmap(x, y,gpsOn_bits,  16, 16, GxEPD_BLACK);
        e_ink.setCursor(x+17,y+height-2);
        e_ink.print(String(value));
    }
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

    //copy values
    actData.alt = (status.GPS_Fix) ? status.GPS_alt : status.varioAlt;
    actData.vario = status.ClimbRate;
    actData.speed = status.GPS_speed;
    //actData.compass = (status.GPS_speed <= 5.0) ? status.varioHeading : status.GPS_course ;
    actData.compass = status.GPS_course ;
    actData.battPercent = (status.BattCharging) ? 255 : status.BattPerc / 25;
    actData.SatCount = (status.GPS_Fix) ? status.GPS_NumSat : 0;
    if (actData.SatCount > 9) actData.SatCount = 9;
    //actData.SatCount = 9;
    actData.flightTime = status.flightTime;
    actData.flying = status.flying;
    actData.wifi = (status.wifiStat) ? true : false;
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
        if ((abs(data.SatCount - actData.SatCount) >= 1) || (bForceUpdate)){
            data.SatCount = actData.SatCount;
            //log_i("update SatCount");
            UpdateScreen = true;
        }
        if ((abs(data.battPercent - actData.battPercent) >= 1) || (bForceUpdate)){
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
        if ((abs(data.alt - actData.alt) >= 1.0) || (bForceUpdate)){
            data.alt = actData.alt;
            UpdateScreen = true;
        }
        if ((abs(data.vario - actData.vario) >= 0.1) || (bForceUpdate)){
            data.vario = actData.vario;
            //log_i("update Vario");
            UpdateScreen = true;
        }
        if ((abs(data.speed - actData.speed) >= 1.0) || (bForceUpdate)){
            data.speed = actData.speed;
            //log_i("update Speed");
            UpdateScreen = true;
        }
        if ((abs((int32_t)data.flightTime - (int32_t)actData.flightTime) >= 60) || (bForceUpdate)){
            data.flightTime = (actData.flightTime / 60) * 60; //only fixed minutes
            //log_i("update flightTime");
            UpdateScreen = true;
        }
        if ((abs(data.compass - actData.compass) >= 1.0) || (bForceUpdate)){
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
        //log_i("fullUpdate=%d UpdateScreen=%d time=%d",bFullUpdate,UpdateScreen,millis()-tOldUpate);
        //tOldUpate = millis();
        tAct = millis();
        e_ink.setTextColor(GxEPD_BLACK);
        e_ink.setRotation(0); 
        e_ink.setTextSize(1);       
        
        if (bFullUpdate){
            e_ink.setFullWindow();
            e_ink.firstPage();
        }else{
            e_ink.setPartialWindow(0,0,e_ink.width(),e_ink.height());
        }
        e_ink.setFont(&NotoSansBold6pt7b);
        getTextPositions(&posx,&posy,0,35,128,10,setting.PilotName);        
        do
        {
            e_ink.fillScreen(GxEPD_WHITE);
            drawspeaker(49,0,16,16,data.volume);
            drawflying(67,0,16,16,data.flying);
            if (data.wifi) e_ink.drawXBitmap(85, 4,WIFI_bits,  14, 8, GxEPD_BLACK);
            if (data.bluetooth == 1){
                e_ink.drawXBitmap(101, 3,BT_bits,  8, 10, GxEPD_BLACK);
            }else if (data.bluetooth == 2){
                e_ink.fillRect(101, 3, 8, 10, GxEPD_BLACK);
                e_ink.drawXBitmap(101, 3,BT_bits,  8, 10, GxEPD_WHITE);
            }  
            drawBatt(111,4,17,8,data.battPercent);
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
            drawSatCount(18,0,26,16,data.SatCount);
            e_ink.drawFastHLine(0,52,e_ink.width(),GxEPD_BLACK);
            e_ink.setFont(&gnuvarioe23pt7b);
            drawValue(0,65,96,34,data.alt,0);
            e_ink.drawFastHLine(0,101,e_ink.width(),GxEPD_BLACK);
            e_ink.setFont(&gnuvarioe23pt7b);
            drawValue(0,114,96,34,data.vario,1);
            e_ink.drawFastHLine(0,150,e_ink.width(),GxEPD_BLACK);
            e_ink.setFont(&gnuvarioe23pt7b);
            drawValue(0,163,96,34,data.speed,0);
            e_ink.drawFastHLine(0,198,e_ink.width(),GxEPD_BLACK);
            drawFlightTime(0,213,128,34,data.flightTime);
            e_ink.drawFastHLine(0,248,e_ink.width(),GxEPD_BLACK);
            e_ink.setFont(&gnuvarioe18pt7b);
            drawCompass(0,261,128,31,data.compass);
            //e_ink.setFont(&FreeSansBold9pt7b);            
            e_ink.setFont(&NotoSansBold6pt7b);
            e_ink.setCursor(posx,posy);
            e_ink.print(setting.PilotName);
            e_ink.setFont(&NotoSans6pt7b);
            e_ink.setCursor(40,30);
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
        e_ink.drawXBitmap(x, y,flying_bits,  width, height, GxEPD_BLACK);
    }else{
        e_ink.drawXBitmap(x, y,not_flying_bits,  width, height, GxEPD_BLACK);
    }
}

void Screen::drawspeaker(int16_t x, int16_t y, int16_t width, int16_t height,uint8_t volume){
    switch (volume)
    {
    case 1:
        e_ink.drawXBitmap(x, y,speakerlow_bits,  16, 16, GxEPD_BLACK);
        break;
    case 2:
        e_ink.drawXBitmap(x, y,speakermid_bits,  16, 16, GxEPD_BLACK);
        break;
    case 3:
        e_ink.drawXBitmap(x, y,speakerhigh_bits,  16, 16, GxEPD_BLACK);
        break;        
    default:
        e_ink.drawXBitmap(x, y,speakeroff_bits,  16, 16, GxEPD_BLACK);
        break;
    }
}

void Screen::drawFlightTime(int16_t x, int16_t y, int16_t width, int16_t height,uint32_t tTime){
    uint8_t min = (tTime / 60) % 60;
    uint8_t hours =  tTime / 3600;
    e_ink.setFont(&gnuvarioe23pt7b);
    e_ink.drawBitmap(x + (width / 2) - 8, y + (height / 2) - 8, hicons, 16, 16, GxEPD_BLACK);   //GxEPD_BLACK);    
    e_ink.setCursor(x + (width / 2) - 54,y + height -1);
    e_ink.printf("%02d",hours);
    e_ink.setCursor(x + (width / 2) + 8,y + height -1);
    e_ink.printf("%02d",min);
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