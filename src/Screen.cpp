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
    e_ink.setRotation(1);
    e_ink.setFont(&FreeMonoBold9pt7b);
    e_ink.setTextColor(GxEPD_BLACK);
    return true;
}

void Screen::end(void){
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
    if (!bInit){
        doInitScreen();
    }
}

void Screen::doInitScreen(void){
    static uint32_t tTimeStamp = millis();
    static uint8_t step = 0;
    uint32_t tAct = millis();
    switch (step)
    {
    case 0:
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
        if (timeOver(tAct,tTimeStamp,1000)){
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
        if (timeOver(tAct,tTimeStamp,1000)){
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
    default:
        step = 0;
        bInit = true;
        break;
    }
}