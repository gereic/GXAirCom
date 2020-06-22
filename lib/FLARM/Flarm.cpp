/*!
 * @file Flarm.cpp
 *
 *
 */

#include "Flarm.h"

#define KMPH_TO_MS 3.61

Flarm::Flarm(){
}


bool Flarm::begin(){
    return true;
}
String Flarm::getHexFromByte(uint8_t val){
    char myHexString[3];

    myHexString[0] = (val >> 4) + 0x30;
    if (myHexString[0] > 0x39) myHexString[0] +=7;


    myHexString[1] = (val & 0x0f) + 0x30;
    if (myHexString[1] > 0x39) myHexString[1] +=7;   
    myHexString[2] = 0;
    return String(myHexString);; 
}
String Flarm::getHexFromByte1(uint8_t val){
    char myHexString[2];

    myHexString[0] = (val & 0x0f) + 0x30;
    if (myHexString[0] > 0x39) myHexString[0] +=7;
    myHexString[1] = 0;
    return String(myHexString);

}


String Flarm::addChecksum(String s){
    int sLen = s.length();
    uint8_t chk = 0;
    char arChar[sLen+1];
    //Serial.print("******Laenge=");
    //Serial.println(sLen);
    s.toCharArray(arChar,sizeof(arChar));
    //Serial.println("******* DATA=");
    for (int i = 1; i < sLen; i++){
        chk ^= arChar[i];
        //Serial.print(arChar[i]);
    }
    //Serial.print("*****chksum=");
    //Serial.println(chk);
    return s + "*" + getHexFromByte(chk) + "\r\n";

}

String Flarm::writeFlarmData(FlarmtrackingData *myData,FlarmtrackingData *movePilotData){
    float pilotBearing = CalcBearingA( myData->lat, myData->lon,movePilotData->lat,movePilotData->lon);
    float pilotDistance = distance(myData->lat, myData->lon,movePilotData->lat,movePilotData->lon, 'K') ;
    float rads = deg2rad(pilotBearing);
    float relNorth=cos(rads) * pilotDistance * 1000;
    float relEast=sin(rads) * pilotDistance * 1000;
    float relVert = movePilotData->altitude - myData->altitude;
    float currentSpeed = movePilotData->speed/KMPH_TO_MS;
    String movingpilotData = "$PFLAA,0," + String((int32_t)round(relNorth)) + "," + String((int32_t)round(relEast)) + "," + String((int32_t)round(relVert)) + ",2," +
    		                    movePilotData->DevId + "," + (int32_t)round(movePilotData->heading) + ",0,"  +
								 String(currentSpeed,1) + "," + String(movePilotData->climb,1) + ","+ getHexFromByte1(uint8_t(movePilotData->aircraftType));
    //Serial.println(getHexFromByte((uint8_t)movePilotData->aircraftType));
    //Serial.println(movingpilotData);
    return addChecksum(movingpilotData);
}



String Flarm::writeDataPort(void){
    return addChecksum("$PFLAU,6,1,2,1,0,144,0,235,446");
}

void Flarm::run(void){    
    //uint32_t tAct = millis();
    //writeDataPort(tAct);
}

