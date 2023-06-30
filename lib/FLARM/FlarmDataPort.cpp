/*!
 * @file FlarmDataPort.cpp
 *
 *
 */

#include "FlarmDataPort.h"

#define KMPH_TO_MS 3.61

FlarmDataPort::FlarmDataPort(){
}


bool FlarmDataPort::begin(){
    return true;
}
String FlarmDataPort::getHexFromByte(uint8_t val){
    char myHexString[3];

    myHexString[0] = (val >> 4) + 0x30;
    if (myHexString[0] > 0x39) myHexString[0] +=7;


    myHexString[1] = (val & 0x0f) + 0x30;
    if (myHexString[1] > 0x39) myHexString[1] +=7;   
    myHexString[2] = 0;
    return String(myHexString);; 
}
String FlarmDataPort::getHexFromByte1(uint8_t val){
    char myHexString[2];

    myHexString[0] = (val & 0x0f) + 0x30;
    if (myHexString[0] > 0x39) myHexString[0] +=7;
    myHexString[1] = 0;
    return String(myHexString);

}


String FlarmDataPort::addChecksum(String s){
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

int FlarmDataPort::writeFlarmData(char *buffer, size_t size,FlarmtrackingData *myData,FlarmtrackingData *movePilotData){
    float pilotBearing = CalcBearingA( myData->lat, myData->lon,movePilotData->lat,movePilotData->lon);
    float pilotDistance = distance(myData->lat, myData->lon,movePilotData->lat,movePilotData->lon, 'K') ;
    float rads = deg2rad(pilotBearing);
    float relNorth=cos(rads) * pilotDistance * 1000;
    float relEast=sin(rads) * pilotDistance * 1000;
    float relVert = movePilotData->altitude - myData->altitude;
    float currentSpeed = movePilotData->speed/KMPH_TO_MS;
    /*
    Serial.printf("myLat=%.6f\n",myData->lat);
    Serial.printf("myLon=%.6f\n",myData->lon);
    Serial.printf("otherLat=%.6f\n",movePilotData->lat);
    Serial.printf("otherLon=%.6f\n",movePilotData->lon);
    Serial.printf("distance=%.6f\n",pilotDistance);
    Serial.printf("bearing=%.6f\n",pilotBearing);
    Serial.printf("relNorth=%.6f\n",relNorth);
    Serial.printf("relEast=%.6f\n",relEast);
    */
    /*
    Serial.print("myLat=");Serial.printf(myData->lat);
    Serial.print("myLon=");Serial.println(myData->lon);
    Serial.print("otherLat=");Serial.println(movePilotData->lat);
    Serial.print("otherLon=");Serial.println(movePilotData->lon);
    Serial.print("distance=");Serial.println(pilotDistance);
    Serial.print("bearing=");Serial.println(pilotBearing);
    Serial.print("relNorth=");Serial.println(relNorth);
    Serial.print("relEast=");Serial.println(relEast);
    */
    snprintf(buffer,size,"$PFLAA,0,%d,%d,%d,%d,%s,%d,0,%.01f,%.01f,%X",(int32_t)round(relNorth),(int32_t)round(relEast),(int32_t)round(relVert),movePilotData->addressType, movePilotData->DevId.c_str(),(int32_t)round(movePilotData->heading),currentSpeed,movePilotData->climb,uint8_t(movePilotData->aircraftType));
    return addChecksum(buffer,size);
    //String movingpilotData = "$PFLAA,0," + String((int32_t)round(relNorth)) + "," + String((int32_t)round(relEast)) + "," + String((int32_t)round(relVert)) + ",2," +
    //		                    movePilotData->DevId + "," + (int32_t)round(movePilotData->heading) + ",0,"  +
	//							 String(currentSpeed,1) + "," + String(movePilotData->climb,1) + ","+ getHexFromByte1(uint8_t(movePilotData->aircraftType));
    //Serial.println(getHexFromByte((uint8_t)movePilotData->aircraftType));
    //Serial.println(movingpilotData);
    //return addChecksum(movingpilotData);

}

/*
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
*/



int FlarmDataPort::writeDataPort(char *buffer, size_t size){
    if (neighbors > 99) neighbors = 99; //no more then 99 neighbors
    snprintf(buffer,size,"$PFLAU,%d,1,%d,1,0,0,0,0,0",neighbors,GPSState);
    return addChecksum(buffer,size);
    //Serial.printf("length=%d\r\n",strlen(buffer));
}

int FlarmDataPort::addChecksum(char *buffer, size_t size){
    uint8_t chk = 0;
    uint16_t tSize = 0;   
    while(*buffer){
        buffer++; //skip first char
        chk ^= *buffer;
        tSize++;
    }
    //Serial.printf("size=%d,tSize=%d,",size,tSize);
    tSize += snprintf(buffer,size-tSize,"*%02X\r\n",chk);
    //Serial.printf("tSize2=%d\r\n",tSize);
    return tSize;
}

int FlarmDataPort::writeVersion(char *buffer, size_t size){
    snprintf(buffer,size,"$PFLAV,A,1.00,1.00,GXAircom");
    return addChecksum(buffer,size);
}

int FlarmDataPort::writeSelfTestResult(char *buffer, size_t size){
    snprintf(buffer,size,"$PFLAE,A,0,0"); //no error 
    return addChecksum(buffer,size);
}

void FlarmDataPort::run(void){    
    //uint32_t tAct = millis();
    //writeDataPort(tAct);
}

