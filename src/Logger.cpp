/*!
 * @file Logger.cpp
 *
 *
 */

#include "Logger.h"

Logger::Logger(){

}

bool Logger::begin(){
  lInit = false;
  lStop = true;

  strcpy(igcPAth,"/default.igc");

  Serial.println("initialization done.");
  uint8_t cardType = SD_MMC.cardType();

    if(cardType == CARD_NONE){
        Serial.println("No SD_MMC card attached");
    }

    Serial.print("SD_MMC Card Type: ");
    if(cardType == CARD_MMC){
        Serial.println("MMC");
    } else if(cardType == CARD_SD){
        Serial.println("SDSC");
    } else if(cardType == CARD_SDHC){
        Serial.println("SDHC");
    } else {
        Serial.println("UNKNOWN");
    }

    uint64_t cardSize = SD_MMC.cardSize() / (1024 * 1024);
    Serial.printf("SD_MMC Card Size: %lluMB\n", cardSize);

    writeFile(SD_MMC, igcPAth, "file write test done.");
    Serial.println();
    Serial.println("wtest.txt written.");

  return true;
};

void Logger::end(void){
  // close logger
  lInit = false;
  lStop = true;
  ltest = true;
};

void Logger::run(void){
//    Serial.println(status.GPS_Date);
  if (status.flying){
    gotflytime = millis();
  }

  // if Flying i.e. also got gps fix and not initialized
  // test TODO CHANGE HERE to start recording when flying
  if (status.GPS_Date && !lInit){
//  if (status.flying && !lInit){
    lInit = true;
    lStop = false;
    // start new igc track with headers
    char trackFile[32];
    strcpy(trackFile,"/");
    strcat(trackFile,status.GPS_Date);
    strcat(trackFile,".igc");
    Serial.print("File to write: ");
    Serial.println(trackFile);
    strcpy(igcPAth,trackFile);
    doInitLogger(igcPAth);
    Serial.println(status.GPS_Date);
  }
    // if initilaized and flying/not but gps fix and sat > 4
  if(lInit && ( status.flying || (status.GPS_Fix && status.GPS_NumSat > 4)) ){
    updateLogger();
  }

  // if not flying (or i.e. gps fix lost) for more than 30s
  if ( ltest && (millis() - gotflytime > 30000)){
    lInit = false;
    ltest = false;
    // stop track close igc
    if (!lStop){
      lStop = true;
      doStopLogger();
    }
  }
}

void Logger::doInitLogger(const char * trackFile){
  const char* headers = "AXXX Tenz_Logger\r\
HFDTE020911\r\
HFFXA035\r\
HFPLTPILOTINCHARGE: John Doe\r\
HFGTYGLIDERTYPE:Paraglider\r\
HFGIDGLIDERID:0\r\
HFDTM100GPSDATUM: WGS-1984\r\
HFRFWFIRMWAREVERSION: 1.0\r\
HFRHWHARDWAREVERSION: 2021\r\
HFFTYFRTYPE: Arduino Nano Tenz Logger\r\
HFGPSGPS:Ublox Neo6m\r\
HFPRSPRESSALTSENSOR: BMP280\r\
HFCIDCOMPETITIONID:0\r\
HFCCLCOMPETITIONCLASS:PG3\r";

  Serial.println(trackFile);
  writeFile(SD_MMC, trackFile, headers);
};

void Logger::updateLogger(void){
  // TODO !!!
  const char* row = "igc row...\r";
  Serial.println(igcPAth);
  appendFile(SD_MMC, igcPAth, row);
}

void Logger::doStopLogger(void){
  // add igc security line
  // TODO
  // close igc file

}

void Logger::writeFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Writing file: %s\n", path);

    File file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("Failed to open file for writing");
        return;
    }
    if(file.print(message)){
        Serial.println("File written");
    } else {
        Serial.println("Write failed");
    }
}

void Logger::appendFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Appending to file: %s\n", path);

    File file = fs.open(path, FILE_APPEND);
    if(!file){
        Serial.println("Failed to open file for appending");
        return;
    }
    if(file.print(message)){
        Serial.println("Message appended");
    } else {
        Serial.println("Append failed");
    }
}

void Logger::listFiles(fs::FS &fs){
  // TODO List all files and push to server
}

// TODO! download igcfiles form server
//https://github.com/G6EJD/ESP32-8266-File-Download/blob/master/ESP_File_Download_v01.ino