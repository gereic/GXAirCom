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

  strcpy(igcPAth,"/test.igc");

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

    writeFile(SD_MMC, igcPAth, igcHeaders());
    Serial.println();
    listFiles(SD_MMC,"/");

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
    int8_t fnum = 0;
    char fnumc[3];
    while (1){
      char trackFile[32];
      strcpy(trackFile,"/");
      strcat(trackFile,status.GPS_Date);
      strcat(trackFile,"_");
      strcat(trackFile,itoa(fnum,fnumc,10));
      // TODO this will be withouth extension and will be added when closing the igc 
      //      to have .igc files downloadable only when closed properly with security line
      strcat(trackFile,".igc"); 
      if (SD_MMC.exists(trackFile)){
        Serial.print("File already exists: ");
        Serial.println(trackFile);
        fnum+=1;
      }else{
        Serial.print("File to write: ");
        Serial.println(trackFile);
        strcpy(igcPAth,trackFile);
        doInitLogger(igcPAth);
        break;
      }
    }
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

char * Logger::igcHeaders(){
  char headers[500] = ""; 
  strcpy(headers,IGC_ROW1);
  strcat(headers,"\r");
  strcat(headers,IGC_ROW2);
  strcat(headers,"\r");
  strcat(headers,IGC_ROW3);
  if (status.GPS_Date) strcat(headers,status.GPS_Date);
  strcat(headers,"\r");
  strcat(headers,IGC_ROW4);
  strcat(headers,"\r");
  strcat(headers,IGC_ROW5);
  strcat(headers,"\r");
  strcat(headers,IGC_ROW6);
  strcat(headers,"\r");
  strcat(headers,IGC_ROW7);
  strcat(headers,"\r");
  strcat(headers,IGC_ROW8);
  strcat(headers,"\r");
  strcat(headers,IGC_ROW9);
  strcat(headers,"\r");
  strcat(headers,IGC_ROW10);
  strcat(headers,"\r");
  strcat(headers,IGC_ROW11);
  strcat(headers,"\r");
  strcat(headers,IGC_ROW12);
  strcat(headers,"\r");
  strcat(headers,IGC_ROW13);
  strcat(headers,"\r");
  strcat(headers,IGC_ROW14);
  strcat(headers,"\r");
  return headers;
}

void Logger::doInitLogger(const char * trackFile){
  
  //const char* headers = "...headers...\r";
  Serial.println(trackFile);
  writeFile(SD_MMC, trackFile, igcHeaders());
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
  // close igc file and rename to .igc to be downloadable

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

void Logger::listFiles(fs::FS &fs, const char * dirname){
  // TODO List all files and push to server
  Serial.println("Listing igc files:");
  File root = fs.open(dirname);
      if(!root){
        Serial.println("Failed to open directory");
        return;
    }
    if(!root.isDirectory()){
        Serial.println("Not a directory");
        return;
    }

    File file = root.openNextFile();
    
    strcpy(igclist,"");
    while(file){
        if(!file.isDirectory()){    
          if (  strstr(file.name(), ".igc") ){       
            Serial.print("  FILE: ");
            Serial.print(file.name());
            Serial.print("  SIZE: ");
            Serial.println(file.size());
            strcat(igclist,file.name());
            strcat(igclist,";");
          }
        }
        file = root.openNextFile();
    }
    Serial.println();

    return;
}

// TODO! download igcfiles form server
//https://github.com/G6EJD/ESP32-8266-File-Download/blob/master/ESP_File_Download_v01.ino