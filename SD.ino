#include <WiFi.h>
#include "time.h"
#include "sntp.h"

#include "FS.h"
#include "SD.h"
#include "SPI.h"

const char* ntpServer1 = "pool.ntp.org";
const char* ntpServer2 = "time.nist.gov";
const long  gmtOffset_sec = 3600;       //UTC+1
const int   daylightOffset_sec = 3600;  //with winter time change

struct tm timeinfo;       //current time





void SDSetup(){
    while(!SD.begin()){
        Serial.println("Card Mount Failed");
    }
    uint8_t cardType = SD.cardType();

    if(cardType == CARD_NONE){
        Serial.println("No SD card attached");
        return;
    }

        Serial.print("SD Card Type: ");
    if(cardType == CARD_MMC){
        Serial.println("MMC");
    } else if(cardType == CARD_SD){
        Serial.println("SDSC");
    } else if(cardType == CARD_SDHC){
        Serial.println("SDHC");
    } else {
        Serial.println("UNKNOWN");
    }

    uint64_t cardSize = SD.cardSize() / (1024 * 1024);
    Serial.printf("SD Card Size: %lluMB\n", cardSize);
    Serial.printf("Total space: %lluMB\n", SD.totalBytes() / (1024 * 1024));
    Serial.printf("Used space: %lluMB\n", SD.usedBytes() / (1024 * 1024));


    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer1, ntpServer2);  //configure time and servers
}

void CSVWrite(float csvarray[], int csvarraylenght){
  getLocalTime(&timeinfo);
  char filename[20];
  char filedata[400];
  sprintf(filename, "/%04d-%02d-%02d.csv", timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday);

  sprintf(filedata, "%d,%d,%d", timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
  for(int i = 0; i < csvarraylenght; i++){
      sprintf(filedata, "%s,%f", filedata, csvarray[i]);
   }
  sprintf(filedata, "%s\n", filedata);
  Serial.print("File name:");
  Serial.println(filename);
  Serial.print("CSV data");
  Serial.println(filedata);
  appendFile(SD, filename, filedata);
}

void appendFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Appending to file: %s\n", path);

    File file = fs.open(path, FILE_APPEND);
    if(!file){
        Serial.println("Failed to open file for appending");
        led.flash(RGBLed::MAGENTA,100);  //LED saving data failed
        return;
    }
    if(file.print(message)){
        Serial.println("Message appended");
    } else {
        Serial.println("Append failed");
        led.flash(RGBLed::MAGENTA,100);  //LED saving data failed
        delay(1000);
    }
    file.close();
}
