////////////LIBRARIES/////////////

#include "I2Cdev.h"
//#include "arduinoFFT.h"
#include <WiFiManager.h>
#include "esp_wifi.h"
#include "esp_system.h"
//#include "MPU6050_6Axis_MotionApps20.h"
//DS18b20
#include <OneWire.h>
#include <DallasTemperature.h>
#include <RGBLed.h>






#include <WiFi.h>
#include <WiFiClient.h> 
#include <ESP32_FTPClient.h>

#include "time.h"
#include "sntp.h"

#include "FS.h"
#include "SD.h"
#include "SPI.h"





// time server
const char* ntpServer1 = "pool.ntp.org";
const char* ntpServer2 = "time.nist.gov";
const long  gmtOffset_sec = 3600;       //UTC+1
const int   daylightOffset_sec = 3600;  //with winter time change


//ftp server settings
char ftp_server[] = "156.67.99.8";
char ftp_user[]   = "exo";
char ftp_pass[]   = "jejutonajgorszawyspa!!!";

String rootpath = "/";  //sd card root
// you can pass a FTP timeout and debbug mode on the last 2 arguments 0-no debug 2-debug
ESP32_FTPClient ftp (ftp_server,ftp_user,ftp_pass, 5000, 0);




#define LedRed 15   //RGB led pins
#define LedGreen 2
#define LedBlue 4

////////////PIN DEFINITION/////////////
#define ButtonPin 35     // Button for baterry status or turning on wifi
#define ONE_WIRE_BUS 32   // dallas pin
/////////////////GLOBAL VARIABLES////////////////
bool blinkState = false;

WiFiManager wm;

OneWire oneWire(ONE_WIRE_BUS);      //inicjalizacja onewire
DallasTemperature Dallas(&oneWire); //inicjalizacja dallasa

RGBLed led(LedRed, LedGreen, LedBlue, RGBLed::COMMON_CATHODE);    //led setup

unsigned long ButtonTime = 0; //time since button pressed down

const uint16_t samples = 128;
const double samplingFrequency = 1300;

float DALTemp = 0; //- temperatura ciała

byte leds = 1;

hw_timer_t *SD_timer = NULL;    //timer A each 10s interrupt to save CSV data

float CSVdata[] = {0, 0, 0, 0};  //main array that get saved on csv file


byte fun[10] = {0,1,0,0,4,0,6,0,8,0};
////////////////////SETUP//////////////////
void setup() 
{ 
  Serial.begin(115200);
  Serial.print("initialized serial");
  led.setColor(RGBLed::WHITE);  //LED start of initialization
  delay(1000);
  
  //innit dallas


  led.setColor(RGBLed::YELLOW); //LED initialize sd card
  SDSetup();
  led.flash(RGBLed::BLUE,100);  //LED initialize wifimanager
  WMSetup();
  delay(1000);
  led.setColor(RGBLed::MAGENTA);  //LED set up sensors

  InnitBattery();
  //innit dallas
  Dallas.begin();
  Dallas.setResolution(12);
  Dallas.setWaitForConversion(false);
  MAXsetup();

  pinMode(ButtonPin, INPUT_PULLUP);
  attachInterrupt(ButtonPin, ButtonInterrupt, CHANGE); //button interupt enable on change
  



  led.setColor(RGBLed::RED);  //LED show battery status
  ShowBattery();
  

   
  SD_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(SD_timer, &SDTimerInter, true);
  timerAlarmWrite(SD_timer, 10000000, true);
  timerAlarmEnable(SD_timer);

  Serial.print("Setup finished");
  led.setColor(RGBLed::GREEN);  //LED confirm correst setup
  delay(1000);
}

void loop()
{ 
  for(int i = 0; i < sizeof(fun)/sizeof(fun[0]) ; i++){
  Serial.print(fun[i]);
  Serial.print(":");
  switch(fun[i]){
    case 1:       //set led color to normal
      //flashLED();
      led.setColor(RGBLed::GREEN);
      break;
    case 2:       //wifi manager process
      wm.process();
      break;
    case 3:       //indicate battery level
      led.setColor(RGBLed::RED);  //LED show battery status
      ShowBattery();
      fun[3] = 0;
      break;
    case 4:       //initialize wifi manager
      led.flash(RGBLed::BLUE,100);  //LED initialize wifimanager
      WMStart();
      fun[4] = 0;
      break;
    case 5:       //read pulse sensor
      MAXREAD();
      break;
    case 6:       //read temperature
      Dallas.requestTemperatures();
      DALTemp = Dallas.getTempCByIndex(0);
      CSVdata[2] = DALTemp;
      break;
    case 7:       //save data to scv file
      led.setColor(RGBLed::YELLOW);;  //LED saving data to CSV
      delay(100);
      CSVWrite(CSVdata, sizeof(CSVdata)/sizeof(float));
      fun[7] = 0;
      break;
    case 8:       //chceck battery precentage and voltage
      CheckBattery();
      break;
    default:
      break;
  }
}
Serial.println();
}


void SDTimerInter(){    //interrupt 10s timer to run save on csv
  fun[7] = 7;
}

void ButtonInterrupt(){
  if(digitalRead(ButtonPin) == LOW){
    Serial.println("Button down");
    ButtonTime = millis();
  }
  else{
    Serial.println("Button up");
    Serial.println(millis() - ButtonTime);
    if((millis() - ButtonTime)>50){
      Serial.println("Button pressed for 50ms");
      fun[3] = 3;
    }
    if((millis() - ButtonTime)>3000){
      Serial.println("Button Held for 3 seconds");
      fun[4] = 4;
    } 
  }
}
