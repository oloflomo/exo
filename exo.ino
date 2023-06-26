////////////LIBRARIES/////////////

#include "I2Cdev.h"
//#include "arduinoFFT.h"
#include <WiFiManager.h>
#include "RGBHelpers.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include <driver/adc.h>
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
//#include "MPU6050_6Axis_MotionApps20.h"


////////////PIN DEFINITION/////////////
#define ButtonPin 35     // Button for baterry status or turning on wifi

#define latchPin 27       // Latch pin of 74HC595 is connected to Digital pin 5
#define clockPin 25       // Clock pin of 74HC595 is connected to Digital pin 6
#define dataPin 26        // Data pin of 74HC595 is connected to Digital pin 4
#define BatteryLedPin1 14 // Additional pin to show full led
#define BatteryLedPin2 12 // Additional pin 2

#define BatteryPin 34     // input pin for battry measurement

/////////////////GLOBAL VARIABLES////////////////
bool blinkState = false;

WiFiManager wm;

unsigned long ButtonTime = 0; //time since button pressed down

const uint16_t samples = 128;
const double samplingFrequency = 1300;
bool wm_nonblocking = false;
float Vmin = 1.645 * 2;
float Vmax = 2.083 * 2;


byte leds = 1;

#define SCL_INDEX 0x00
#define SCL_TIME 0x01         //???
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03


byte fun[10] = {0,1,2,0,0,0,0,0,0,0};
////////////////////SETUP//////////////////
void setup() 
{ 
  Serial.begin(115200);
  Serial.print("initialized serial");
  SetLedRGB(0, 0, 150);
  delay(1000);

  // Set all the pins of 74HC595 as OUTPUT
  pinMode(latchPin, OUTPUT);
  pinMode(dataPin, OUTPUT);  
  pinMode(clockPin, OUTPUT);
  pinMode(BatteryLedPin1, OUTPUT);
  pinMode(BatteryLedPin2, OUTPUT);
  pinMode(ButtonPin, INPUT_PULLUP);
  attachInterrupt(ButtonPin, ButtonInterrupt, CHANGE); //button interupt enable on change


  //wifimanager
  WiFi.mode(WIFI_STA);
  Serial.setDebugOutput(true);  
  delay(3000);
  Serial.println("\n Wifi Starting");

  if(wm_nonblocking) wm.setConfigPortalBlocking(false);

  std::vector<const char *> menu = {"wifi","info","param","sep","restart","exit"};
  wm.setMenu(menu);

  wm.setClass("invert");

  wm.setConfigPortalTimeout(120);
  
  ShowBattery();

  SetLedRGB(150, 0, 0);

  Serial.print("Setup finished");
}

void loop()
{ 
  delay(1000);
  for(int i = 0; i < sizeof(fun)/sizeof(fun[0]) ; i++){
  Serial.print(fun[i]);
  switch(fun[i]){
    case 1:       //set led color to normal
      SetLedRGB(150, 0, 0);
      break;
    case 2:       //wifi manager process
      SetLedRGB(0, 150, 0);
      if(wm_nonblocking) wm.process();
      break;
    case 3:       //indicate battery level
      SetLedRGB(0, 0, 150);
      ShowBattery();
      fun[3] = 0;
      break;
    case 4:       //initialize wifi manager
      SetLedRGB(150, 0, 150);
      WMinit();
      fun[4] = 0;
      break;

    
    default:
      break;
  }
}
Serial.println();
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

void WMinit(){  //initialize wifi manager
    wm.setConfigPortalTimeout(120);
      if (!wm.startConfigPortal("Exo","1234567890"))
      {
        Serial.println("failed to connect or hit timeout");
        delay(3000);
        // ESP.restart();
      } 
      else
      {
        //if you get here you have connected to the WiFi
        Serial.println("connected...yeey :)");
        SetLedRGB(0, 150, 0);
      }
    fun[2] = 2; //run service
}


void ShowBattery()
{
  ByteToRegister(BatteryToByte());
  digitalWrite(BatteryLedPin1, LOW);
  digitalWrite(BatteryLedPin2, LOW);
  float Voltage = CheckBattery() * 2;
  float ref = Vmin + ((Vmax - Vmin) / float(10)) * float(8);
  if(Voltage > ref)
    digitalWrite(BatteryLedPin1, HIGH);
  ref = Vmin + ((Vmax - Vmin) / float(10)) * float(9);
  if(Voltage > ref)
    digitalWrite(BatteryLedPin2, HIGH);
  delay(3000);
  digitalWrite(BatteryLedPin1, LOW);
  digitalWrite(BatteryLedPin2, LOW);
  ByteToRegister(0);
}

void ByteToRegister(byte SomeByte)
{
  digitalWrite(latchPin, LOW);
  shiftOut(dataPin, clockPin, LSBFIRST, SomeByte);
  digitalWrite(latchPin, HIGH);
}

byte BatteryToByte()
{
  byte BatteryByte;
  float Voltage = CheckBattery() * 2;
  BatteryByte = 0;
  for (int i = 0; i < 8; ++i)
  {
    float ref = Vmin + ((Vmax - Vmin) / float(10)) * float(i);
    if(Voltage > ref)
      bitSet(BatteryByte, i);
      
      digitalWrite(latchPin, LOW);
      shiftOut(dataPin, clockPin, LSBFIRST, BatteryByte);
      digitalWrite(latchPin, HIGH);
      delay(50);
  }
  return BatteryByte;
  Serial.print(BatteryByte);
}

float CheckBattery()
{
  float ADC_VALUE = analogRead(BatteryPin);
  float VoltageValue = (ADC_VALUE * 3.3 ) / (4095);

  Serial.print("Voltage = ");
  Serial.print(VoltageValue);
  Serial.print("volts\n");
  return VoltageValue;
}
