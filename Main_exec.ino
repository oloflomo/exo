#include <WiFiManager.h>
#include "RGBHelpers.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include <driver/adc.h>
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
//max napiecie 2.083V
//min napiecie 1.645V

float Vmin = 1.645 * 2;
float Vmax = 2.083 * 2;
int ButtonPin = 32;
int latchPin = 27;      // Latch pin of 74HC595 is connected to Digital pin 5
int clockPin = 25;      // Clock pin of 74HC595 is connected to Digital pin 6
int dataPin = 26;       // Data pin of 74HC595 is connected to Digital pin 4
int BatteryPin = 34;
int BatteryLedPin1 = 12;
int BatteryLedPin2 = 13;


bool wm_nonblocking = false;
WiFiManager wm;


byte leds = 1;

void setup() 
{ 
  SetLedRGB(0, 0, 150);
  delay(1000);

  // Set all the pins of 74HC595 as OUTPUT
  pinMode(latchPin, OUTPUT);
  pinMode(dataPin, OUTPUT);  
  pinMode(clockPin, OUTPUT);
  pinMode(BatteryLedPin1, OUTPUT);
  pinMode(BatteryLedPin2, OUTPUT);
  pinMode(ButtonPin, INPUT);

  WiFi.mode(WIFI_STA);
  Serial.begin(115200);
  Serial.setDebugOutput(true);  
  delay(3000);
  Serial.println("\n Starting");

  pinMode(ButtonPin, INPUT);

  if(wm_nonblocking) wm.setConfigPortalBlocking(false);

  std::vector<const char *> menu = {"wifi","info","param","sep","restart","exit"};
  wm.setMenu(menu);

  wm.setClass("invert");

  wm.setConfigPortalTimeout(120);

  //bool res;

  //res = wm.autoConnect("AutoConnectAP","password");

  //if(!res) {
  //  Serial.println("Failed to connect or hit timeout");
  //} 
  //else { 
  //  Serial.println("connected...yeey :)");
  //}
  ShowBattery();

  SetLedRGB(150, 0, 0);
}

/*
 * loop() - this function runs over and over again
 */
void loop() 
{
  SetLedRGB(150, 0, 0);

  CheckButton();

  if(wm_nonblocking) wm.process();
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

void FlashRegister()
{
  leds = 0;        // Initially turns all the LEDs off, by giving the variable 'leds' the value 0
  digitalWrite(latchPin, LOW);
  shiftOut(dataPin, clockPin, LSBFIRST, leds);
  digitalWrite(latchPin, HIGH);
  delay(50);
  for (int i = 0; i < 8; i++)        // Turn all the LEDs ON one by one.
  {
    bitSet(leds, i);                // Set the bit that controls that LED in the variable 'leds'
    digitalWrite(latchPin, LOW);
    shiftOut(dataPin, clockPin, LSBFIRST, leds);
    digitalWrite(latchPin, HIGH);
    delay(50);
  }
}

void ByteToRegister(byte SomeByte)
{
  digitalWrite(latchPin, LOW);
  shiftOut(dataPin, clockPin, LSBFIRST, SomeByte);
  digitalWrite(latchPin, HIGH);
  delay(500);
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
  }
  return BatteryByte;
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

void CheckButton()
{
  // check for button press
  if ( digitalRead(ButtonPin) == LOW ) {
    delay(50);
    if( digitalRead(ButtonPin) == LOW )
    {
      SetLedRGB(50, 50, 50);
      Serial.println("Button Pressed");
      delay(3000); // reset delay hold
      if( digitalRead(ButtonPin) == LOW )
      {
        Serial.println("Button Held");
        // start portal w delay
        Serial.println("Starting config portal");
        wm.setConfigPortalTimeout(10); 
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
        }
      }
      SetLedRGB(0, 150, 0);
      
      FlashRegister();

      ShowBattery();
      
    }
  }
}