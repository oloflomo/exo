#include <WiFiManager.h>
#include "RGBHelpers.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include <driver/adc.h>
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "I2Cdev.h"
#include "MPU6050.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#define OUTPUT_READABLE_ACCELGYRO

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69

//MPU6050 accelgyro(0x69); // <-- use for AD0 high



// uncomment "OUTPUT_READABLE_ACCELGYRO" if you want to see a tab-separated
// list of the accel X/Y/Z and then gyro X/Y/Z values in decimal. Easy to read,
// not so easy to parse, and slow(er) over UART.


// uncomment "OUTPUT_BINARY_ACCELGYRO" to send all 6 axes of data as 16-bit
// binary, one right after the other. This is very fast (as fast as possible
// without compression or data loss), and easy to parse, but impossible to read
// for a human.
//#define OUTPUT_BINARY_ACCELGYRO


int ButtonPin = 32;
int latchPin = 27;      // Latch pin of 74HC595 is connected to Digital pin 5
int clockPin = 25;      // Clock pin of 74HC595 is connected to Digital pin 6
int dataPin = 26;       // Data pin of 74HC595 is connected to Digital pin 4
int BatteryPin = 34;
int BatteryLedPin1 = 12;
int BatteryLedPin2 = 13;
#define LED_PIN 13

WiFiManager wm;
MPU6050 accelgyro;

bool blinkState = false;
bool wm_nonblocking = false;
float Vmin = 1.645 * 2;
float Vmax = 2.083 * 2;
int16_t ax, ay, az;
int16_t gx, gy, gz;
int axtab[128], ax1[128], ax2[128], ax2fft[128];
int iteration = 0;

byte leds = 1;

void setup() 
{ 
  for(int i = 0; i < 128; ++i)
  {
    axtab[i] = 0;
    ax1[i] = 0;
    ax2[i] = 0;
    ax2fft[i] = 0;
  }
  // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

  // initialize serial communication
  // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
  // it's really up to you depending on your project)
  Serial.begin(115200);

  // initialize device
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  
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
  Serial.setDebugOutput(true);  
  delay(3000);
  Serial.println("\n Wifi Starting");

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
void loop() //base delay 1000
{
  delay(1000);
  // read raw accel/gyro measurements from device
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // these methods (and a few others) are also available
  //accelgyro.getAcceleration(&ax, &ay, &az);
  //accelgyro.getRotation(&gx, &gy, &gz);

  #ifdef OUTPUT_READABLE_ACCELGYRO
      // display tab-separated accel/gyro x/y/z values
      Serial.print("a/g:\t");
      Serial.print(ax); Serial.print("\t");
      Serial.print(ay); Serial.print("\t");
      Serial.print(az); Serial.print("\t");
      Serial.print(gx); Serial.print("\t");
      Serial.print(gy); Serial.print("\t");
      Serial.println(gz);
  #endif

    #ifdef OUTPUT_BINARY_ACCELGYRO
        Serial.write((uint8_t)(ax >> 8)); Serial.write((uint8_t)(ax & 0xFF));
        Serial.write((uint8_t)(ay >> 8)); Serial.write((uint8_t)(ay & 0xFF));
        Serial.write((uint8_t)(az >> 8)); Serial.write((uint8_t)(az & 0xFF));
        Serial.write((uint8_t)(gx >> 8)); Serial.write((uint8_t)(gx & 0xFF));
        Serial.write((uint8_t)(gy >> 8)); Serial.write((uint8_t)(gy & 0xFF));
        Serial.write((uint8_t)(gz >> 8)); Serial.write((uint8_t)(gz & 0xFF));
    #endif

  axtab[iteration] = ax;
  if (iteration > 0)
  {
    ax1[iteration] = ax1[iteration - 1] += axtab[iteration];
    ax2[iteration] = ax2[iteration - 1] += ax1[iteration];
  }
  iteration += 1;

  if (iteration == 128)
  {
    //fft(ax2);
    iteration = 0;
  }

  flash(); //delay 300

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