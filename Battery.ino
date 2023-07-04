const float Vmin = 1.000;   //minimum input battery voltage
const float Vmax = 3.000;   //maximum input battery voltage

float BatteryPrecent = 0;


void InnitBattery(){  // 
  // Set all the pins of 74HC595 as OUTPUT
  pinMode(latchPin, OUTPUT);
  pinMode(dataPin, OUTPUT);  
  pinMode(clockPin, OUTPUT);
  pinMode(BatteryLedPin1, OUTPUT);
  pinMode(BatteryLedPin2, OUTPUT);
}

void ShowBattery()  //diplay battery level on led indicator
{
  ByteToRegister(BatteryToByte());
  digitalWrite(BatteryLedPin1, LOW);
  digitalWrite(BatteryLedPin2, LOW);
  float Voltage = CheckBattery();
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

float CheckBattery()  //get battery voltage and precentage
{
  float ADC_VALUE = analogRead(BatteryPin);
  float VoltageValue = (ADC_VALUE * 3.3 ) / (4095);
  float BatteryPrecent = (VoltageValue - Vmin)*100/(Vmax - Vmin);

  CSVdata[0] = BatteryPrecent;
  CSVdata[1] = VoltageValue;

  Serial.print("Voltage = ");
  Serial.print(VoltageValue);
  Serial.print("volts, ");
  Serial.print(BatteryPrecent);
  Serial.print("%");
  return VoltageValue;
}
