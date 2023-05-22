#include <RGBLed.h>
#include "RGBHelpers.h"

void flash()
{
RGBLed led(15, 2, 4, RGBLed::COMMON_CATHODE);
// Set color to red
led.setColor(130, 0, 200);
delay(100);

// Set color to green
led.setColor(100, 230, 0);
delay(100);

led.setColor(100, 68, 228);
delay(100);
}

void SetLedRGB(int r, int g, int b)
{
  RGBLed led(15, 2, 4, RGBLed::COMMON_CATHODE);
  led.setColor(r, g, b);
}