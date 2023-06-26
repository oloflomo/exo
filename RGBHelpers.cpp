#include <RGBLed.h>
#include "RGBHelpers.h"

#define LedRed 15
#define LedGreen 2
#define LedBlue 4

void flash()
{
RGBLed led(LedRed, LedGreen, LedBlue, RGBLed::COMMON_CATHODE);
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
  RGBLed led(LedRed, LedGreen, LedBlue, RGBLed::COMMON_CATHODE);
  led.setColor(r, g, b);
}
