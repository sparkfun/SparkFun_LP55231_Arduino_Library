/******************************************************************************
simple-master-fader.ino
simple demo of using LP55231 master faders
Byron Jacquot @ SparkFun Electronics
October 21, 2016
https://github.com/sparkfun/SparkFun_LP55231_Arduino_Library

Demonstration of LP55231 master fader functionality.

The first RGB LED is configured to a color mix of 100% red, 50% blue and 25% green, a
pinkish-white tint.
Those channels are assigned to the same master fader, and the fader output is incremented.
The result is that the LED maintains the color balance as it fades, remaining pinkish-white
at increasing intensities.

Resources:
Written using SparkFun Pro Micro controller, with an LP55231 breakout board.

Development environment specifics:
Written using Arduino 1.6.5


This code is released under the [MIT License](http://opensource.org/licenses/MIT).

Please review the LICENSE.md file included with this example. If you have any questions
or concerns with licensing, please contact techsupport@sparkfun.com.

Distributed as-is; no warranty is given.
******************************************************************************/

#include <Wire.h>
#include <lp55231.h>

Lp55231 ledChip;

void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);
  delay(5000);
  Serial.println("-- Starting Setup() --");

  ledChip.Begin();
  ledChip.Enable();

  ledChip.AssignChannelToMasterFader(0, 0);
  ledChip.AssignChannelToMasterFader(1, 0);
  ledChip.AssignChannelToMasterFader(6, 0);

  ledChip.SetLogBrightness(0, true);
  ledChip.SetLogBrightness(1, true);
  ledChip.SetLogBrightness(6, true);

  ledChip.SetDriveCurrent(0, 0xff);
  ledChip.SetDriveCurrent(1, 0xff);
  ledChip.SetDriveCurrent(6, 0xff);

  ledChip.SetChannelPWM(0,0x80);
  ledChip.SetChannelPWM(1,0x40);
  ledChip.SetChannelPWM(6,0xff);

  delay(500);

  Serial.println("-- Setup() Complete --");

}

void loop() {
  // put your main code here, to run repeatedly:

  // current will track the LED we're turning on
  // previous will keep track of the last one we turned on to turn it off again

  static uint8_t current = 0, previous = 0;
  static uint32_t next = millis()+1000;

  if(millis() >= next)
  {
    next += 50;

    Serial.print("Illuminating: ");
    Serial.println(current);

    ledChip.SetMasterFader(0, current);

    current++;
  }
}
