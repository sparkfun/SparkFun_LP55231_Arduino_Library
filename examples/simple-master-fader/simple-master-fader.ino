/******************************************************************************
simple-demo.ino
simple demo of using LP55231 to control 9 LEDs.
Byron Jacquot @ SparkFun Electronics
October 12, 2016
https://github.com/sparkfun/SparkFun_LP55231_Arduino_Library

The simplest demo of LP55231 functionality.  Initializes the chip, then
sequentially turn on each of the 9 channels.

Resources:
Written using SparkFun Pro Micro controller, with an LP55231 breakut board.

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

  ledChip.SetChannelPWM(0,0xff);
  ledChip.SetChannelPWM(1,0x00);
  ledChip.SetChannelPWM(6,0x80);

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
    next += 100;

    Serial.print("Illuminating: ");
    Serial.println(current);

    ledChip.SetMasterFader(0, current);

    current++;
  }
}
