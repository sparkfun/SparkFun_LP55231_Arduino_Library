/******************************************************************************
simple-two-chips.ino
simple demo of using two LP55231s in parallel to control 18 LEDs.
Byron Jacquot @ SparkFun Electronics
October 21, 2016
https://github.com/sparkfun/SparkFun_LP55231_Arduino_Library

Demonstration of two LP55231s on an I2C bus.

Two Lp55231 objects are declared, at unique I2C addresses.  The program then
writes different values to each chip, resulting an a chase pattern in which one
chip appears to be a step behind the other.

Resources:
Written using SparkFun Pro Micro controller, with two LP55231 breakout boards.

The second Lp55231 board is configured for address 0x33, and the first board has
the I2C pullup resistors disconnected.

Development environment specifics:
Written using Arduino 1.6.5


This code is released under the [MIT License](http://opensource.org/licenses/MIT).

Please review the LICENSE.md file included with this example. If you have any questions
or concerns with licensing, please contact techsupport@sparkfun.com.

Distributed as-is; no warranty is given.
******************************************************************************/

#include <lp55231.h>

#include <Wire.h>

#include "lp55231.h"

static const int32_t enable_pin = 15; // Apparently active high?

// A quick example demonstrating two LP55231's on the same I2C bus.
//
// One of them will need to be reconfigured to address 0x33, by changing
// Address Jumper A0.
//
// to interface them, two Lp55231 objects are declared, each with their own address.
//
// It will simply chase among the LED outputs.  The second chip will be one LED
// ahead of the other, and the chip are set to opposite ends of the range of
// output current, making the first chip brighter than the second..

static uint32_t next;

static Lp55231 ledChip(0x32);
static Lp55231 ledChip2(0x33);

void setup()
{
  Serial.begin(9600);

  delay(2000);
  Serial.println("### Setup entry");

  pinMode(enable_pin, OUTPUT);
  digitalWrite(enable_pin, LOW);
  digitalWrite(enable_pin, HIGH);

  ledChip.Begin();
  ledChip.Enable();

  ledChip2.Begin();
  ledChip2.Enable();

  delay(1000);

  for(uint8_t i = 0; i < 9; i++)
  {
    ledChip.SetDriveCurrent(i, 0xff);
    ledChip2.SetDriveCurrent(i, 1);
  }

  next = millis() + 3000;

  Serial.println("### Setup complete");

}



void loop()
{
  int32_t result;
  int8_t  val;
  static uint32_t count = 0;
  static uint32_t prev = 0;

  if(millis() >= next)
  {
    Serial.print("#");
    Serial.println(count);

    ledChip.SetChannelPWM(prev, 0);
    ledChip.SetChannelPWM(count, 0xff);

    prev = count % 9;
    count++;
    count %= 9;
    next += 1000;

    // Chip2 will lead chip1 by one LED.
    ledChip2.SetChannelPWM(prev, 0);
    ledChip2.SetChannelPWM(count, 0xff);
  }
}
