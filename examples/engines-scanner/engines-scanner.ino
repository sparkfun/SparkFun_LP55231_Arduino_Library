/******************************************************************************
Engines-scaner.ino
9-channel LED scanner using LP55231.

Byron Jacquot @ SparkFun Electronics
October 21, 2016
https://github.com/sparkfun/SparkFun_LP55231_Arduino_Library


Using output maps for the execution engine, sequentially cycles through the 9
LED outputs, resulting in a scrolling pattern.  See the LP55231 breakout board
hookup guide for a  detailed examination of how this works.


Resources:
Written using SparkFun Pro Micro controller, with LP55231 breakout board.

Development environment specifics:
Written using Arduino 1.6.5

This code is released under the [MIT License](http://opensource.org/licenses/MIT).

Please review the LICENSE.md file included with this example. If you have any questions
or concerns with licensing, please contact techsupport@sparkfun.com.

Distributed as-is; no warranty is given.
******************************************************************************/


#include <Wire.h>

#include <lp55231.h>

static const int32_t enable_pin = 15; // Apparently active high?
static const int32_t trigger_pin = 14; // low if unused
static const int32_t interrupt_pin = 16;


static uint32_t next;
//static bool     ledon;

static Lp55231Engines ledChip(0x32);

static const uint16_t program[] =
{
  0x9c10, // 0 map start
  0x9c9f, // 1 map end
  0x02ff, // 2 ramp up
  0x0800, // 3 wait
  0x03ff, // 4 ramp down
  0x9d80, // 5 map next
  0xa002, // 6 loop to 2
  0x000a, // 7 - empty placeholder
  0x0005, // 8 - empty placeholder
  0x000a, // 9 - empty placeholder
  0x0005, // a - empty placeholder
  0x000a, // b - empty placeholder
  0x0005, // c - empty placeholder
  0x000a, // d - empty placeholder
  0x0005, // e - empty placeholder
  0x000a, // f - empty placeholder
  0x0001, // 10 map begin - start of 2nd page
  0x0002, // 11
  0x0040, // 12
  0x0004, // 13
  0x0008, // 14
  0x0080, // 15
  0x0010, // 16
  0x0020, // 17
  0x0100, // 18
  0x0020, // 19
  0x0010, // 1a
  0x0080, // 1b
  0x0008, // 1c
  0x0004, // 1d
  0x0040, // 1e
  0x0002, // 1f  map end
};


void setup()
{
  Serial.begin(9600);

  delay(5000);
  Serial.println("### Setup entry");

  pinMode(enable_pin, OUTPUT);
  digitalWrite(enable_pin, LOW);
  digitalWrite(enable_pin, HIGH);

  ledChip.Begin();
  ledChip.Enable();

  // Chip needs a moment to wake up.

  delay(1000);

  ledChip.ClearInterrupt();

  for(uint8_t i = 0; i < 9; i++)
  {
    ledChip.SetLogBrightness(i, true);
    ledChip.SetDriveCurrent(i, 111);
  }

  if(ledChip.LoadProgram(program, (sizeof(program)/2)))
  {
    Serial.println("Program loaded?");

    if(ledChip.VerifyProgram(program, (sizeof(program)/2)))
    {
      Serial.println("program verifies");
    }
  }
  else
  {
    Serial.println("Program dodn't load?");
  }

  next = millis() + 3000;

//  ledChip.clearInterrupt();

  ledChip.SetEngineEntryPoint(0, 0);
  ledChip.SetEnginePC(0, 0);

  ledChip.SetEngineModeFree(0);
  ledChip.SetEngineRunning(0);

  Serial.println("### Setup complete");

}



void loop()
{
  int32_t result;
  int8_t  val;
  static uint32_t count = 0;

  if(millis() >= next)
  {
    next += 1000;
    count++;

    Serial.print("# ");
    Serial.println(count);

    Serial.print(ledChip.GetEnginePC(0));
    Serial.print(" ");
    Serial.println(ledChip.GetEngineMode(0));

  }
}
