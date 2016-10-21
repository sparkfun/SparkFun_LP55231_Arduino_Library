/******************************************************************************
Engines-parallel.ino
Application demonstrating all three engines at the same time.

Byron Jacquot @ SparkFun Electronics
October 21, 2016
https://github.com/sparkfun/SparkFun_LP55231_Arduino_Library


Multiple execution engine example.

loads three engines in patallel, and sets them all running.
- Engine one fades first blue LED in and out.
- Engine two blinks LED 2 between green and blue.
- Engine three blinks LED 3 in a faster red.

Multi-engine programs have a few particular details that need te be understood.

After loading the program, the load/verify routines leave the execution engines
in hold state.  Apparently necessary to allow PC/entry point setting, and to set them to
running.

Entry points have a funny side effect: the engine operates relative to the entry
point, NOT the absolute program memory address.
IE: the program counter for the engine is relative to the entry point.  In the program
we're using here, engine 1 takes a start point of 4.  It will initialize, and report a
PC of 0, which is actually the offset into memory from the entrry point.  If you
observe the PC as it rungs, it will be in the range 0..7.

The branch also
branches to 0, which is relative ot the entry point, meaning instruction address 4.

The sequence of operations is a rigid/fragile.  Getting things out of order means they
probably won't work.
In general:
-load the program (optionally, verify),
-set the relevant entry points and PCs,
-set execution mode (usually free-running), 
-then set them running.

You'll also gind some commented-out chunkso fo code that would allow a demonstration
of the "step" execution mode.  If you set engine 3 to step, then you can see that
it's incremented exactly one instruction


Resources:
Written using SparkFun Pro Micro controller, with LP55231 breakout board.

Development environment specifics:
Written using Arduino 1.6.5

This code is released under the [MIT License](http://opensource.org/licenses/MIT).

Please review the LICENSE.md file included with this example. If you have any questions
or concerns with licensing, please contact techsupport@sparkfun.com.

Distributed as-is; no warranty is given.
******************************************************************************/

#include <lp55231.h>

#include <Wire.h>

static const int32_t enable_pin = 15; // active high




static uint32_t next;
//static bool     ledon;

static Lp55231Engines ledChip(0x32);

static const uint16_t program[] =
{
  // Engine one
  0x9D02, // 0 map direct
  0x18ff, // 1 ramp up
  0x19ff, // 2 ramp dn
  0xa000, // 3 loop to 0

  // Engine two
  0x9D03, // 4 map direct
  0x18ff, // 5 ramp up
  0x19ff, // 6 ramp dn
  0x1d00, // 7 wait...
  0x9D04, // 8 map direct
  0x18ff, // 9 ramp up
  0x19ff, // a ramp dn
  0xa000, // b loop to 4

  // Engine three
  0x9d09, // c map direct
  0x04ff, // d ramp up
  0x05ff, // e ramp dn
  0xa001 // f loop to d (skip map instr...)
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

  ledChip.ClearInterrupt();

  ledChip.SetEngineEntryPoint(0, 0);
  ledChip.SetEnginePC(0, 0);

  ledChip.SetEngineEntryPoint(1, 4);
  ledChip.SetEnginePC(1, 4);

  ledChip.SetEngineEntryPoint(2, 0x0c);
  ledChip.SetEnginePC(2, 0x0c);

  ledChip.SetEngineModeFree(0);
  ledChip.SetEngineModeFree(1);
  //ledChip.setEngineModeStep(2);
  ledChip.SetEngineModeFree(2);
  // Tried "once" mode, but it's not very useful...

  ledChip.SetEngineRunning(0);
  ledChip.SetEngineRunning(1);
  ledChip.SetEngineRunning(2);

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
    Serial.print(ledChip.GetEngineMode(0));
    Serial.print(" ");
    Serial.print(ledChip.GetEnginePC(1));
    Serial.print(" ");
    Serial.print(ledChip.GetEngineMode(1));
    Serial.print(" ");
    Serial.print(ledChip.GetEnginePC(2));
    Serial.print(" ");
    Serial.println(ledChip.GetEngineMode(2));

    // Single step sets engine mode to hold at end.
    // So we need to keep setting it.
    //ledChip.setEngineModeStep(2);
    //ledChip.setEngineRunning(2);
  }
}
