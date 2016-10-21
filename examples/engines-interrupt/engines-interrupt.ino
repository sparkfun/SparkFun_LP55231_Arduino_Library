/******************************************************************************
Engines-interrupt.ino
Application demonstrating interrupt pin for engine signal to host.
Byron Jacquot @ SparkFun Electronics
October 21, 2016
https://github.com/sparkfun/SparkFun_LP55231_Arduino_Library


Demonstrating the "interrupt" feature.
Confision ensues.

The interrupt featuere of the LP55231 isn't entirely simple - it's not
clearly documented, and has side effects.

First thing to note: page 1 of the datasheet indicates that
int is used to inform the host the program execution is complete.  It might not serve as a
very good waypoint in the middle of a program, due to the side effects.

Second thing to note: the datasheet appears to have transposed the row labels between
the "INT" and "END" in table 6.  End is apparently 0xC400, Int is 0xC000
with optional flags of 0x1000 (int) and 0x0800 (reset)

If the int flag in the instruction isn't set, I'm not sure what it does.

If the reset flag is set, it resets the engine PC, as well as the engine.
It also looks like the mapping values get cleared, and maybe the engine's variable....making
it hard to presist a map across a reset.

If you're going to stat over from scratch after an interrupt, the reset bit is probably useful.
If you're looking to retrigger the same program, there are some loose ends that need handling.
- clear the interrupt source
- set the PC to the new starting point
- set the engine exec mode to free
- start the engine running

As such, this example is showing the INT instruction flag with the INT bit set, but not reset.

When interrupt is detected, the handler sets PC to 2, the first instruction after the table.  This prevents
it from executing the map start instruction, which resets the map pointer, so it can increment
through the map.

Ultimately, the trigger pin & instruction might be more useful for this sort
of interlock behavior, because
it doesn't require nearly as much cleanup.


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

static const int32_t enable_pin = 15; // Apparently active high?
static const int32_t trigger_pin = 14; // low if unused
static const int32_t interrupt_pin = 16;

static uint32_t next;

static Lp55231Engines ledChip(0x32);

static const uint16_t program[] =
{
  0x9c09, // 0 map start addr
  0x9c8c, // 1 map end addr
  0xf000, // 2 wait for trigger
  0x0eff, // 3 ramp up over 255
  0x3000, // 4 wait
  0x0fff, // 5 ramp down over 255
  0x9d80, // 6 mux map next
  0xD000, // 7 interrupt
  0xa002, // 8 loop to 2
  0x01ff, // 9 channel map - all
  0x01c0, // a channel map - reds
  0x0015, // b channel map - greens
  0x002a, // c channel map - blues
};


void setup()
{
  Serial.begin(9600);

  delay(5000);
  Serial.println("### Setup entry");

  pinMode(enable_pin, OUTPUT);
  digitalWrite(enable_pin, LOW);
  digitalWrite(enable_pin, HIGH);

  // Trigger is active hi, so unused means low
  // but External trigger signal is active low
  pinMode(trigger_pin, OUTPUT);
  digitalWrite(trigger_pin, HIGH);

  pinMode(interrupt_pin, INPUT);

  ledChip.Begin();
  ledChip.Enable();

  delay(1000);

  for(uint8_t i = 0; i < 9; i++)
  {
    ledChip.SetLogBrightness(i, true);
    ledChip.SetDriveCurrent(i, 111);
  }

  if(ledChip.LoadProgram(program, 16))
  {
    Serial.println("Program loaded?");
  }
  else
  {
    Serial.println("Program dodn't load?");
  }

  next = millis() + 3000;

  ledChip.ClearInterrupt();

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
  static uint32_t count = 1;

  // Poll the interrupt line every time we get the chance.
  if( !digitalRead(interrupt_pin))
  {
    Serial.println("Interrupt asserted");

    Serial.println(ledChip.GetEngineMap(0), HEX);

    ledChip.ClearInterrupt();
    ledChip.SetEnginePC(0, 2);
    ledChip.SetEngineModeFree(0);
    ledChip.SetEngineRunning(0);
  }

  if(millis() >= next)
  {
    Serial.print("#");
    Serial.print(count);

    Serial.print(" PC:");
    Serial.println(ledChip.GetEnginePC(0));

    if(count % 4 == 0)
    {
      Serial.println("send trigger");
      digitalWrite(trigger_pin, LOW);
      delay(20);
      digitalWrite(trigger_pin, HIGH);

      Serial.println(ledChip.GetEngineMap(0), HEX);


    }

    count++;
    next += 1000;

  }
}
