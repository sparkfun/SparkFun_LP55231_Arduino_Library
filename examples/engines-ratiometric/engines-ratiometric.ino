/******************************************************************************
Engines-ratiometric.ino
Using the ratiometric bit to achieve engine-cntrolled ratiometric-color fades

Byron Jacquot @ SparkFun Electronics
October 21, 2016
https://github.com/sparkfun/SparkFun_LP55231_Arduino_Library



Ratiometric control is similar to the master-fader option, but applies
when outputs are under control of an execution engine.

Normally, when multiple outputs are mapped to an engine, they all get the
same value, as the engine only produces a single value.  If you map all 3 colors
of an RGB LED to an engine, the result will be white (the mix of equal amounts of RG & B)

If you want a specific mix of colors under engine control, you need to set up
that mix using the direct PWM registers, and set the channels to ratiometric
mode.

Then, the engine output value is multiplied by the PWM register values to
calculate the resulting blend.  The result is that the engine brightness is
scaled proportionally by the channel values, and the color of illumination
will be consistent, and it's brightness will vary.

In setup, the channel is set to ratiometric mode with the SetRatiometricDimming
method.  Also, the direct control registers need to be programmed for the
desired ratios using SetChannelPWM.

Then the engine is programmed and started.

This example products pastel output colors - light blue, light green and pink.
If you turn off the Ratiometric setting, they revert to white.

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

//#include "lp55231.h"

static const int32_t enable_pin = 15; // Apparently active high?
static const int32_t trigger_pin = 14; // low if unused
static const int32_t interrupt_pin = 16;


static uint32_t next;
//static bool     ledon;

static Lp55231Engines ledChip(0x32);


static const uint16_t program[] =
{
  0x9c09, // 0 map start addr
  0x9c89, // 1 map end addr
  0x3000, // 2 wait
  0x0eff, // 3 ramp up over 255
  0x3000, // 4 wait
  0x0fff, // 5 ramp down over 255
  0x9d80, // 6 mux map next
  0xf040, // 7 trigger - send to micro, and wait for response.
  0xa002, // 8 loop to 2
  0x01ff, // 9 channel map - all
  0x01c0, // a channel map - reds
  0x0015, // b channel map - greens
  0x002a, // c channel map - blues
};


void setup()
{
  Serial.begin(9600);

  delay(2000);
  Serial.println("### Setup entry");

  pinMode(enable_pin, OUTPUT);
  digitalWrite(enable_pin, LOW);
  digitalWrite(enable_pin, HIGH);

  // Trigger is active hi, so unused means low
  // but External trigger signal is active low
  pinMode(trigger_pin, INPUT_PULLUP);
  //digitalWrite(trigger_pin, HIGH);

  pinMode(interrupt_pin, INPUT);

  ledChip.Begin();
  ledChip.Enable();

  delay(1000);

  ledChip.SetChannelPWM(0, 0xff);
  ledChip.SetChannelPWM(1, 0x80);
  ledChip.SetChannelPWM(2, 0x80);
  ledChip.SetChannelPWM(3, 0xff);
  ledChip.SetChannelPWM(4, 0x40);
  ledChip.SetChannelPWM(5, 0x80);
  ledChip.SetChannelPWM(6, 0x40);
  ledChip.SetChannelPWM(7, 0x40);
  ledChip.SetChannelPWM(8, 0xff);

  for (uint8_t i = 0; i < Lp55231::NumChannels; i++)
  {
    //ledChip.SetLogBrightness(i, true);
    ledChip.SetDriveCurrent(i, 111);
    ledChip.SetRatiometricDimming(i, true);
  }

#if 1
  if (ledChip.LoadProgram(program, 16))
  {
    Serial.println("Program loaded?");
  }
  else
  {
    Serial.println("Program dodn't load?");
  }
#endif

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

  if ( !digitalRead(trigger_pin))
  {
    Serial.println("trigger");

    while (!digitalRead(trigger_pin))
      ;

    delay(5);
    digitalWrite(trigger_pin, LOW);
    delay(5);
    digitalWrite(trigger_pin, HIGH);

  }

  if (millis() >= next)
  {
    Serial.print("#");
    Serial.print(count);

    Serial.print(" PC:");
    Serial.println(ledChip.GetEnginePC(0));

    count++;
    next += 1000;

  }
}
