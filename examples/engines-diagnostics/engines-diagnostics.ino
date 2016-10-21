/******************************************************************************
Engines-diagnostics.ino
Demonstration of diagnostic features onboard the LP55231 IC.
Byron Jacquot @ SparkFun Electronics
October 21, 2016
https://github.com/sparkfun/SparkFun_LP55231_Arduino_Library

An example of the LP55231 diagnostics.

The LP55231 includes an internal analog-to-digital converter.  Its input
is multiplexed among 12 input sources:
   - the 9 LEd outputs
   - the Vcc input
   - the Vout charge pump output voltage
   - the interrupt pin.

Additionally, there is also an internal temperature sensor.

Using this ADC, it is possible to determine if the LED outputs
are open (no LED populated) or shorted to ground, as well as measuring the
forward voltage on the channel when the LED is properly connected.

This example reads the temperature and pin supplies, then cycles through the LED
outputs, activating a channel, and reading the resulting voltage.
With the default LEDs on the breakout, channels 0 through 5 (the  green
and blue channels ) will read in the
neighborhood of 3.5V, and 6 through 9 (the red channels) roughly 2.1V.

If you want to see what a shorted output looks like, you can use a short piece of
wire to bridge one of the output pins to ground, and that chanel will read close to 0V
when tested.

An open output can be created by cutting an LED disconnect jumper.  Open
green and blue outputs will read close to the charge pump voltage, and
open red outputs will read close to VCC.

One last thing worth noting:
The interrupt pin can be used an an external input to the execution engines.
Instructions that use variables can be set to use the value in the ADC
conversion as a variable: Use variable selection bits 0x11, and set the MSB
of REG_MISC.  When variable D is referenced in programs, they will read
this input.  This could allow control of program behavior
via a manual potentiometer, or an
analog signal from anther circuit.

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

static uint32_t next;

static Lp55231Engines ledChip(0x32);


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
    ledChip.SetDriveCurrent(i, 0xff);
  }

  delay(1000);

  Serial.print("Deg C: ");
  Serial.println(ledChip.ReadDegC());
  Serial.print("Supply voltage: ");
  Serial.println(ledChip.ReadVddADC());
  Serial.print("Pump voltage: ");
  Serial.println(ledChip.ReadVoutADC());
  Serial.print("Interrupt pin voltage: ");
  Serial.println(ledChip.ReadVoutADC());

  Serial.println("### Setup complete");
  next = millis() + 3000;

}



void loop()
{
  static uint32_t count = 0;

  if(millis() >= next)
  {
    next += 1000;
    Serial.print("# ");
    Serial.print(count);

    Serial.print(" Deg C: ");
    Serial.println(ledChip.ReadDegC());


    ledChip.SetChannelPWM((count-1) % Lp55231::NumChannels, 0x0);
    ledChip.SetChannelPWM(count % Lp55231::NumChannels, 0xff);

    Serial.print("Channel: ");
    Serial.print(count % Lp55231::NumChannels);
    Serial.print(" LED forward voltage: ");
    Serial.println(ledChip.ReadLEDADC(count % Lp55231::NumChannels));

    count++;

  }
}
