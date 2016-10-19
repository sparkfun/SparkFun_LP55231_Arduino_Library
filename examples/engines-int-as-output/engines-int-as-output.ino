#include <lp55231.h>

#include <Wire.h>

static const int32_t enable_pin = 15; // Apparently active high?
static const int32_t trigger_pin = 14; // low if unused
static const int32_t interrupt_pin = 16;


static uint32_t next;
static Lp55231Engines ledChip(0x32);

// This is a simple example of an esiteric feature.
//
// We're repurposing the interrupt pin (normally tied to the execution engines)
// as a general purpose output, under control of the host microcontroller.
//
// The host takes control of it by calling `OverrideIntToGPO()`.
// It can then set the output state of the pin by calling `SetIntGPOVal()`


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

  delay(1000);

  for(uint8_t i = 0; i < 9; i++)
  {
    ledChip.SetDriveCurrent(i, 111);
  }

  ledChip.OverrideIntToGPO(true);

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

    ledChip.SetIntGPOVal(count & 0x01);

    prev = count % 9;
    count++;
    count %= 9;
    next += 1000;

  }
}
