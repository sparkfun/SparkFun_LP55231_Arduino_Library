#include <Arduino.h>
#include <Wire.h>

#include "lp55231.h"

// register stuff
static const uint8_t REG_CNTRL1 = 0x00;
static const uint8_t REG_CNTRL2 = 0x01;
static const uint8_t REG_RATIO_MSB = 0x02;
static const uint8_t REG_RATIO_LSB = 0x03;
static const uint8_t REG_OUTPUT_ONOFF_MSB = 0x04;
static const uint8_t REG_OUTPUT_ONOFF_LSB = 0x05;

// Per LED control channels - fader channel assig, log dimming enable, temperature compensation
static const uint8_t REG_D1_CTRL = 0x06;
static const uint8_t REG_D2_CTRL = 0x07;
static const uint8_t REG_D3_CTRL = 0x08;
static const uint8_t REG_D4_CTRL = 0x09;
static const uint8_t REG_D5_CTRL = 0x0a;
static const uint8_t REG_D6_CTRL = 0x0b;
static const uint8_t REG_D7_CTRL = 0x0c;
static const uint8_t REG_D8_CTRL = 0x0d;
static const uint8_t REG_D9_CTRL = 0x0e;

// 0x0f to 0x15 reserved

// Direct PWM control registers
static const uint8_t REG_D1_PWM  = 0x16;
static const uint8_t REG_D2_PWM  = 0x17;
static const uint8_t REG_D3_PWM  = 0x18;
static const uint8_t REG_D4_PWM  = 0x19;
static const uint8_t REG_D5_PWM  = 0x1a;
static const uint8_t REG_D6_PWM  = 0x1b;
static const uint8_t REG_D7_PWM  = 0x1c;
static const uint8_t REG_D8_PWM  = 0x1d;
static const uint8_t REG_D9_PWM  = 0x1e;

// 0x1f to 0x25 reserved

// Drive current registers
static const uint8_t REG_D1_I_CTL = 0x26;
static const uint8_t REG_D2_I_CTL  = 0x27;
static const uint8_t REG_D3_I_CTL  = 0x28;
static const uint8_t REG_D4_I_CTL  = 0x29;
static const uint8_t REG_D5_I_CTL  = 0x2a;
static const uint8_t REG_D6_I_CTL  = 0x2b;
static const uint8_t REG_D7_I_CTL  = 0x2c;
static const uint8_t REG_D8_I_CTL  = 0x2d;
static const uint8_t REG_D9_I_CTL  = 0x2e;

// 0x2f to 0x35 reserved

static const uint8_t REG_MISC     = 0x36;
static const uint8_t REG_PC1      = 0x37;
static const uint8_t REG_PC2      = 0x38;
static const uint8_t REG_PC3      = 0x39;
static const uint8_t REG_STATUS_IRQ = 0x3A;
static const uint8_t REG_INT_GPIO   = 0x3B;
static const uint8_t REG_GLOBAL_VAR = 0x3C;
static const uint8_t REG_RESET      = 0x3D;
static const uint8_t REG_TEMP_CTL   = 0x3E;
static const uint8_t REG_TEMP_READ  = 0x3F;
static const uint8_t REG_TEMP_WRITE = 0x40;
static const uint8_t REG_TEST_CTL   = 0x41;
static const uint8_t REG_TEST_ADC   = 0x42;

// 0x43 to 0x44 reserved

static const uint8_t REG_ENGINE_A_VAR = 0x45;
static const uint8_t REG_ENGINE_B_VAR = 0x46;
static const uint8_t REG_ENGINE_C_VAR = 0x47;

static const uint8_t REG_MASTER_FADE_1 = 0x48;
static const uint8_t REG_MASTER_FADE_2 = 0x49;
static const uint8_t REG_MASTER_FADE_3 = 0x4A;

// 0x4b Reserved

static const uint8_t REG_PROG1_START = 0x4C;
static const uint8_t REG_PROG2_START = 0x4D;
static const uint8_t REG_PROG3_START = 0x4E;
static const uint8_t REG_PROG_PAGE_SEL = 0x4f;

// Memory is more confusing - there are 6 pages, sel by addr 4f
static const uint8_t REG_PROG_MEM_BASE = 0x50;
//static const uint8_t REG_PROG_MEM_SIZE = 0x;//
static const uint8_t REG_PROG_MEM_END  = 0x6f;

static const uint8_t REG_ENG1_MAP_MSB = 0x70;
static const uint8_t REG_ENG1_MAP_LSB = 0x71;
static const uint8_t REG_ENG2_MAP_MSB = 0x72;
static const uint8_t REG_ENG2_MAP_LSB = 0x73;
static const uint8_t REG_ENG3_MAP_MSB = 0x74;
static const uint8_t REG_ENG3_MAP_LSB = 0x75;

static const uint8_t REG_GAIN_CHANGE = 0x76;

/********************************************************************************/
//  Lp55231: The Simple base class.
//
// allows direct control over the LED outputs, and basic chip featurers like output current setting,
/********************************************************************************/

Lp55231::Lp55231(uint8_t address)
{
  _address = address;
}

void Lp55231::Begin()
{
  Wire.begin();

  Reset();
}

void Lp55231::Enable()
{
  // Set enable bit
  WriteReg(REG_CNTRL1, 0x40 );

  // enable internal clock & charge pump & write auto increment
  WriteReg(REG_MISC, 0x53);
}

void Lp55231::Disable()
{
  uint8_t val;

  val = ReadReg(REG_CNTRL1);
  val &= ~0x40;
  WriteReg(REG_CNTRL1, val);
}

void Lp55231::Reset()
{
  // force reset
  WriteReg(REG_RESET, 0xff);
}

bool Lp55231::SetChannelPWM(uint8_t channel, uint8_t value)
{
  if(channel >= NumChannels)
  {
    Serial.println("setBrightness: invalid channel");
    return false;
  }

  WriteReg(REG_D1_PWM + channel, value);
  return true;
}

bool Lp55231::SetMasterFader(uint8_t fader, uint8_t value)
{
  if(fader >= NumFaders)
  {
    return false;
  }

  WriteReg(REG_MASTER_FADE_1 + fader, value);
}

bool Lp55231::SetLogBrightness(uint8_t channel, bool enable)
{
  uint8_t regVal, bitVal;

  if(channel >= NumChannels)
  {
    return false;
  }

  regVal = ReadReg(REG_D1_CTRL + channel);
  bitVal = enable?0x20:0x00;
  regVal &= ~0x20;
  regVal |= bitVal;
  WriteReg(REG_D1_CTRL + channel, regVal);
}

bool Lp55231::SetDriveCurrent(uint8_t channel, uint8_t value)
{
  if(channel >= NumChannels)
  {
    return false;
  }

  WriteReg(REG_D1_I_CTL + channel, value);
  return true;
}


bool Lp55231::AssignChannelToMasterFader(uint8_t channel, uint8_t fader)
{
  uint8_t regVal, bitVal;

  if(channel >= NumChannels)
  {
    return false;
  }
  else if(fader >= NumFaders)
  {
    return false;
  }

  regVal = ReadReg(REG_D1_CTRL + channel);
  bitVal = (fader + 1) & 0x03;
  bitVal <<= 6;
  regVal &= ~0xc0;
  regVal |= bitVal;
  WriteReg(REG_D1_CTRL + channel, regVal);

}

/********************************************************************************/
/**  Engine related derived class functions. **/
/********************************************************************************/

// Ratiometric dimming is similar to master fader, but when LEDs are driven by
// Execution engines.
bool Lp55231Engines::SetRatiometricDimming(uint8_t channel, bool value)
{
  uint8_t regVal;

  if(channel >= NumChannels)
  {
    Serial.println("setLogBrightness: invalid channel");
    return false;
  }

  if(channel == NumChannels - 1)
  {
    regVal = ReadReg(REG_RATIO_MSB);
    if(value)
    {
      regVal |= 0x01;
    }
    else
    {
      regVal &= ~0x01;
    }
    WriteReg(REG_RATIO_MSB, regVal);
  }
  else
  {
    regVal = ReadReg(REG_RATIO_LSB);
    if(value)
    {
      regVal |= (0x01 << channel);
    }
    else
    {
      regVal &= ~(0x01 << channel);
    }
    WriteReg(REG_RATIO_LSB, regVal);
  }

  return true;
}

bool Lp55231Engines::LoadProgram(const uint16_t* prog, uint8_t len)
{
  uint8_t val;
  uint8_t page;

  if(len >= NumInstructions)
  {
    Serial.println("program too long");
    return false;
  }

  // set up program write
  // start in execution disabled mode (0b00)
  // required to get into load mode.
  // "Load program mode can be entered from the disabled mode only.  be
  // entered from the disabled mode only."
  WriteReg(REG_CNTRL2, 0x00);
  WriteReg(REG_CNTRL2, 0x15);

  WaitForBusy();

  // try to write program from example
  // datasheet says MSB of each instruction is in earlier address
  // TBD: could optimize with a sequence of byte writes, using auto increment

  // use auto-increment of chip - enabled in MISC.
  // If it gets turned off, this breaks.  TBD: set it explicitly?

  // Write complete pages, setting page reg for each.
  for(page = 0; page < (len/16); page++)
  {
    WriteReg(REG_PROG_PAGE_SEL, page);

    for(uint8_t i = 0; i < 16; i++)
    {
      Wire.beginTransmission(_address);
      Wire.write((REG_PROG_MEM_BASE + (i*2)));
      // MSB then LSB
      Wire.write((prog[(i + (page*16))]>> 8) & 0xff);
      Wire.write(prog[i + (page*16)] & 0xff);
      Wire.endTransmission();
    }
  }

  // plus any incomplete pages
  page = len/16;
  WriteReg(REG_PROG_PAGE_SEL, page);
  for(uint8_t i = 0; i < (len%16); i++)
  {
    Wire.beginTransmission(_address);
    Wire.write((REG_PROG_MEM_BASE + (i*2)));
    // MSB then LSB
    Wire.write((prog[i + (page*16)]>> 8) & 0xff);
    Wire.write(prog[i + (page*16)] & 0xff);
    Wire.endTransmission();
  }

  WriteReg(REG_CNTRL2, 0x00);

  return true;
}

bool Lp55231Engines::VerifyProgram(const uint16_t* prog, uint8_t len)
{
  uint8_t val, page;

  if(len >= NumInstructions)
  {
    // TBD - support multiple pages

    Serial.println("Verify program too long");
    return false;
  }

  WriteReg(REG_CNTRL2, 0x00);// engines into disable mode - required for entry to program mode.
  WriteReg(REG_CNTRL2, 0x15);// engines into program mode?
  //try to read  program from chip,
  // datasheet says MSB of each instruction is in earlier address
  // TBD: could optimize with a sequence of byte writes, using auto increment

  // Auto-increment may not work for sequential reads...
  for(page = 0; page < (len/16); page++)
  {
    WriteReg(REG_PROG_PAGE_SEL, page);

    for(uint8_t i = 0; i < 16; i++)
    {
      uint16_t msb, lsb;
      uint8_t addr = (REG_PROG_MEM_BASE + (i*2));
      //Serial.print("Verifying: ");
      //Serial.println(addr, HEX);

      msb = ReadReg(addr);
      lsb = ReadReg(addr + 1);

      lsb |= (msb << 8);

      if(lsb != prog[i + (page*16)])
      {
        Serial.print("program mismatch.  Idx:");
        Serial.print(i);
        Serial.print(" local:");
        Serial.print(prog[i + (page*16)], HEX);
        Serial.print(" remote:");
        Serial.println(lsb, HEX);

        return false;
      }
    }
  }

  // plus any incomplete pages
  page = len/16;
  WriteReg(REG_PROG_PAGE_SEL, page);
  for(uint8_t i = 0; i < (len%16); i++)
  {
    uint16_t msb, lsb;
    uint8_t addr = (REG_PROG_MEM_BASE + (i*2));
    Serial.print("Verifying: ");
    Serial.println(addr, HEX);

    msb = ReadReg(addr);
    lsb = ReadReg(addr + 1);

    lsb |= (msb << 8);

    if(lsb != prog[i + (page*16)])
    {
      Serial.print("program mismatch.  Idx:");
      Serial.print(i);
      Serial.print(" local:");
      Serial.print(prog[i + (page*16)], HEX);
      Serial.print(" remote:");
      Serial.println(lsb, HEX);

      return false;
    }
  }

  WriteReg(REG_CNTRL2, 0x00);

  return true;
}

bool Lp55231Engines::SetEngineEntryPoint(uint8_t engine, uint8_t addr)
{

  if(engine >= NumEngines)
  {
    Serial.println("Invalid engine num in set EP");
    return false;
  }

  WriteReg(REG_PROG1_START + engine, addr);

  return true;
}

bool Lp55231Engines::SetEnginePC(uint8_t engine, uint8_t addr)
{
  uint8_t control_val, control2_val, temp;;

  if(engine >= NumEngines)
  {
    Serial.println("Invalid engine num in set PC");
    return false;
  }

  // There are 6 pages of 16 instructions each (0..95)
  if(addr >= NumInstructions)
  {
    Serial.println("Invalid addr in set PC");
    return false;
  }

  // In Ctl1 descriptions:
  //00 = hold: Hold causes the execution engine to finish the current instruction and then stop. Program counter
  //(PC) can be read or written only in this mode.

  control_val = ReadReg(REG_CNTRL1);
  control2_val = ReadReg(REG_CNTRL2);

  temp = (control_val & ~(0x30 >> (engine * 2)));

  WriteReg(REG_CNTRL2, 0x3ff); // halt engines immediately.
  WriteReg(REG_CNTRL1, temp);// put engine in load mode

  WriteReg(REG_PC1 + engine, addr);

  // restore prev mode?
  WriteReg(REG_CNTRL1, control_val);
  WriteReg(REG_CNTRL2, control2_val);

  return true;
}

uint8_t Lp55231Engines::GetEnginePC(uint8_t engine)
{
  // must set Hold to touch PC...
  uint8_t control_val, pc_val;

  if(engine >= NumEngines)
  {
    Serial.println("Invalid engine num in set PC");
    return -1;
  }

  pc_val = ReadReg(REG_PC1 + engine);

  return(pc_val);
}


uint8_t Lp55231Engines::GetEngineMode(uint8_t engine)
{
  uint8_t val;

  if(engine >= NumEngines)
  {
    Serial.println("Get engine mode got invalid engine #");
    return false;
  }

  val = ReadReg(REG_CNTRL1);
  val >>= (engine * 2);
  val &= 0x03;
  return(val);

}


uint8_t Lp55231Engines::GetEngineMap(uint8_t engine)
{
  if(engine >= NumEngines)
  {
    Serial.println("Invalid engine num in get map");
    return -1;
  }

  return(ReadReg(REG_ENG1_MAP_LSB + engine));
}

bool Lp55231Engines::SetEngineModeHold(uint8_t engine)
{
  uint8_t val;

  if(engine >= NumEngines)
  {
    Serial.println("Set free got invalid engine #");
    return false;
  }

  // Set the enghine to "free running" execution type
  // bits to 0b00
  val = ReadReg(REG_CNTRL1);
  val &= ~(0x30 >> (engine * 2));
  //val |= (0x10 >> (engine * 2));
  WriteReg(REG_CNTRL1, val );

  return(true);
}

bool Lp55231Engines::SetEngineModeStep(uint8_t engine)
{
  uint8_t val;

  if(engine >= NumEngines)
  {
    Serial.println("Set free got invalid engine #");
    return false;
  }

  // Set the enghine to "single step" execution type
  // bits to 0b01
  val = ReadReg(REG_CNTRL1);
  val &= ~(0x30 >> (engine * 2));
  val |= (0x10 >> (engine * 2));
  WriteReg(REG_CNTRL1, val );

  return(true);
}

bool Lp55231Engines::SetEngineModeOnce(uint8_t engine)
{
  uint8_t val;

  // This mode might not be the most useful.
  // It executes the pointed instruction, then
  // sets exec mode to hold, and resets the PC.
  // It's an astringent form of step (which advances the PC, instead)

  if(engine >= NumEngines)
  {
    Serial.println("Set one shot got invalid engine #");
    return false;
  }

  // Set the enghine to "one shot" execution type
  // Bits to 0b11
  val = ReadReg(REG_CNTRL1);
  val |= (0x30 >> (engine * 2));

  Serial.print("C1: ");
  Serial.println(val, HEX);

  WriteReg(REG_CNTRL1, val );

  return(true);

}

bool Lp55231Engines::SetEngineModeFree(uint8_t engine)
{
  uint8_t val;

  if(engine >= NumEngines)
  {
    Serial.println("Set free got invalid engine #");
    return false;
  }

  // Set the enghine to "free running" execution type
  val = ReadReg(REG_CNTRL1);
  val &= ~(0x30 >> (engine * 2));
  val |= (0x20 >> (engine * 2));

  // Serial.print("Free: ");
  // Serial.print(engine, HEX);
  // Serial.print(" ");
  // Serial.println(val, HEX);

  WriteReg(REG_CNTRL1, val );

  return(true);
}

bool Lp55231Engines::SetEngineRunning(uint8_t engine)
{
  uint8_t val;

  if(engine >= NumEngines)
  {
    Serial.println("Set running got invalid engine #");
    return false;
  }

  // This assumes that a suitable run mode in CNTRL1 was already selected.
  // start execution by setting "run program" mode
  val = ReadReg(REG_CNTRL2);
  val &= ~(0x30 >> (engine * 2));
  val |= (0x20>> (engine * 2));
  WriteReg(REG_CNTRL2, val);

  return true;
}


/********************************************************************************/
/**  Derived class - interrupt related functions. **/
/********************************************************************************/

uint8_t Lp55231Engines::ClearInterrupt()
{
  // TBD: make this more channel specific?
  return( ReadReg(REG_STATUS_IRQ) & 0x07);
}

void Lp55231Engines::OverrideIntToGPO(bool overrideOn )
{
  uint8_t regVal;

  if(overrideOn)
  {
      regVal = 0x04;
  }
  else
  {
    regVal = 0;
  }

  WriteReg(REG_INT_GPIO, regVal);
}

bool Lp55231Engines::SetIntGPOVal(bool value)
{
  uint8_t regVal;

  regVal = ReadReg(REG_INT_GPIO);

  if (!(regVal & 0x04))
  {
    return false;
  }

  if(value)
  {
    regVal |= 0x01;
  }
  else
  {
    regVal &= ~0x01;
  }

  WriteReg(REG_INT_GPIO, regVal);
}


/********************************************************************************/
/**  Derived class - Diagnostic functions. **/
/********************************************************************************/

int8_t Lp55231Engines::ReadDegC()
{
  uint8_t status;
  int8_t  temperature;

  WriteReg(REG_TEMP_CTL, 0x04);

  do
  {
    status = ReadReg(REG_TEMP_CTL);
    //Serial.print(".");
  }while(status & 0x80);

  temperature = (int8_t)ReadReg(REG_TEMP_READ);

  return temperature;
}

float  Lp55231Engines::ReadLEDADC(uint8_t channel)
{
  uint8_t reading;
  float volts;

  if(channel >= NumChannels)
  {
    return 0.0;
  }

  reading = ReadADCInternal(channel & 0x0f);

  volts = (reading * 0.03) - 1.478;
  return volts;
}

float  Lp55231Engines::ReadVoutADC()
{
  uint8_t reading;
  float volts;

  reading = ReadADCInternal(0x0f);

  volts = (reading * 0.03) - 1.478;
  return volts;
}

float  Lp55231Engines::ReadVddADC()
{
  uint8_t reading;
  float volts;

  reading = ReadADCInternal(0x10);

  volts = (reading * 0.03) - 1.478;
  return volts;
}

float  Lp55231Engines::ReadIntADC()
{
  // reads voltage at interrupt pin

  uint8_t reading;
  float volts;

  reading = ReadADCInternal(0x11);

  volts = (reading * 0.03) - 1.478;
  return volts;
}



/********************************************************************************/
/**  private base class member functions. **/
/********************************************************************************/

uint8_t Lp55231::ReadReg(uint8_t reg)
{
  // Wire is awkward because it doesn't really have a register address concept.
  // http://www.arduino.cc/en/Tutorial/SFRRangerReader for reference

  Wire.beginTransmission(_address);
  Wire.write(reg);
  Wire.endTransmission(false);// false keeps connection active so we can read.

  delayMicroseconds(10);

  uint8_t status = Wire.requestFrom(_address, (uint8_t)1);
  if(status)
  {
    return(Wire.read());
  }
  else
  {
    Serial.print("readReg failed? status:");
    Serial.println(status, HEX);
  }
  return 0xff;
}

void Lp55231::WriteReg(uint8_t reg, uint8_t val)
{
  Wire.beginTransmission(_address);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}


/********************************************************************************/
/**  private derived class member functions. **/
/********************************************************************************/

void Lp55231Engines::WaitForBusy()
{
  uint8_t val;

  // then wait to change modes
  do
  {
    val = ReadReg(REG_STATUS_IRQ) & 0x10; // engine busy bit
  }
  while(val);

}



uint8_t Lp55231Engines::ReadADCInternal(uint8_t channel)
{
  WriteReg(REG_TEST_CTL, 0x80 |(channel & 0x1f));

  // No reg bit to poll for completing - simply delay.
  delay(3);

  return(ReadReg(REG_TEST_ADC));

}
