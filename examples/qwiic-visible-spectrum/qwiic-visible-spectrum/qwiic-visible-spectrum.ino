/*
  This is example code for the LP55231-based Qwiic Visible Spectrum Emitter
  SparkFun sells these at its website: www.sparkfun.com
  Do you like this library? Help support SparkFun.
  https://www.sparkfun.com/products/21316
  
  Written by Nick Poole @ SparkFun Electronics, February 2, 2023
  https://github.com/sparkfun/SparkFun_LP55231_Arduino_Library
  
  Development environment specifics:
  Arduino IDE 1.8.19
  
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.
  
  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <Wire.h>
#include <lp55231.h>

// This is the default address for the Visible Spectrum board
Lp55231 ledChip(0x35); 

// This array returns the channel number for each LED in spectral order from Royal Blue to Far Red
int channel[] = {2,1,7,3,4,8,5,6,9};

void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);
  while(!Serial);
  Serial.println("-- Starting Setup() --");

  ledChip.Begin();
  ledChip.Enable();
  // The LP55231 charge pump should be disabled to avoid instability
  ledChip.SetChargePumpMode(CP_BYPASS);

  // Give things half a sec to settle
  delay(500);

  // Configure each LED channel for logarithmic brightness control
  for(uint8_t i = 0; i < 9; i++)
  {
    ledChip.SetLogBrightness(i, true);
  }

  Serial.println("-- Setup() Complete --");

}


// Flash each LED in turn and then ramp all LEDs from 0 to full brightness
void loop() {

  for(int c=0; c<9; c++){
    ledChip.SetChannelPWM(channel[c]-1, 200);
    delay(200);
    ledChip.SetChannelPWM(channel[c]-1, 0);
  }

  for(int p=0; p<255; p++){
    for(int c=0; c<9; c++){
     ledChip.SetChannelPWM(c, p);
    }
  }

  for(int p=0; p<255; p++){
    for(int c=0; c<9; c++){
     ledChip.SetChannelPWM(c, 255-p);
    }
  }  

  for(int c=0; c<9; c++){
   ledChip.SetChannelPWM(c, 0);
  }

  delay(200);
  
}
