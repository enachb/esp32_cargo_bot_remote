#include <nRF24L01.h>
#include <printf.h>
#include <RF24.h>
#include <RF24_config.h>
#include <EEPROM.h>

#include <math.h>
#include "Wire.h"
//#include "WiiChuckClass.h" //most likely its WiiChuck.h for the rest of us.
#include "WiiChuck.h"

#define MAXSPEED              1000
#define EEPROMADDR             13

#define FILTER              0.1
#define SPEED_COEFFICIENT   0.5
#define STEER_COEFFICIENT   0.5

//#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

// TEMP VARIABLES
float   nMotPremixL;    // Motor (left)  premixed output        (-128..+127)
float   nMotMixR;    // Motor (right) premixed output        (-128..+127)
float   nMotMixL;    // Motor (left)  premixed output        (-128..+127)
float   nMotPremixR;    // Motor (right) premixed output        (-128..+127)
int     nPivSpeed;      // Pivot Speed                          (-128..+127)
float   fPivScale;      // Balance scale b/w drive and pivot    (   0..1   )

// CONFIG
// - fPivYLimt  : The threshold at which the pivot action starts
//                This threshold is measured in units on the Y-axis
//                away from the X-axis (Y=0). A greater value will assign
//                more of the joystick's range to pivot actions.
//                Allowable range: (0..+127)
#define fPivYLimit 32.0


WiiChuck chuck = WiiChuck();

struct metricsStruct {
  int16_t leftMotor;
  int16_t rightMotor;
};

metricsStruct metrics = {0, 0};

// Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins 7 & 8
RF24 radio(7, 8);

// Topology
const uint64_t pipe = 0xABBDABCD71LL;              // Radio pipe addresses for the 2 nodes to communicate.

uint16_t count = 0;
long currMax = 250;

int current;         // Current state of the button

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setup() {
  //nunchuck_init();
  Serial.begin(115200);
  Serial.print("Serial init");
  chuck.begin();
  Serial.print("Nunhcuck init");
  chuck.update();
  chuck.calibrateJoy();

  radio.begin();
  radio.setChannel(45);
  radio.openWritingPipe(pipe);        // Both radios listen on the same pipes by default, and switch when writing
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  radio.stopListening();                                  // First, stop listening so we can talk.
  radio.printDetails();                   // Dump the configuration of the rf unit for debugging

  pinMode(13, OUTPUT);

  currMax = EEPROMReadlong(EEPROMADDR);
}

void steering(float nJoyX, float nJoyY) {
  /* Serial.print("  +x ");
    Serial.print(x);
    Serial.print(" +y ");
    Serial.print(y);
    Serial.print(" ");
  */

  // Calculate Drive Turn output due to Joystick X input
  if (nJoyY >= 0) {
    // Forward
    nMotPremixL = (nJoyX >= 0) ? 127.0 : (127.0 + nJoyX);
    nMotPremixR = (nJoyX >= 0) ? (127.0 - nJoyX) : 127.0;
  } else {
    // Reverse
    nMotPremixL = (nJoyX >= 0) ? (127.0 - nJoyX) : 127.0;
    nMotPremixR = (nJoyX >= 0) ? 127.0 : (127.0 + nJoyX);
  }

  // Scale Drive output due to Joystick Y input (throttle)
  nMotPremixL = nMotPremixL * nJoyY / 128.0;
  nMotPremixR = nMotPremixR * nJoyY / 128.0;

  // Now calculate pivot amount
  // - Strength of pivot (nPivSpeed) based on Joystick X input
  // - Blending of pivot vs drive (fPivScale) based on Joystick Y input
  nPivSpeed = nJoyX;
  fPivScale = (abs(nJoyY) > fPivYLimit) ? 0.0 : (1.0 - abs(nJoyY) / fPivYLimit);

  // Calculate final mix of Drive and Pivot
  nMotMixL = (1.0 - fPivScale) * nMotPremixL + fPivScale * ( nPivSpeed);
  nMotMixR = (1.0 - fPivScale) * nMotPremixR + fPivScale * (-nPivSpeed);

  // Convert to Motor PWM range
  metrics.leftMotor = mapFloat(nMotMixL, -128, 128, -currMax, currMax);
  metrics.rightMotor = mapFloat(nMotMixR, -128, 128, -currMax, currMax);
}

void loop() {
  delay(20);
  chuck.update();

  steering(chuck.readJoyY(), chuck.readJoyX());

  if (count % 10 == 0) {
    Serial.print(chuck.readJoyX());
    Serial.print(", ");
    Serial.print(chuck.readJoyY());
    Serial.print(", ");
    Serial.print(metrics.leftMotor);
    Serial.print(", ");
    Serial.print(metrics.rightMotor);
    Serial.print(", ");
    Serial.print(chuck.buttonZ);
    Serial.print(", ");
    Serial.print(currMax);
    Serial.println("");
    //  Serial.print(" sizeof: ");
    //  Serial.print(sizeof(motors))
    //  Serial.printLN(" ");
    count = 1;
  } else {
    count++;
  }

  // send it
  if (!radio.write(&metrics, sizeof(metrics) )) {
    Serial.println(F("failed."));
  }

  // Set max speed after devouncing button
  // *************************************
  long startTime = millis();
  long endTime = millis();
  while (chuck.buttonZ) {
    delay(50);
    chuck.update();
    // Long press
    if (millis() - startTime > 1000) {
      if (currMax == 250) {
        currMax = 1000;
      } else {
        currMax = 250;
      }
      EEPROMWritelong(EEPROMADDR, currMax);
      startTime = millis();
    }

  }

}

void EEPROMWritelong(int address, long value)
{
  //Decomposition from a long to 4 bytes by using bitshift.
  //One = Most significant -> Four = Least significant byte
  byte four = (value & 0xFF);
  byte three = ((value >> 8) & 0xFF);
  byte two = ((value >> 16) & 0xFF);
  byte one = ((value >> 24) & 0xFF);

  //Write the 4 bytes into the eeprom memory.
  EEPROM.write(address, four);
  EEPROM.write(address + 1, three);
  EEPROM.write(address + 2, two);
  EEPROM.write(address + 3, one);
}

long EEPROMReadlong(long address)
{
  //Read the 4 bytes from the eeprom memory.
  long four = EEPROM.read(address);
  long three = EEPROM.read(address + 1);
  long two = EEPROM.read(address + 2);
  long one = EEPROM.read(address + 3);

  //Return the recomposed long by using bitshift.
  return ((four << 0) & 0xFF) + ((three << 8) & 0xFFFF) + ((two << 16) & 0xFFFFFF) + ((one << 24) & 0xFFFFFFFF);
}

void ledblink(int times, int lengthms, int pinnum) {
  for (int x = 0; x < times; x++) {
    digitalWrite(pinnum, HIGH);
    delay (lengthms);
    digitalWrite(pinnum, LOW);
    delay(lengthms);
  }
}


