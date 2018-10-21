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

// ###### BOBBYCAR ######
// #define FILTER              0.1
// #define SPEED_COEFFICIENT   -1
// #define STEER_COEFFICIENT   0

// ###### ARMCHAIR ######
// #define FILTER              0.05
// #define SPEED_COEFFICIENT   0.5
// #define STEER_COEFFICIENT   -0.2

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

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
  Serial.print("Nunchuck init");
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

void steering(float xTmp, float yTmp) {
  /* Serial.print("  +x ");
    Serial.print(x);
    Serial.print(" +y ");
    Serial.print(y);
    Serial.print(" ");
  */

  float x = xTmp;
  float y = yTmp * mapFloat(abs(x), 0, 1, 1, 0.2 );

  // convert to polar
  float r = sqrt(x * x + y * y);
  float t = atan2(y, x);

  // rotate by 45 degrees
  t += PI / 4;

  // back to cartesian
  float left = r * cos(t);
  float right = r * sin(t);

  // rescale the new coords
  left = left * sqrt(2);
  right = right * sqrt(2);

  // clamp to -1/+1
  left = max(-1, min(left, 1));
  right = max(-1, min(right, 1));

  metrics.leftMotor = mapFloat(left, -1, 1, -currMax, currMax);
  metrics.rightMotor = mapFloat(right, -1, 1, -currMax, currMax);
}

void loop() {
  delay(20);
  chuck.update();

  steering(
    mapFloat(
      constrain(chuck.readJoyY(), -100, 100),
      -100, 100, -1, 1),
    mapFloat(
      constrain(chuck.readJoyX(), -100, 100),
      -100, 100, -1, 1)
  );

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



