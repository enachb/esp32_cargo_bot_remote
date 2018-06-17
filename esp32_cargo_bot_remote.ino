#include <nRF24L01.h>
#include <printf.h>
#include <RF24.h>
#include <RF24_config.h>

#include <math.h>
#include "Wire.h"
//#include "WiiChuckClass.h" //most likely its WiiChuck.h for the rest of us.
#include "WiiChuck.h"

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

//#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

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

}

void steering(float x, float y) {
  /* Serial.print("  +x ");
    Serial.print(x);
    Serial.print(" +y ");
    Serial.print(y);
    Serial.print(" ");
  */

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

  metrics.leftMotor = mapFloat(left, -1, 1, -1000, 1000);
  metrics.rightMotor = mapFloat(right, -1, 1, -1000, 1000);
}

void loop() {
  delay(20);
  chuck.update();

  steering(
    mapFloat(chuck.readJoyY(), -100, 100, -1, 1),
    mapFloat(chuck.readJoyX(), 100, -100, -1, 1)
  );

  if (count % 10 == 0) {
    Serial.print(chuck.readJoyX());
    Serial.print(", ");
    Serial.print(chuck.readJoyY());
    Serial.print(", ");
    Serial.print(metrics.leftMotor);
    Serial.print(", ");
    Serial.print(metrics.rightMotor);
    Serial.println("");
    //  Serial.print(" sizeof: ");
    //  Serial.print(sizeof(motors));
    //  Serial.printLN(" ");
    count = 1;
  } else {
    count++;
  }

  // Should be a message for us now
  //  memcpy(&motors, &buf, sizeof(motors));

  // send it
  if (!radio.write(&metrics, sizeof(metrics) )) {
    Serial.println(F("failed."));
  }

}

