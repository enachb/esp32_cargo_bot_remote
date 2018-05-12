#include <radio_config_Si4460.h>
#include <RadioHead.h>
#include <RH_ASK.h>
#include <RH_NRF24.h>
#include <RH_NRF51.h>
#include <RH_NRF905.h>
#include <RH_RF22.h>
#include <RH_RF24.h>
#include <RH_RF69.h>
#include <RH_RF95.h>
#include <RH_Serial.h>
#include <RH_TCP.h>
#include <RHCRC.h>
#include <RHDatagram.h>
#include <RHGenericDriver.h>
#include <RHGenericSPI.h>
#include <RHHardwareSPI.h>
#include <RHMesh.h>
#include <RHNRFSPIDriver.h>
#include <RHReliableDatagram.h>
#include <RHRouter.h>
#include <RHSoftwareSPI.h>
#include <RHSPIDriver.h>
#include <RHTcpProtocol.h>

#include <Math.h>
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
RH_NRF24 nrf24;
int8_t motors[2] = {0, 0};

int16_t speedL;
int16_t speedR;

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

  if (!nrf24.init())
    Serial.println("nrf init failed");
  else
    Serial.println("nrf init success!");
  // Defaults after init are 2.402 GHz (channel 2), 2Mbps, 0dBm
  if (!nrf24.setChannel(1))
    Serial.println("setChannel failed");
  else
    Serial.println("setChannel ok");
  if (!nrf24.setRF(RH_NRF24::DataRate2Mbps, RH_NRF24::TransmitPower0dBm))
    Serial.println("setRF failed");


}

// not working yet!!!!
void hoverboard_mix() {
  // compute nunchuck to motor values
  int cmd1 = constrain((chuck.readJoyY() - 127) * 8, -1000, 1000); // x - axis. Nunchuck joystick readings range 30 - 230
  int cmd2 = constrain((chuck.readJoyX() - 128) * 8, -1000, 1000); // y - axis

  // ####### LOW-PASS FILTER #######
  int steer = steer * (1.0 - FILTER) + cmd1 * FILTER;
  int speed = speed * (1.0 - FILTER) + cmd2 * FILTER;

  // motor mixer
  int16_t speedR = constrain(speed * SPEED_COEFFICIENT -  steer * STEER_COEFFICIENT, -1000, 1000);
  int16_t speedL = constrain(speed * SPEED_COEFFICIENT +  steer * STEER_COEFFICIENT, -1000, 1000);
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

  speedL = mapFloat(left, -1, 1, -1000, 1000);
  speedR = mapFloat(right, -1, 1, -1000, 1000);
}

void loop() {
  delay(20);
  delay(500);
  chuck.update();

  steering(
    mapFloat(chuck.readJoyY(), -100, 100, -1, 1),
    mapFloat(chuck.readJoyX(), -100, 100, -1, 1)
  );

  //  Serial.print(chuck.readJoyX());
  //  Serial.print(", ");
  //  Serial.print(chuck.readJoyY());
  Serial.print(", ");
  Serial.print(speedL);
  Serial.print(", ");
  Serial.print(speedR);
  Serial.println("");

  motors[0] = speedL;
  motors[1] = speedR;

  // Should be a message for us now
  uint8_t buf[sizeof(motors)];
  memcpy(motors, buf, sizeof(motors));

  Serial.println("....trying to send");
  nrf24.send(buf, sizeof(buf));
  nrf24.waitPacketSent();
  Serial.println("Sent a reply");

}

