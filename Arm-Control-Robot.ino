#include <SoftwareSerial.h>
#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>
extern "C" { 
#include "utility/twi.h"  // from Wire library, so we can do bus scanning
}

#define TCAADDR 0x70

SoftwareSerial BT(10,11);

LSM9DS1 imu1;
LSM9DS1 imu2;

String packet;

const int FLEX_PIN = 0;
const float VCC = 5.02;
const float R_DIV = 47000.0;
const float STRAIGHT_RESIST = 37300.0;
const float ANGLE_RESIST = 90000.0;

void tcaselect(uint8_t i) {
  if (i > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}

void setup() {
  Wire.begin();
  BT.begin(38400);

  imu1.settings.device.commInterface = IMU_MODE_I2C; // Set mode to I2C 
  imu1.settings.device.mAddress = 0x1E; // Set mag address to 0x1E
  imu1.settings.device.agAddress = 0x6B; // Set ag address to 0x6B

  imu2.settings.device.commInterface = IMU_MODE_I2C; // Set mode to I2C 
  imu2.settings.device.mAddress = 0x1E; // Set mag address to 0x1E
  imu2.settings.device.agAddress = 0x6B; // Set ag address to 0x6B
  tcaselect(2);
  if(!imu1.begin())
    Serial.println("failed to communicate with IMU2!");
  tcaselect(4);
  if(!imu2.begin())
    Serial.println("Failed to communicate with IMU4!");
  packet = "";
}

void loop() 
{
  int flexADC = analogRead(FLEX_PIN);
  float flexV = flexADC * VCC / 1023.0;
  float flexR = R_DIV * (VCC / flexV - 1.0);

  float angle = map(flexR, STRAIGHT_RESIST, ANGLE_RESIST,
                   0, 90.0);
  packet += "a,";
  packet += String(angle); packet += ',';
  tcaselect(2);
  imu1.readAccel();
  imu1.readGyro();
  imu1.readMag();
  packet += String(imu1.calcAccel(imu1.ax)); packet += ',';
  packet += String(imu1.calcAccel(imu1.ay)); packet += ',';
  packet += String(imu1.calcAccel(imu1.az)); packet += ',';
  packet += String(imu1.calcGyro(imu1.gx)); packet += ',';
  packet += String(imu1.calcGyro(imu1.gy)); packet += ',';
  packet += String(imu1.calcGyro(imu1.gz)); packet += ',';
  packet += String(imu1.calcMag(imu1.mx)); packet += ',';
  packet += String(imu1.calcMag(imu1.my)); packet += ',';
  packet += String(imu1.calcMag(imu1.mz));
  
  // put your main code here, to run repeatedly:
  BT.println(packet);
  packet = "b,";
  tcaselect(4);
  imu2.readAccel();
  imu2.readGyro();
  imu2.readMag();
  packet += String(imu2.calcAccel(imu2.ax)); packet += ',';
  packet += String(imu2.calcAccel(imu2.ay)); packet += ',';
  packet += String(imu2.calcAccel(imu2.az)); packet += ',';
  packet += String(imu2.calcGyro(imu2.gx)); packet += ',';
  packet += String(imu2.calcGyro(imu2.gy)); packet += ',';
  packet += String(imu2.calcGyro(imu2.gz)); packet += ',';
  packet += String(imu2.calcMag(imu2.mx)); packet += ',';
  packet += String(imu2.calcMag(imu2.my)); packet += ',';
  packet += String(imu2.calcMag(imu2.mz));
  BT.println(packet);
  packet = "";
  delay(100);
}
