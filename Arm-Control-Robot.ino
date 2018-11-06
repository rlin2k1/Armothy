/**
 * TCA9548 I2CScanner.pde -- I2C bus scanner for Arduino
 *
 * Based on code c. 2009, Tod E. Kurt, http://todbot.com/blog/
 *
 */
 
#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>
extern "C" { 
#include "utility/twi.h"  // from Wire library, so we can do bus scanning
}
 
#define TCAADDR 0x70

LSM9DS1 imu1;
LSM9DS1 imu2;
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
 
 
// standard Arduino setup()
void setup()
{
    while (!Serial);
    delay(1000);
 
    Wire.begin();
    
    Serial.begin(115200);
    pinMode(FLEX_PIN, INPUT);
    Serial.println("\nTCAScanner ready!");
    
    for (uint8_t t=0; t<8; t++) {
      tcaselect(t);
      Serial.print("TCA Port #"); Serial.println(t);
 
      for (uint8_t addr = 0; addr<=127; addr++) {
        if (addr == TCAADDR) continue;
      
        uint8_t data;
        if (! twi_writeTo(addr, &data, 0, 1, 1)) {
           Serial.print("Found I2C 0x");  Serial.println(addr,HEX);
        }
      }
    }
    Serial.println("\ndone");

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
    
}
 
void loop() 
{
  int flexADC = analogRead(FLEX_PIN);
  float flexV = flexADC * VCC / 1023.0;
  float flexR = R_DIV * (VCC / flexV - 1.0);
  Serial.println("Resistance: " + String(flexR) + " ohms");

  float angle = map(flexR, STRAIGHT_RESIST, ANGLE_RESIST,
                   0, 90.0);
  Serial.println("Bend: " + String(angle) + " degrees");
  Serial.println();


  
  tcaselect(2);
  imu1.readAccel();
  imu1.readGyro();
 
  /* Display the results */
  Serial.print("Sensor #1 - ");
  Serial.print("X: "); Serial.print(imu1.calcAccel(imu1.ax)); Serial.print(" "); Serial.println(imu1.calcGyro(imu1.gx));
  Serial.print("Y: "); Serial.print(imu1.calcAccel(imu1.ay)); Serial.print("  "); Serial.println(imu1.calcGyro(imu1.gy));
  Serial.print("Z: "); Serial.print(imu1.calcAccel(imu1.az)); Serial.print("  "); Serial.println(imu1.calcGyro(imu1.gz));

  
  tcaselect(4);
  imu2.readAccel();
  imu2.readGyro();
 
  /* Display the results */
  Serial.print("Sensor #2 - ");
  Serial.print("X: "); Serial.print(imu2.calcAccel(imu2.ax)); Serial.print(" "); Serial.println(imu2.calcGyro(imu2.gx));
  Serial.print("Y: "); Serial.print(imu2.calcAccel(imu2.ay)); Serial.print("  "); Serial.println(imu2.calcGyro(imu2.gy));
  Serial.print("Z: "); Serial.print(imu2.calcAccel(imu2.az)); Serial.print("  "); Serial.println(imu2.calcGyro(imu2.gz));
  delay(500);
}