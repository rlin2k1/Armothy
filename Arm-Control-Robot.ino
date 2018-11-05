#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>
//#include <SoftwareSerial.h>
 
#define TCAADDR 0x70
#define LSM9DS1_M   0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B // Would be 0x6A if SDO_AG is LOW

//SoftwareSerial BT(10, 11); // May have to configure these values (should be TX, RX)
LSM9DS1 imu1;
LSM9DS1 imu2;
LSM9DS1 imu3;
 
void tcaselect(uint8_t i) {
  if (i > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}
 
 
void setup(void) 
{
  Serial.begin(9600);  
  Wire.begin();
  //BT.begin(9600);
  //Use these to configure settings on each imu
  //imu.settings.accel.scale = 16; // Set accel range to +/-16g
  //imu.settings.gyro.scale = 2000; // Set gyro range to +/-2000dps
  //imu.settings.mag.scale = 8; // Set mag range to +/-8Gs
  /* Initialise the 1st sensor */
  tcaselect(2);
  imu1.settings.device.commInterface = IMU_MODE_I2C;
  imu1.settings.device.mAddress = LSM9DS1_M;
  imu1.settings.device.agAddress = LSM9DS1_AG;
  if(!imu1.begin())
  {
    Serial.println("Failed to communicate with IMU 1.");
    while(true);
  }
  /* Initialise the 2nd sensor */
  /*tcaselect(5);
  imu2.settings.device.commInterface = IMU_MODE_I2C;
  imu2.settings.device.mAddress = LSM9DS1_M;
  imu2.settings.device.agAddress = LSM9DS1_AG;
  if(!imu2.begin())
  {
    Serial.println("Failed to communicate with IMU 2.");
    while(true);
  }*/
  /* Initialise the 3rd sensor */
  /*tcaselect(6);
  imu3.settings.device.commInterface = IMU_MODE_I2C;
  imu3.settings.device.mAddress = LSM9DS1_M;
  imu3.settings.device.agAddress = LSM9DS1_AG;
  if(!imu3.begin())
  {
    Serial.println("Failed to communicate with IMU 3.");
    while(true);
  }*/
}
 
void loop(void) 
{
  //Use BT.println(Data_to_be_sent); to transmit data.
  //Use if(BT.available()) to check if data has arrived in the bluetooth serial
  //Use data_type (might have to be char) d = BT.read();
  tcaselect(2);
  imu1.readAccel();
  imu1.readGyro();
 
  /* Display the results */
  Serial.print("Sensor #1 - ");
  Serial.print("X: "); Serial.print(imu1.calcAccel(imu1.ax)); Serial.print(" "); Serial.println(imu1.calcGyro(imu1.gx));
  Serial.print("Y: "); Serial.print(imu1.calcAccel(imu1.ay)); Serial.print("  "); Serial.println(imu1.calcGyro(imu1.gy));
  Serial.print("Z: "); Serial.print(imu1.calcAccel(imu1.az)); Serial.print("  "); Serial.println(imu1.calcGyro(imu1.gz));
  
  /*tcaselect(5);
  imu1.readAccel();
  imu1.readGyro();
 
  Serial.print("Sensor #2 - ");
  Serial.print("X: "); Serial.print(imu2.calcAccel(imu2.ax)); Serial.print(" "); Serial.println(imu2.calcGyro(imu2.gx));
  Serial.print("Y: "); Serial.print(imu2.calcAccel(imu2.ay)); Serial.print("  "); Serial.println(imu2.calcGyro(imu2.gy));
  Serial.print("Z: "); Serial.print(imu2.calcAccel(imu2.az)); Serial.print("  "); Serial.println(imu2.calcGyro(imu2.gz));

  tcaselect(6);
  imu3.readAccel();
  imu3.readGyro();
 
  Serial.print("Sensor #3 - ");
  Serial.print("X: "); Serial.print(imu3.calcAccel(imu3.ax)); Serial.print(" "); Serial.println(imu3.calcGyro(imu3.gx));
  Serial.print("Y: "); Serial.print(imu3.calcAccel(imu3.ay)); Serial.print("  "); Serial.println(imu3.calcGyro(imu3.gy));
  Serial.print("Z: "); Serial.print(imu3.calcAccel(imu3.az)); Serial.print("  "); Serial.println(imu3.calcGyro(imu3.gz));
  */
  delay(500);
}
