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

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
#define GyroMeasError PI * (40.0f / 180.0f)       // gyroscope measurement error in rads/s (shown as 3 deg/s)
#define GyroMeasDrift PI * (0.0f / 180.0f)      // gyroscope measurement drift in rad/s/s (shown as 0.0 deg/s/s)
// There is a tradeoff in the beta parameter between accuracy and response speed.
// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
// However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
// Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
// I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense; 
// the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy. 
// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
#define beta sqrt(3.0f / 4.0f) * GyroMeasError   // compute beta
#define zeta sqrt(3.0f / 4.0f) * GyroMeasDrift   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f

uint32_t count = 0;  // used to control display output rate
uint32_t delt_t = 0; // used to control display output rate
float heading;
float deltat = 0.0f;        // integration interval for both filter schemes
uint32_t lastUpdate = 0;    // used to calculate integration interval
uint32_t Now = 0;           // used to calculate integration interval

struct imu_data {
  double yaw;
  double pitch;
  double roll;
  float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
  float clawDeg;
};
imu_data* imu = new imu_data;
void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, imu_data* imu)
{
            float q1 = imu->q[0], q2 = imu->q[1], q3 = imu->q[2], q4 = imu->q[3];   // short name local variable for readability
            float norm;
            float hx, hy, _2bx, _2bz;
            float s1, s2, s3, s4;
            float qDot1, qDot2, qDot3, qDot4;

            // Auxiliary variables to avoid repeated arithmetic
            float _2q1mx;
            float _2q1my;
            float _2q1mz;
            float _2q2mx;
            float _4bx;
            float _4bz;
            float _2q1 = 2.0f * q1;
            float _2q2 = 2.0f * q2;
            float _2q3 = 2.0f * q3;
            float _2q4 = 2.0f * q4;
            float _2q1q3 = 2.0f * q1 * q3;
            float _2q3q4 = 2.0f * q3 * q4;
            float q1q1 = q1 * q1;
            float q1q2 = q1 * q2;
            float q1q3 = q1 * q3;
            float q1q4 = q1 * q4;
            float q2q2 = q2 * q2;
            float q2q3 = q2 * q3;
            float q2q4 = q2 * q4;
            float q3q3 = q3 * q3;
            float q3q4 = q3 * q4;
            float q4q4 = q4 * q4;

            // Normalise accelerometer measurement
            norm = sqrt(ax * ax + ay * ay + az * az);
            if (norm == 0.0f) return; // handle NaN
            norm = 1.0f/norm;
            ax *= norm;
            ay *= norm;
            az *= norm;

            // Normalise magnetometer measurement
            norm = sqrt(mx * mx + my * my + mz * mz);
            if (norm == 0.0f) return; // handle NaN
            norm = 1.0f/norm;
            mx *= norm;
            my *= norm;
            mz *= norm;

            // Reference direction of Earth's magnetic field
            _2q1mx = 2.0f * q1 * mx;
            _2q1my = 2.0f * q1 * my;
            _2q1mz = 2.0f * q1 * mz;
            _2q2mx = 2.0f * q2 * mx;
            hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
            hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
            _2bx = sqrt(hx * hx + hy * hy);
            _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
            _4bx = 2.0f * _2bx;
            _4bz = 2.0f * _2bz;

            // Gradient decent algorithm corrective step
            s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
            norm = 1.0f/norm;
            s1 *= norm;
            s2 *= norm;
            s3 *= norm;
            s4 *= norm;

            // Compute rate of change of quaternion
            qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
            qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
            qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
            qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

            // Integrate to yield quaternion
            q1 += qDot1 * deltat;
            q2 += qDot2 * deltat;
            q3 += qDot3 * deltat;
            q4 += qDot4 * deltat;
            norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
            norm = 1.0f/norm;
            imu->q[0] = q1 * norm;
            imu->q[1] = q2 * norm;
            imu->q[2] = q3 * norm;
            imu->q[3] = q4 * norm;

}



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
   // Set data output ranges; choose lowest ranges for maximum resolution
 // Accelerometer scale can be: A_SCALE_2G, A_SCALE_4G, A_SCALE_6G, A_SCALE_8G, or A_SCALE_16G   
    imu1.setAccelScale(0.000061);
 // Gyro scale can be:  G_SCALE__245, G_SCALE__500, or G_SCALE__2000DPS
    imu1.setGyroScale(0.00875);
 // Magnetometer scale can be: M_SCALE_2GS, M_SCALE_4GS, M_SCALE_8GS, M_SCALE_12GS   
    imu1.setMagScale(0.00014);
    
  if(!imu1.begin())
    Serial.println("failed to communicate with IMU2!");
  tcaselect(4);
   // Set data output ranges; choose lowest ranges for maximum resolution
 // Accelerometer scale can be: A_SCALE_2G, A_SCALE_4G, A_SCALE_6G, A_SCALE_8G, or A_SCALE_16G   
    imu2.setAccelScale(0.000061);
 // Gyro scale can be:  G_SCALE__245, G_SCALE__500, or G_SCALE__2000DPS
    imu2.setGyroScale(0.00875);
 // Magnetometer scale can be: M_SCALE_2GS, M_SCALE_4GS, M_SCALE_8GS, M_SCALE_12GS   
    imu2.setMagScale(0.00014);
  
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

  tcaselect(2);
  imu1.readAccel();
  float ax = imu1.calcAccel(imu1.ax);
  float ay = imu1.calcAccel(imu1.ay);
  float az = imu1.calcAccel(imu1.az);
  imu1.readGyro();
  float gx = imu1.calcGyro(imu1.gx);
  float gy = imu1.calcGyro(imu1.gy);
  float gz = imu1.calcGyro(imu1.gz);
  imu1.readMag();
  float mx = imu1.calcMag(imu1.mx);
  float my = imu1.calcMag(imu1.my);
  float mz = imu1.calcMag(imu1.mz);
  
  Now = micros();
  deltat = ((Now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
  lastUpdate = Now;
  // Sensors x- and y-axes are aligned but magnetometer z-axis (+ down) is opposite to z-axis (+ up) of accelerometer and gyro!
  // This is ok by aircraft orientation standards!  
  // Pass gyro rate as rad/s
  MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, mx, my, mz, imu);
   
  // Serial print and/or display at 0.5 s rate independent of data rates
  imu->yaw   = atan2(2.0f * (imu->q[1] * imu->q[2] + imu->q[0] * imu->q[3]), imu->q[0] * imu->q[0] + imu->q[1] * imu->q[1] - imu->q[2] * imu->q[2] - imu->q[3] * imu->q[3]);   
  imu->pitch = -asin(2.0f * (imu->q[1] * imu->q[3] - imu->q[0] * imu->q[2]));
  imu->roll  = atan2(2.0f * (imu->q[0] * imu->q[1] + imu->q[2] * imu->q[3]), imu->q[0] * imu->q[0] - imu->q[1] * imu->q[1] - imu->q[2] * imu->q[2] + imu->q[3] * imu->q[3]);
  imu->pitch *= 180.0f / PI;
  imu->yaw   *= 180.0f / PI; 
  imu->yaw   -= 13.8; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
  imu->roll  *= 180.0f / PI;
  packet += "a,";
  packet += String(angle); packet += ',';
  packet += String(imu->yaw); packet += ',';
  packet += String(imu->pitch); packet += ',';
  packet += String(imu->roll);
  
  // put your main code here, to run repeatedly:
  BT.println(packet);
  delay(1);
  packet = "";
  tcaselect(4);
  
  imu2.readAccel();
  ax = imu2.calcAccel(imu2.ax);
  ay = imu2.calcAccel(imu2.ay);
  az = imu2.calcAccel(imu2.az);
  imu2.readGyro();
  gx = imu2.calcGyro(imu2.gx);
  gy = imu2.calcGyro(imu2.gy);
  gz = imu2.calcGyro(imu2.gz);
  imu2.readMag();
  mx = imu2.calcMag(imu2.mx);
  my = imu2.calcMag(imu2.my);
  mz = imu2.calcMag(imu2.mz);
  
  Now = micros();
  deltat = ((Now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
  lastUpdate = Now;
  // Sensors x- and y-axes are aligned but magnetometer z-axis (+ down) is opposite to z-axis (+ up) of accelerometer and gyro!
  // This is ok by aircraft orientation standards!  
  // Pass gyro rate as rad/s
  MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, mx, my, mz, imu);
   
  // Serial print and/or display at 0.5 s rate independent of data rates
  imu->yaw   = atan2(2.0f * (imu->q[1] * imu->q[2] + imu->q[0] * imu->q[3]), imu->q[0] * imu->q[0] + imu->q[1] * imu->q[1] - imu->q[2] * imu->q[2] - imu->q[3] * imu->q[3]);   
  imu->pitch = -asin(2.0f * (imu->q[1] * imu->q[3] - imu->q[0] * imu->q[2]));
  imu->roll  = atan2(2.0f * (imu->q[0] * imu->q[1] + imu->q[2] * imu->q[3]), imu->q[0] * imu->q[0] - imu->q[1] * imu->q[1] - imu->q[2] * imu->q[2] + imu->q[3] * imu->q[3]);
  imu->pitch *= 180.0f / PI;
  imu->yaw   *= 180.0f / PI; 
  imu->yaw   -= 13.8; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
  imu->roll  *= 180.0f / PI;

 
  packet = "b,";
  packet += String(imu->yaw); packet += ',';
  packet += String(imu->pitch); packet += ',';
  packet += String(imu->roll);
  BT.println(packet);

  Serial.println(packet);
  delay(1);

  packet = "";
}
