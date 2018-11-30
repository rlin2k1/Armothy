#include <SPI.h> // Included for SFE_LSM9DS0 library
#include <Wire.h>

#include <SoftwareSerial.h>
#include "SparkFunLSM9DS1.h"
#include "SFE_LSM9DS0.h"
#include <nRF24L01.h>
#include <RF24.h>

//#include "Arduino.h"
extern "C" { 
#include "utility/twi.h"  // from Wire library, so we can do bus scanning
}

//SoftwareSerial BT(0,1);
#define LSM9DS0_XM  0x1D // Would be 0x1E if SDO_XM is LOW
#define LSM9DS0_G   0x6B // Would be 0x6A if SDO_G is LOW
#define LSM9DS1_M   0x1E
#define LSM9DS1_AG  0x6B

#define TCAADDR 0x70

RF24 radio(9, 8); //CE, CSN

// [SPI or I2C Mode declaration],[gyro I2C address],[xm I2C add.]
LSM9DS0 imu3(MODE_I2C, LSM9DS0_G, LSM9DS0_XM);
LSM9DS1 imu2(1, LSM9DS1_AG, LSM9DS1_M);
LSM9DS1 imu1(1, LSM9DS1_AG, LSM9DS1_M);
//////////////////////
//address for RF 
/////////////////////
const byte address[6] = "00001";

///////////////////////////////
// Interrupt Pin Definitions //
///////////////////////////////
const byte INT1XM = 5; // INT1XM tells us when accel data is ready
const byte INT2XM = 4; // INT2XM tells us when mag data is ready
const byte DRDYG  = 6; // DRDYG  tells us when gyro data is ready

// global constants for 9 imu1 fusion and AHRS (Attitude and Heading Reference System)
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


float pitch, yaw, roll, heading;
float deltat1 = 0.0f;        // integration interval for both filter schemes
float deltat2 = 0.0f;
float deltat3 = 0.0f;
uint32_t lastUpdate1 = 0;    // used to calculate integration interval
uint32_t lastUpdate2 = 0;
uint32_t lastUpdate3 = 0;
uint32_t Now1 = 0;           // used to calculate integration interval
uint32_t Now2 = 0;
uint32_t Now3 = 0;


String packet;

const int FLEX_PIN = 0;
const float VCC = 5.02;
const float R_DIV = 47000.0;
const float STRAIGHT_RESIST = 37300.0;
const float ANGLE_RESIST = 90000.0;

float abias[3] = {0, 0, 0}, gbias[3] = {0, 0, 0};
float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values 

struct imu_data {
  double yaw;
  double pitch;
  double roll;
  float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
  float clawDeg;
};
imu_data* imu_d1 = new imu_data;
imu_data* imu_d2 = new imu_data;
imu_data* imu_d3 = new imu_data;

void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat);
void tcaselect(uint8_t i);
void extract_and_transmit_imu1(); 
void extract_and_transmit_imu2(); 
void extract_and_transmit_imu3(); 



void setup()
{
  Wire.begin();
  Serial.begin(38400); // Start serial at 38400 bps
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();
  
  //////////////////////////////////////////
  //SELECT DATA FROM IMU1   pin 7
  /////////////////////////////////////////
  tcaselect(1); 
  
  if(!imu1.begin())
    Serial.println("failed to communicate with IMU1!");
    
  imu1.setAccelScale(imu1.A_SCALE_2G);
  imu1.setGyroScale(imu1.G_SCALE_245DPS);
  imu1.setMagScale(imu1.M_SCALE_4GS);



  //////////////////////////////////////////
  //SELECT DATA FROM IMU2   pin 6
  /////////////////////////////////////////
  tcaselect(6);
  
  if(!imu2.begin())
    Serial.println("failed to communicate with IMU2!");
    
  imu2.setAccelScale(imu2.A_SCALE_2G);
  imu2.setGyroScale(imu2.G_SCALE_245DPS);
  imu2.setMagScale(imu2.M_SCALE_4GS);
  

  /////////////////////////////////////////
  //SELECT DATA FROM IMU3  pin 1
  ////////////////////////////////////////
  tcaselect(7);
  
  if(!imu3.begin())
    Serial.println("failed to communicate with IMU2!");
    
  imu3.setAccelScale(imu3.A_SCALE_2G);
  imu3.setGyroScale(imu3.G_SCALE_245DPS);
  imu3.setMagScale(imu3.M_SCALE_2GS);
  
  packet = "";
  

}

void loop()
{

  char senddata[32];
  
  int flexADC = analogRead(FLEX_PIN);
  float flexV = flexADC * VCC / 1023.0;
  float flexR = R_DIV * (VCC / flexV - 1.0);
  
  float angle = map(flexR, STRAIGHT_RESIST, ANGLE_RESIST,
                 0, 90.0);
  //////////////////////////////////////////////////////////
  //////IMU1 DATA INPUT
  //////////////////////////////////////////////////////////
  tcaselect(1);
  imu1.readGyro();           // Read raw gyro data
  gx = imu1.calcGyro(imu1.gx) - gbias[0];   // Convert to degrees per seconds, remove gyro biases
  gy = imu1.calcGyro(imu1.gy) - gbias[1];
  gz = imu1.calcGyro(imu1.gz) - gbias[2];
  
  imu1.readAccel();         // Read raw accelerometer data
  ax = imu1.calcAccel(imu1.ax) - abias[0];   // Convert to g's, remove accelerometer biases
  ay = imu1.calcAccel(imu1.ay) - abias[1];
  az = imu1.calcAccel(imu1.az) - abias[2];
  
  imu1.readMag();           // Read raw magnetometer data
  mx = imu1.calcMag(imu1.mx);     // Convert to Gauss and correct for calibration
  my = imu1.calcMag(imu1.my);
  mz = imu1.calcMag(imu1.mz);

  
  Now1 = micros();
  deltat1 = ((Now1 - lastUpdate1)/1000000.0f); // set integration time by time elapsed since last filter update
  lastUpdate1 = Now1;
  MadgwickQuaternionUpdate(-ax, ay, az, -gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, mx, my, -mz, deltat1, imu_d1);
  
  yaw   = atan2(2.0f * (imu_d1->q[1] * imu_d1->q[2] + imu_d1->q[0] * imu_d1->q[3]), imu_d1->q[0] * imu_d1->q[0] + imu_d1->q[1] * imu_d1->q[1] - imu_d1->q[2] * imu_d1->q[2] - imu_d1->q[3] * imu_d1->q[3]);   
  pitch = -asin(2.0f * (imu_d1->q[1] * imu_d1->q[3] - imu_d1->q[0] * imu_d1->q[2]));
  roll  = atan2(2.0f * (imu_d1->q[0] * imu_d1->q[1] + imu_d1->q[2] * imu_d1->q[3]), imu_d1->q[0] * imu_d1->q[0] - imu_d1->q[1] * imu_d1->q[1] - imu_d1->q[2] * imu_d1->q[2] + imu_d1->q[3] * imu_d1->q[3]);
  pitch *= 180.0f / PI;
  yaw   *= 180.0f / PI; 
  yaw   -= 13.8; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
  roll  *= 180.0f / PI;
  imu_d1->yaw = yaw;
  imu_d1->pitch = pitch;
  imu_d1->roll = roll;
  
  
  
  packet += "1"; packet += ",";
 // packet += String(angle); packet += ",";
  packet += String(imu_d1->yaw); packet += ",";
  packet += String(imu_d1->pitch); packet += ",";
  packet += String(imu_d1->roll);   
  Serial.println(packet);
  packet.toCharArray(senddata, 32);

  radio.write(senddata, sizeof(senddata));

  delay(1);
  packet = ""; 



  //////////////////////////////////////////////////////////
  //////IMU2 DATA INPUT
  //////////////////////////////////////////////////////////
  tcaselect(6);
  imu2.readGyro();           // Read raw gyro data
  gx = imu2.calcGyro(imu2.gx) - gbias[0];   // Convert to degrees per seconds, remove gyro biases
  gy = imu2.calcGyro(imu2.gy) - gbias[1];
  gz = imu2.calcGyro(imu2.gz) - gbias[2];
    
  imu2.readAccel();         // Read raw accelerometer data
  ax = imu2.calcAccel(imu2.ax) - abias[0];   // Convert to g's, remove accelerometer biases
  ay = imu2.calcAccel(imu2.ay) - abias[1];
  az = imu2.calcAccel(imu2.az) - abias[2];
  
  imu2.readMag();           // Read raw magnetometer data
  mx = imu2.calcMag(imu2.mx);     // Convert to Gauss and correct for calibration
  my = imu2.calcMag(imu2.my);
  mz = imu2.calcMag(imu2.mz);

  
  Now2 = micros();
  deltat2 = ((Now2 - lastUpdate2)/1000000.0f); // set integration time by time elapsed since last filter update
  lastUpdate2 = Now2;
  MadgwickQuaternionUpdate(-ax, ay, az, -gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, mx, my, -mz, deltat2, imu_d2);

  yaw   = atan2(2.0f * (imu_d2->q[1] * imu_d2->q[2] + imu_d2->q[0] * imu_d2->q[3]), imu_d2->q[0] * imu_d2->q[0] + imu_d2->q[1] * imu_d2->q[1] - imu_d2->q[2] * imu_d2->q[2] - imu_d2->q[3] * imu_d2->q[3]);   
  pitch = -asin(2.0f * (imu_d2->q[1] * imu_d2->q[3] - imu_d2->q[0] * imu_d2->q[2]));
  roll  = atan2(2.0f * (imu_d2->q[0] * imu_d2->q[1] + imu_d2->q[2] * imu_d2->q[3]), imu_d2->q[0] * imu_d2->q[0] - imu_d2->q[1] * imu_d2->q[1] - imu_d2->q[2] * imu_d2->q[2] + imu_d2->q[3] * imu_d2->q[3]);
  pitch *= 180.0f / PI;
  yaw   *= 180.0f / PI; 
  yaw   -= 13.8; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
  roll  *= 180.0f / PI;
  imu_d2->yaw = yaw;
  imu_d2->pitch = pitch;
  imu_d2->roll = roll;
  

  packet += "2"; packet += ",";
  packet += String(imu_d2->yaw); packet += ",";
  packet += String(imu_d2->pitch); packet += ",";
  packet += String(imu_d2->roll);   
  packet.toCharArray(senddata, 32);

    // put your main code here, to run repeatedly:
  Serial.println(packet);
  radio.write(senddata, sizeof(senddata));
  delay(1);
  packet = ""; 
  


  //////////////////////////////////////////////////////////
  //////IMU3 DATA INPUT
  //////////////////////////////////////////////////////////
  tcaselect(7);
  imu3.readGyro();           // Read raw gyro data
  gx = imu3.calcGyro(imu3.gx) - gbias[0];   // Convert to degrees per seconds, remove gyro biases
  gy = imu3.calcGyro(imu3.gy) - gbias[1];
  gz = imu3.calcGyro(imu3.gz) - gbias[2];
    
  imu3.readAccel();         // Read raw accelerometer data
  ax = imu3.calcAccel(imu3.ax) - abias[0];   // Convert to g's, remove accelerometer biases
  ay = imu3.calcAccel(imu3.ay) - abias[1];
  az = imu3.calcAccel(imu3.az) - abias[2];
  
  imu3.readMag();           // Read raw magnetometer data
  mx = imu3.calcMag(imu3.mx);     // Convert to Gauss and correct for calibration
  my = imu3.calcMag(imu3.my);
  mz = imu3.calcMag(imu3.mz);
    
  
  Now3 = micros();
  deltat3 = ((Now3 - lastUpdate3)/1000000.0f); // set integration time by time elapsed since last filter update
  lastUpdate3 = Now3;
  MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, mx, my, mz, deltat3, imu_d3);

  yaw   = atan2(2.0f * (imu_d3->q[1] * imu_d3->q[2] + imu_d3->q[0] * imu_d3->q[3]), imu_d3->q[0] * imu_d3->q[0] + imu_d3->q[1] * imu_d3->q[1] - imu_d3->q[2] * imu_d3->q[2] - imu_d3->q[3] * imu_d3->q[3]);   
  pitch = -asin(2.0f * (imu_d3->q[1] * imu_d3->q[3] - imu_d3->q[0] * imu_d3->q[2]));
  roll  = atan2(2.0f * (imu_d3->q[0] * imu_d3->q[1] + imu_d3->q[2] * imu_d3->q[3]), imu_d3->q[0] * imu_d3->q[0] - imu_d3->q[1] * imu_d3->q[1] - imu_d3->q[2] * imu_d3->q[2] + imu_d3->q[3] * imu_d3->q[3]);
  pitch *= 180.0f / PI;
  yaw   *= 180.0f / PI; 
  yaw   -= 13.8; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
  roll  *= 180.0f / PI;
  imu_d3->yaw = yaw;
  imu_d3->pitch = pitch;
  imu_d3->roll = roll;

  packet += "3"; packet += ",";
  packet += String(imu_d3->yaw); packet += ",";
  packet += String(imu_d3->pitch); packet += ",";
  packet += String(imu_d3->roll);  
  packet.toCharArray(senddata, 32);

  Serial.println(packet);
  radio.write(senddata, sizeof(senddata)); 
  delay(1);
  packet = "";
  

}

void tcaselect(uint8_t i) {
  if (i > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}

// Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
// (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
// which fuses acceleration, rotation rate, and magnetic moments to produce a quaternion-based estimate of absolute
// device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
// The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms
// but is much less computationally intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz!
  
        void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat, imu_data* imu)
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
