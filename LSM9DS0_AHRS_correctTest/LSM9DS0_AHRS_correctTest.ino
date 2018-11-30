#include <nRF24L01.h>
#include <printf.h>
#include <RF24.h>
#include <RF24_config.h>

#include <SPI.h> // Included for SFE_LSM9DS0 library
#include <Wire.h>
#include <Servo.h>
#include <SoftwareSerial.h>
RF24 radio(9, 8); // CE, CSN

          //initialization for servos
Servo base;
Servo shoulder;
Servo elbow;
Servo cuff;
Servo wrist;
Servo gripper;
const byte address[6] = "00001";

const int BASE = 2;
const int SHOULDER = 3;
const int ELBOW = 4;
const int CUFF = 5;
const int WRIST = 6;
const int GRIPPER = 7;

int x = 0;
//initialization for angles on arm
int b_angle = 90;
int s_angle = 180;
int e_angle = 90;
int c_angle = 90;
int w_angle = 90;
int g_angle = 90;

struct imu_data {
  double yaw;
  double pitch;
  double roll;
  float clawDeg;
};

imu_data* imu1 = new imu_data;
imu_data* imu2 = new imu_data;
imu_data* imu3 = new imu_data;
String readString;


void update_movement(imu_data* imu)
{
  if (imu == imu1)
  {
    w_angle = 90-imu->roll;
    if(w_angle >= 180)
    {
      w_angle = 180;
    }
    else if(w_angle <= 0)
    {
      w_angle = 0;
    }
    wrist.write(w_angle);


    c_angle = 90-imu->pitch;
    if(c_angle >= 180)
    {
      c_angle = 180;
    }
    else if(c_angle <= 0)
    {
      c_angle = 0;
    }
    cuff.write(c_angle);

    Serial.print("wrist: ");
    Serial.println(w_angle);
    Serial.print("cuff: ");
    Serial.println(c_angle);
    //execute movement of arm
//    if ((imu->roll <= 180) && (imu->pitch <= 180))
//    {
//      wrist.write(imu->pitch);
//      cuff.write(180 - imu->roll);
//    }
  }
  else if (imu == imu2)
  {
//    if (imu->yaw <= 180) {
//      elbow.write(imu->pitch);
//    }
  }
  else if (imu == imu3)
  {
//    if ((imu->roll <= 180) && (imu->pitch <= 180))
//    {
//      base.write(imu->roll);
//      shoulder.write(180 - imu->pitch);
//    }
  }
  //Serial.println("wrist angle: "+w_angle);
}

void setup()
{
  Serial.begin(9600); // Start serial at 38400 bps
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();  


  //Setup for servos and initialization
  base.attach(BASE);
  base.write(b_angle);
  shoulder.attach(SHOULDER);
  shoulder.write(s_angle);
  elbow.attach(ELBOW);
  elbow.write(e_angle);
  cuff.attach(CUFF);
  cuff.write(c_angle);
  wrist.attach(WRIST);
  wrist.write(w_angle);
  gripper.attach(GRIPPER);
  gripper.write(g_angle);

}

void loop()
{
  imu_data* imu = nullptr;
  if (radio.available())
  {
    char text[32] = "";
    radio.read(&text, sizeof(text));
    //Serial.println(text);

    int whichimu;
    char *data;
    data = strtok(text, ",");
    whichimu = atoi(data);
    switch (whichimu)
    {
    case 1:
      imu = imu1;
      break;
    case 2:
      imu = imu2;
      break;
    case 3:
      imu = imu3;
      break;
    }
    if (whichimu == 1)
    {
      data = strtok(0, ",");
      imu->clawDeg = atof(data);
    }
    data = strtok(0, ",");
    imu->yaw = atof(data);
    data = strtok(0, ",");
    imu->pitch = atof(data);
    data = strtok(0, ",");
    imu->roll = atof(data);
    update_movement(imu);
  }
}

