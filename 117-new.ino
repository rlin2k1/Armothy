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
int s_angle = 120;
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

int r = 0; 
int p = 0;
void update_movement(imu_data* imu)
{
  if(imu == imu2)// && ((p > 20 && p < 55) && (r < -65 && r > -85)))
  {
    //Serial.print("ELBOW: ");
    int roll = imu2->roll;
    if(imu2->roll < 0)
    {
      roll = 90;
    }
    else if(imu2->roll > 70)
    {
      roll = 180;
    }
    else 
    {
      roll = map(imu2->roll,0,70,90,180);
    }
    elbow.write(roll);
//    if(roll == 180);
//    {
//      shoulder.write(50);
//      base.write(80);
//      wrist.write(90);
//    }
    //Serial.println(roll);
  }
  else if(imu == imu3)
  {
    r = imu3->roll;
    p = imu3->pitch;
    //Angle ranges from 120 to 50 (0, 90)
    int pitch = imu3->pitch;
    if(pitch>= 0 && pitch <= 80)
    {
      pitch = map(pitch, 0, 80, 120 ,50);
    }
    else if (pitch < 0)
    {
      pitch = 120;
    }
    else if(pitch > 80)
    {
      pitch = 50; 
    }
    s_angle = pitch;
    shoulder.write(pitch);
    //Serial.print("Shoulder: ");
    //Serial.println(pitch);
    //Angle ranges from 10 - 150 with 80 being the midpoint; (0, 180);
    int roll = imu3->roll;
    if(roll >= -10 && roll <= 0) //relax state
    {
      roll = 80; 
    }
    else if(roll < -10 && roll > -100) //forward rotation
    {
      roll = map(roll, -11, -99, 80, 150);
    }
    else if(roll <= -100) //high point
    {
      roll= 150;
    }
    else if(roll > 0 && roll < 70)
    {
      roll = map(roll, 0, 70, 80, 10);
    }
    else if(roll >= 70)
    {
      roll = 10;
    }
    base.write(roll);
    //Serial.print("Base: ");
    //Serial.println(roll);
  }
  else if(imu == imu1)
  {
    if(imu1->clawDeg >= 1100 && imu1->clawDeg <= 2900)
    {
      g_angle = map(imu1->clawDeg, 1100,2900, 90, 0);
      gripper.write(g_angle);
    }
    //Tristan's wrist up @ -80; wrist down @ 90; neutral @ 0;
    int roll = imu1->roll;
    Serial.print("Wrist :");
    if(imu1->roll >=-5 && imu1->roll <=5)
    {
      roll = 90;
    }
    else if(imu1->roll < -5 && imu1->roll > -80)
    {
      roll = map(imu1->roll, -5, -80, 90, 180);
    }
    else if(imu1->roll <= -80)
    {
      roll = 180;
    }
    else if(imu1->roll > 5 && imu1->roll < 90)
    {
      roll = map(imu1->roll, 5, 90, 90, 0);
    }
    else if(imu1->roll > 90)
    {
      roll = 0;
    }

    if(s_angle > 55)
    {
      wrist.write(roll+s_angle-35);
      //Serial.println(roll+s_angle-65);
    }
    else
    {
      wrist.write(roll);
      //Serial.println(roll);
    }
    
    Serial.println(imu1->clawDeg);
  }
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
      //Serial.println(data);
    }
    data = strtok(0, ",");
    imu->yaw = atof(data);
    data = strtok(0, ",");
    imu->pitch = atof(data);
    data = strtok(0, ",");
    imu->roll = atof(data);
    update_movement(imu);
//    Serial.print("P :");
//    Serial.println(p);
//    Serial.print("R :");
//    Serial.println(r);
  }
}
