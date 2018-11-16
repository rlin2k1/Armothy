#include <SPI.h> // Included for SFE_LSM9DS0 library
#include <Wire.h>
#include <Servo.h>
#include <SoftwareSerial.h>

//initialization for servos
Servo base;
Servo shoulder;
Servo elbow;
Servo cuff;
Servo wrist;
Servo gripper;

SoftwareSerial BT(10, 11);

const int BASE = 2;
const int SHOULDER = 3;
const int ELBOW = 4;
const int CUFF = 5;
const int WRIST = 6;
const int GRIPPER = 7;
const int def_angle = 90;
const int B_CAM = 90;
const int S_CAM = 90;
const int E_CAM = 90;
const int C_CAM = 90;
const int W_CAM = 90;

int x = 0;
//initialization for angles on arm
int b_angle = 1500;
int s_angle = 1500;
int e_angle = 1500;
int c_angle = 1500;
int w_angle = 1500;
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
    //Serial.println(imu->clawDeg);
    if(imu == imu1)
    {
      //Serial.println(imu->clawDeg);
      gripper.write(180 - imu->clawDeg);
      

      //execute movement of arm
      if((imu->roll <= 180) && (imu->pitch <= 180))
      {
        //JASON SHOULD KNOW WHAT SERVOS CORRESPOND TO WHAT PINS
        //ERICK SHOULD KNOW WHAT SERVOS DO WHAT

        wrist.write( imu->roll);
        cuff.write(180 - imu->pitch);
        Serial.print(imu->roll); Serial.print("\t"); Serial.println(imu->pitch); 
      }
    }
    else if(imu == imu2)
    {
      //if((imu->roll <= 180) && (imu->pitch <= 180))
      //{
       // base.write(imu->pitch);
       // shoulder.write(180 - imu->roll);
      //}
    }
    else if(imu == imu3)
    {
      
    }
}

void setup()
{
  Serial.begin(9600); // Start serial at 38400 bps
  BT.begin(38400);
  //Setup for servos and initialization
  base.attach(BASE);
  base.write(def_angle);
  shoulder.attach(SHOULDER);
  shoulder.write(S_CAM);
  elbow.attach(ELBOW);
  elbow.write(E_CAM);
  cuff.attach(CUFF);
  cuff.write(C_CAM);
  wrist.attach(WRIST);
  wrist.write(W_CAM);
  gripper.attach(GRIPPER);
  gripper.write(90);

}

void loop()
{
  imu_data* imu = nullptr;
  if(BT.available())
  { 
    char c = BT.read();  //gets one byte from serial buffer
    if (c == '\n') 
    {
      if (readString.length() >1) 
      {
        readString += c;
        //Serial.print(readString);
        char* cdata = readString.c_str();
        char* data = strtok(cdata, ",");
        int whichimu = *data - 97;
        switch(whichimu)
        {
          case 0:
            imu = imu1;
            break;
          case 1:
            imu = imu2;
            break;
          case 2:
            imu = imu3;
            break;
          default:
            readString = "";
            break;
        }
        if(whichimu == 0)
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
        readString="";
        update_movement(imu);
      }
    }  
     else 
     {     
      readString += c; //makes the string readString
     }  
  }
}
