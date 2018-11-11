#include <SoftwareSerial.h>

String readString;
SoftwareSerial BT(10,11);
void setup() {
  Serial.begin(9600);
  BT.begin(38400);
  Serial.println("serial delimit test 1.0"); // so I can keep track of what is loaded
}

void loop() {

  //expect a string like wer,qwe rty,123 456,hyre kjhg,
  //or like hello world,who are you?,bye!,

  if (BT.available())  {
    char c = BT.read();  //gets one byte from serial buffer
    if (c == '\n') {
      if (readString.length() >1) {
        readString += c;
        Serial.print(readString); //prints string to serial port out
        //Do stuff
        readString="";
        //clears variable for new input
      }
    }  
    else {     
      readString += c; //makes the string readString
    }
  }
}

