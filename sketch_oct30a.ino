#include <Servo.h>
String inputString = "";         // a String to hold incoming data
bool receivedAngles = false;  // whether the string is complete

bool DEBUGGING = false;

int servoOnePin = 2;
int servoTwoPin = 3;
int servoThreePin = 4;


Servo Servo1;
Servo Servo2;
Servo Servo3;

void setup() {
  // init servos
  Servo1.attach(servoOnePin);
  Servo2.attach(servoTwoPin);
  Servo3.attach(servoThreePin);
  // initialize serial:
  Serial.begin(9600);
  // reserve 200 bytes for the inputString:
  inputString.reserve(200);
}

void loop() {
  // parse angle data when input arrives 
  if (receivedAngles) {
    // parse the inputString....
    int i = inputString.indexOf(';');
    if(i==-1){
      
      //open/close grip
      String s_grip = inputString;
      int grip = s_grip.toInt();
      if(grip == 1){
        setGrip(true);
      }else{
        setGrip(false);
      }
    }else{
      
      String s_a1 = inputString.substring(0,i);
      String s_a2 = inputString.substring(i+1);
  
      if(DEBUGGING){
        // write to serial for debugging
        for(int i=0;i<s_a1.length();i++){
          Serial.write(s_a1[i]);    
        }
        for(int i=0;i<s_a2.length();i++){
          Serial.write(s_a2[i]);    
        }
      }  
      // 
      float a1 = s_a1.toFloat();
      float a2 = s_a2.toFloat();
      if(DEBUGGING){
        Serial.write('\n');
      }
      // set our angles on our servo...
      setArmServo(a1,a2);
    }
    // clear the string:
    inputString = "";
    // reset receivedAngles boolean
    receivedAngles = false;
  }
}
void setGrip(boolean closed){
  if(closed){
    Servo3.write(0);
  }else{
    Servo3.write(180);
  }
}
void setArmServo(float a1, float a2){
  Servo1.write((int)a1);
  Servo2.write((int)a2);
}
/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      receivedAngles = true;
    }else{
      // add it to the inputString:
      inputString += inChar;
    }
  }
}
