#include <Servo.h>

Servo motor_1;
Servo motor_2;
Servo motor_3;
// Servo motor_4;

int gotArray[4];
int rpm1, rpm2, rpm3, rpm4;
bool gotData = false;

void setup(){
  Serial.begin(115200);
  
  // Motors attached to arduino pins
  motor_1.attach(9);
  motor_2.attach(10);
  motor_3.attach(11);
  // motor_4.attach(12);
  delay(1000);
  
  // Sending min throttle
  motor_1.write(300);
  motor_2.write(30);
  motor_3.write(30);
  // motor_4.write(40);
  delay(3000);
  
}

void loop(){    
  int throttlevalue = 100;
  motor_1.write(70);
  motor_2.write(70);
  motor_3.write(70);
  delay(33); 
}
