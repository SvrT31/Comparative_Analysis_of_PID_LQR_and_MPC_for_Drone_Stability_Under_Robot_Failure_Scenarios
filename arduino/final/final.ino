#include <Servo.h>

Servo motor_1;
Servo motor_2;
Servo motor_3;
Servo motor_4;

int pwm1, pwm2, pwm3, pwm4;
unsigned char message[5]; 

void setup(){
  Serial.begin(115200);
  
  motor_1.attach(8);
  motor_2.attach(9);
  motor_3.attach(10);
  motor_4.attach(11);
  delay(1000);
  
  motor_1.write(30);
  motor_2.write(30);
  motor_3.write(30);
  motor_4.write(40);
  delay(3000);
}

void loop(){    
  if (Serial.available() >= 5) {
    Serial.readBytes(message, 5);
    
    if (message[0] == 0xFF) {
      pwm1 = message[1];
      pwm2 = message[2];
      pwm3 = message[3];
      pwm4 = message[4];
      
          
      motor_1.write(pwm1);
      motor_2.write(pwm2);
      motor_3.write(pwm3);
      motor_4.write(pwm4);
    }
  }
  delay(33); // Short delay
}
