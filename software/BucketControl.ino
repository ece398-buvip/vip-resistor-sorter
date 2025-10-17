#include <Servo.h>
Servo claw;
Servo pull; // Create a servo object
#define Magnet 11 
//if we need to we can add a capcitatance sensor to detect if pull servo is in the right position

void setup() {
pinMode(Magnet,OUTPUT);
claw.attach(9); // Attach claw servo to Pin 10 (Standard servo with position controls)
pull.attach(10); // Attach Pull continous servo to Pin 9
}

void runSequence(){
  //For Continuous rotation servos, 0-89 is Clockwise??, 90 is stop, 91-180 is the other direction the 90 is not perfect it will be a little off so you will have to test it and find the perfect stop value
  //Also we need to figure out the delays or find a better way to make sure everything is happening correctly as of right now this is an open loop system, hardcoded maybe adding a sensor is a good idea to have some fail prevention
  
  pull.write(135);
      delay(1000);
  
  pull.write(90); //Change to whatever angle makes servo stop
  digitalWrite(Magnet,HIGH);
      delay(200);
  
  pull.write(45);
      delay(1000);
  
  pull.write(90); //Change to whatever angle makes servo stop
    
  claw.write(90);// drop resistor
      delay(500);
  
  claw.write(0);//  go back to intial position
      delay(500);
  
  pull.write(135);
      delay(1000);
  
  pull.write(90); //Change to whatever angle makes servo stop
  digitalWrite(Magnet,LOW);
      delay(200);
  
  pull.write(45);
      delay(1000);
  
  pull.write(90); //Change to whatever angle makes servo stop
}
