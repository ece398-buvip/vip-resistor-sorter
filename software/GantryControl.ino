#include <Servo.h>

// Prox Sensor inputs
#define GANTRY_PROX A1 //Metal sensor

// Gantry Servos pins
 #define FORKS_SERVO_PWM 9       //pwm SG90
 #define SLIDE_SERVO_PWM 10      //pwm FS5103R Continuous rotation (Stationary at 12.5 ms)

#define MAGNET_RELAY A4



void setup() {
  pinMode(MAGNET_RELAY,OUTPUT);
  SlideServo.attach(9); // Attach claw servo to Pin 10 (Standard servo with position controls)
  ForksServo.attach(10); // Attach Pull continous servo to Pin 9

}

void start_gantry_seq() //THE TIME DELAY WILL NEED TO BE TESTED
{
    //Take out drawer
    // Move forward initial (This is to actually get the prox sensor on the tape)
    SlideServo.writeMicroseconds(2000); // forward full speed
    delay(100); //<-- TEST TIME DELAY
    //Move forward until prox is triggered (The prox will continue to move until it runs out of tape)
    while (digitalRead(GANTRY_PROX) == HIGH)
    {
        SlideServo.writeMicroseconds(2000); // forward
    }
    SlideServo.writeMicroseconds(1500); // stop servo
    delay(300);

    // Turn ON magnet
    digitalWrite(MAGNET_RELAY, HIGH);
    delay(300);

    // Move backward initial
    SlideServo.writeMicroseconds(1000); // backward full speed
    delay(100); //<-- TEST TIME DELAY
    //Move forward until prox is triggered
    while (digitalRead(GANTRY_PROX) == HIGH)
    {
        SlideServo.writeMicroseconds(1000); // reverse
    }
    SlideServo.writeMicroseconds(1500); // stop servo
    delay(300);

    //Place resistor in the drawer
    // Open forks
    ForksServo.write(0); // open claws
    delay(300);

    // Close forks
    ForksServo.write(90); // close claws
    delay(300);

    //Return drawer to cabinet
     //Take out drawer
    // Move forward initial
    SlideServo.writeMicroseconds(2000); // forward full speed
    delay(100); //<-- TEST TIME DELAY
    //Move forward until prox is triggered
    while (digitalRead(GANTRY_PROX) == HIGH)
    {
        SlideServo.writeMicroseconds(2000); // forward
    }
    SlideServo.writeMicroseconds(1500); // stop servo
    delay(300);

    //power OFF magnet
    digitalWrite(MAGNET_RELAY, LOW);
    delay(300);

    // Move backward initial
    SlideServo.writeMicroseconds(1000); // backward full speed
    delay(100); //<-- TEST TIME DELAY
    //Move backward until prox is triggered
    while (digitalRead(GANTRY_PROX) == HIGH)
    {
        SlideServo.writeMicroseconds(1000); // reverse
    }
    SlideServo.writeMicroseconds(1500); // stop servo
    delay(300);

}

void loop() {
  // put your main code here, to run repeatedly:

}
