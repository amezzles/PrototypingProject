#include <Servo.h>

Servo myServo; // Create a servo object to control a servo motor

int servoPin = 9; // Define the pin to which the servo is connected

void setup() {
  myServo.attach(servoPin); // Attaches the servo on pin 9 to the servo object
  int currSlot = 0;
}

void loop() {
}

void turnSlot(int currslot, char direction){
  if (direction = 'l'){
    myServo.write(currSlot*72 - 72);
  } else {
    myServo.write(currSlot*72 - 72);
  }
  delay(5000);
}

void turnSlot(int slot){
  myServo.write(slot*72);
  delay(5000);
}