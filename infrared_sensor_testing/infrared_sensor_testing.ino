const int pirPin = 6;   // PIR sensor connected to digital pin 6
const int ledPin = 7;   // LED connected to digital pin 7

void setup() {
    pinMode(pirPin, INPUT);  // Set PIR sensor as input
    pinMode(ledPin, OUTPUT); // Set LED as output
}

void loop() {
    int motionDetected = digitalRead(pirPin); // Read PIR sensor state
    
    if (motionDetected == HIGH) {
        digitalWrite(ledPin, HIGH); // Turn LED on if motion is detected
    } else {
        digitalWrite(ledPin, LOW);  // Turn LED off if no motion
    }
}