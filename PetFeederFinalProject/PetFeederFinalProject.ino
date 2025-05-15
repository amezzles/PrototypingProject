// PetFeederArduinoActuator.ino

// --- Pin Definitions ---
#define LED_PIN 3       // Grove LED Signal pin (or any digital pin)

// --- Timing Constants ---
const unsigned long LED_ON_DURATION_MS = 5000; // Keep LED on for 5 seconds

// --- State Variables ---
boolean piIsReady = false;
String currentTargetAnimalOnArduino = "DOG"; // Arduino's current understanding of the target
unsigned long ledTurnOffTime = 0;
boolean ledIsOn = false;
unsigned long lastHeartbeatToPi = 0;

// For sending initial target animal only once after Pi is ready
boolean initialTargetSent = false;

// PetFeederArduinoActuator.ino (with more debugging)

// ... (pin defs, constants, state vars - including lastHeartbeatToPi declaration) ...
unsigned long lastLoopTime = 0; // For checking if loop is running

void setup() {
  Serial.begin(9600);
  while (!Serial) { ; }
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  Serial.println("ARDUINO_ACTUATOR_BOOTING");
  Serial.println("DEBUG_ARD: Setup complete. Waiting for PI_READY."); // For Arduino Serial Monitor
}

void loop() {
  unsigned long currentMillis = millis();

  // Debug: Check if loop is running
  if (currentMillis - lastLoopTime > 2000) { // Print every 2 seconds
    Serial.println("DEBUG_ARD: Loop is running...");
    lastLoopTime = currentMillis;
  }

  // --- Handle Incoming Pi Messages ---
  if (Serial.available() > 0) {
    Serial.println("DEBUG_ARD: Serial.available() > 0 is TRUE"); // Got something
    String piMessage = Serial.readStringUntil('\n');
    piMessage.trim();
    Serial.print("DEBUG_ARD: Raw message received (trimmed, len="); Serial.print(piMessage.length()); Serial.print("): ["); Serial.print(piMessage); Serial.println("]");


    if (piMessage.length() > 0) {
      if (piMessage.equals("PI_READY")) {
        Serial.println("DEBUG_ARD: 'PI_READY' MATCHED!"); // Critical check
        piIsReady = true;
        initialTargetSent = false;
        Serial.println("INFO_ARD: Pi reported READY. Sending current target animal NOW.");
        sendTargetAnimalToPi();
        initialTargetSent = true;
        // lastHeartbeatToPi = currentMillis; // Removed as per no Arduino heartbeat logic in this version
      } else if (piMessage.startsWith("TRIGGER_ACTION:")) {
        // ... (rest of your existing logic for TRIGGER_ACTION) ...
        String animalDetectedByPi = piMessage.substring(piMessage.indexOf(':') + 1);
        Serial.print("INFO_ARD: Pi triggered action for: "); Serial.println(animalDetectedByPi);
        digitalWrite(LED_PIN, HIGH);
        ledIsOn = true;
        ledTurnOffTime = currentMillis + LED_ON_DURATION_MS;
        Serial.println("ACTION_ARD: LED ON");
      } else if (piMessage.startsWith("PI_ACK_TARGET:")) {
        // ... (rest of your existing logic for PI_ACK_TARGET) ...
        String ackAnimal = piMessage.substring(piMessage.indexOf(':') + 1);
        Serial.print("INFO_ARD: Pi acknowledged target as: "); Serial.println(ackAnimal);
        currentTargetAnimalOnArduino = ackAnimal;
      } else if (piMessage.equals("PI_SHUTTING_DOWN")) {
        // ... (rest of your existing logic for PI_SHUTTING_DOWN) ...
         Serial.println("WARN_ARD: Pi reported shutting down.");
        piIsReady = false;
        digitalWrite(LED_PIN, LOW);
        ledIsOn = false;
      } else {
        Serial.println("DEBUG_ARD: Pi message received, but not a recognized command.");
      }
    } else {
       Serial.println("DEBUG_ARD: Received empty message after trim.");
    }
  } // End of if (Serial.available())

  // --- Send Initial/Current Target Animal to Pi ---
  if (piIsReady && !initialTargetSent) {
    Serial.println("DEBUG_ARD: Condition met to send target animal from loop.");
    sendTargetAnimalToPi();
    initialTargetSent = true;
  }

  // --- Manage LED State ---
  if (ledIsOn && currentMillis >= ledTurnOffTime) {
    digitalWrite(LED_PIN, LOW);
    ledIsOn = false;
    Serial.println("ACTION_ARD: LED OFF");
  }

  // --- Handle Local Commands (e.g., from Arduino Serial Monitor) ---
  handleLocalCommands();
}

void sendTargetAnimalToPi() {
  if (piIsReady) {
    Serial.println("DEBUG_ARD: Inside sendTargetAnimalToPi(), piIsReady is TRUE.");
    Serial.print("TARGET_ANIMAL:");
    Serial.println(currentTargetAnimalOnArduino);
    Serial.flush();
    Serial.println("DEBUG_ARD: TARGET_ANIMAL message sent."); // For Arduino Serial Monitor
  } else {
    Serial.println("DEBUG_ARD: Inside sendTargetAnimalToPi(), but piIsReady is FALSE.");
  }
}

// handleLocalCommands() remains the same
// void handleLocalCommands() { ... }

void handleLocalCommands() {
  if (Serial.available() > 0) {
      char firstChar = Serial.peek();
      if (firstChar == 'S') {
          String command = Serial.readStringUntil('\n');
          command.trim();
          if (command.startsWith("SET_TARGET:")) {
              String newTarget = command.substring(11);
              newTarget.toUpperCase();
              if (newTarget.equals("CAT") || newTarget.equals("DOG")) {
                  if (!newTarget.equals(currentTargetAnimalOnArduino)) {
                      currentTargetAnimalOnArduino = newTarget;
                      Serial.print("CONFIG_ARD: Target animal set locally to: "); Serial.println(currentTargetAnimalOnArduino);
                      sendTargetAnimalToPi(); // Inform Pi of the change
                      initialTargetSent = true; // Consider it sent
                  } else {
                      // Serial.print("INFO_ARD: Target animal is already "); Serial.println(newTarget);
                  }
              } else {
                  Serial.println("ERROR_ARD: Invalid target. Use CAT or DOG.");
              }
              return;
          }
          while (Serial.available() > 0 && Serial.read() != '\n'); // Discard
          return;
      }
      // Discard other unexpected input
      while (Serial.available() > 0) { Serial.read(); }
  }
} 