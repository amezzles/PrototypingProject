// PetFeederArduinoController.ino (Offline Capable LED)

// --- Pin Definitions ---
#define PIR_PIN 2       // PIR sensor OUT pin (Must be an interrupt pin: 2 or 3 on Uno)
#define LED_PIN 3       // Grove LED Signal pin

// --- Timing Constants ---
const unsigned long PI_RESPONSE_TIMEOUT_MS = 20000; // 20 seconds for Pi to respond after motion
const unsigned long HEARTBEAT_INTERVAL_MS = 15000; // Send a heartbeat every 15s
const unsigned long MOTION_ACTIVE_TIMEOUT_MS = 180000; // 3 minutes (Keep LED on / consider motion active for this long after last detection)
const unsigned long LED_FLASH_DURATION_MS = 2000;   // Flash LED for 2 seconds on success
const unsigned long LED_BLINK_INTERVAL_MS = 200;    // Blink speed for flashing

// --- State Variables ---
volatile boolean motionDetectedISRQFlag = false; // Flag set by ISR only
boolean piIsReady = false;
boolean awaitingPiResponseAfterMotion = false;
String currentTargetAnimal = "DOG"; // Default, Pi will be informed
unsigned long lastMotionTimestamp = 0;     // Last time PIR ISR was triggered
unsigned long lastMotionSignalToPiTime = 0;// Last time we sent MOTION_DETECTED to Pi
unsigned long lastHeartbeatToPi = 0;
boolean isLedFlashing = false;
unsigned long ledFlashStartTime = 0;
unsigned long lastLedBlinkTime = 0;
boolean ledBlinkState = false; // For toggling during flash

void setup() {
  Serial.begin(9600); // Match BAUD_RATE in Pi script
  while (!Serial) { ; } // Wait for serial port to connect

  pinMode(PIR_PIN, INPUT_PULLUP); // Use INPUT_PULLUP if needed, or INPUT
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW); // Start with LED off

  attachInterrupt(digitalPinToInterrupt(PIR_PIN), handleMotionISR, RISING);

  Serial.println("ARDUINO_BOOTING");
  Serial.println("Pet Feeder Controller Initialized."); // Log for human
}

void loop() {
  unsigned long currentMillis = millis(); // Get current time once per loop

  // --- Handle Pi Communication (if connected) ---
  checkPiMessages(currentMillis); // Check for incoming messages regardless of piIsReady state

  // --- Handle Motion Detection (Works Regardless of Pi Connection) ---
  if (motionDetectedISRQFlag) {
    lastMotionTimestamp = currentMillis; // Record time of latest motion detection
    Serial.println("DEBUG: PIR Detected Motion.");
    motionDetectedISRQFlag = false; // Reset the ISR flag AFTER processing it

    // --- Try to Send to Pi (only if ready and not already waiting) ---
    if (piIsReady && !awaitingPiResponseAfterMotion) {
        Serial.print("Sending MOTION_DETECTED to Pi for target: "); Serial.println(currentTargetAnimal);
        Serial.println("MOTION_DETECTED"); // Send actual command to Pi
        Serial.flush();
        awaitingPiResponseAfterMotion = true;
        lastMotionSignalToPiTime = currentMillis;
    } else if (!piIsReady) {
        Serial.println("DEBUG: Motion detected, but Pi not ready. Not sending.");
    } else { // Pi is ready, but we are still awaiting previous response
         Serial.println("DEBUG: Motion detected, but still awaiting previous Pi response.");
    }
  }

  // --- Motion Active State & Timeout (Works Regardless of Pi Connection) ---
  bool motionIsActive = (lastMotionTimestamp > 0) && (currentMillis - lastMotionTimestamp < MOTION_ACTIVE_TIMEOUT_MS);

  // --- Pi Response Timeout Check (Only relevant if Pi was ready) ---
  if (awaitingPiResponseAfterMotion && (currentMillis - lastMotionSignalToPiTime > PI_RESPONSE_TIMEOUT_MS)) {
    Serial.println("WARN: Timed out waiting for Pi's response to motion. Resetting wait flag.");
    awaitingPiResponseAfterMotion = false;
  }

  // --- Update LED State (Works Regardless of Pi Connection) ---
  updateLed(currentMillis, motionIsActive);

  // --- Heartbeat (Only send if Pi was ready at some point) ---
  // Or could send always and let Pi ignore if not ready, simpler:
  if (currentMillis - lastHeartbeatToPi > HEARTBEAT_INTERVAL_MS) {
    Serial.println("ARDUINO_HEARTBEAT"); // Pi will ignore if not ready
    lastHeartbeatToPi = currentMillis;
  }

  // --- Local Commands (Works Regardless of Pi Connection) ---
  handleLocalCommands(currentMillis); // Pass currentMillis for simulation below
}

// --- Interrupt Service Routine ---
void handleMotionISR() {
  // Keep this VERY short and fast! Just set the flag.
  motionDetectedISRQFlag = true;
}

// --- LED Control Logic ---
void updateLed(unsigned long currentMillis, bool motionIsActive) {
  // Flashing state overrides other states
  if (isLedFlashing) {
    if (currentMillis - ledFlashStartTime >= LED_FLASH_DURATION_MS) {
      isLedFlashing = false;
      digitalWrite(LED_PIN, motionIsActive ? HIGH : LOW); // Set based on current motion
      Serial.println("DEBUG: LED Flash finished.");
    } else {
      if (currentMillis - lastLedBlinkTime >= LED_BLINK_INTERVAL_MS) {
        ledBlinkState = !ledBlinkState; // Toggle state
        digitalWrite(LED_PIN, ledBlinkState);
        lastLedBlinkTime = currentMillis;
      }
    }
  } else {
    // Not flashing, set LED based purely on motion activity
    digitalWrite(LED_PIN, motionIsActive ? HIGH : LOW);
  }
}

void startLedFlash() {
  if (!isLedFlashing) {
    Serial.println("DEBUG: Starting LED Flash.");
    isLedFlashing = true;
    ledFlashStartTime = millis();
    lastLedBlinkTime = ledFlashStartTime;
    ledBlinkState = HIGH;
    digitalWrite(LED_PIN, ledBlinkState);
  }
}

// --- Communication with Pi ---
void checkPiMessages(unsigned long currentMillis) {
  if (Serial.available() > 0) {
    String piMessage = Serial.readStringUntil('\n');
    piMessage.trim();
    if (piMessage.length() == 0) return;

    Serial.print("Arduino RX from Pi: "); Serial.println(piMessage);

    if (piMessage.equals("PI_READY")) {
      piIsReady = true; // Mark Pi as ready
      Serial.println("INFO: Pi reported READY. Sending current target animal.");
      sendTargetAnimalToPi();
      lastHeartbeatToPi = currentMillis;
    } else if (piMessage.startsWith("PI_ACK_TARGET:")) {
      // Process acknowledgment as before...
      String ackAnimal = piMessage.substring(piMessage.indexOf(':') + 1);
      Serial.print("INFO: Pi acknowledged target animal set to: "); Serial.println(ackAnimal);
      // Optional: Check for mismatch and resend (already in previous code)
    } else if (piMessage.equals("CAT_CONFIRMED")) {
      awaitingPiResponseAfterMotion = false; // Pi responded
      Serial.println("INFO: Pi confirmed CAT.");
      if (currentTargetAnimal.equals("CAT")) {
        Serial.println("ACTION: Target is CAT, activating flash & bowl!");
        startLedFlash(); // FLASH LED on success
        // openBowl();
      } // else { No flash/action if wrong target }
    } else if (piMessage.equals("DOG_CONFIRMED")) {
      awaitingPiResponseAfterMotion = false; // Pi responded
      Serial.println("INFO: Pi confirmed DOG.");
      if (currentTargetAnimal.equals("DOG")) {
        Serial.println("ACTION: Target is DOG, activating flash & bowl!");
        startLedFlash(); // FLASH LED on success
        // openBowl();
      } // else { No flash/action if wrong target }
    } else if (piMessage.equals("WRONG_ANIMAL_DETECTED") ||
               piMessage.equals("NO_ANIMAL_DETECTED_BY_AI") ||
               piMessage.equals("AI_NOT_READY") ||
               piMessage.equals("NO_TARGET_CONFIGURED") ||
               piMessage.startsWith("PI_ERR"))
    {
      // Any non-confirmation or error message from Pi regarding motion response
      Serial.println("INFO: Pi response received, but not a confirmation for target animal.");
      awaitingPiResponseAfterMotion = false; // Pi responded, stop waiting
    } else if (piMessage.equals("PI_PONG")) {
      lastHeartbeatToPi = currentMillis; // Pi is alive
    } else if (piMessage.equals("PI_SHUTTING_DOWN")) {
        Serial.println("WARN: Pi reported shutting down. Setting piIsReady to false.");
        piIsReady = false;
        awaitingPiResponseAfterMotion = false; // Stop waiting if Pi shuts down
    }
  }
}

void sendTargetAnimalToPi() {
  if (piIsReady) { // Only send if Pi is actually ready
    Serial.print("TARGET_ANIMAL:");
    Serial.println(currentTargetAnimal);
    Serial.flush();
    Serial.print("INFO: Sent TARGET_ANIMAL:"); Serial.print(currentTargetAnimal); Serial.println(" to Pi.");
  } // else { Do nothing if Pi not ready }
}

void handleLocalCommands(unsigned long currentMillis) { // Pass currentMillis
  if (Serial.available() > 0) {
      char firstChar = Serial.peek();
      // Handle SET_TARGET command (works regardless of Pi connection)
      if (firstChar == 'S') {
          String command = Serial.readStringUntil('\n');
          command.trim();
          if (command.startsWith("SET_TARGET:")) {
              String newTarget = command.substring(11);
              newTarget.toUpperCase();
              if (newTarget.equals("CAT") || newTarget.equals("DOG")) {
                  if (!newTarget.equals(currentTargetAnimal)) {
                      currentTargetAnimal = newTarget;
                      Serial.print("CONFIG: Target animal preference changed locally to: "); Serial.println(currentTargetAnimal);
                      sendTargetAnimalToPi(); // Attempt to inform Pi if it's ready
                  } else {
                      Serial.print("INFO: Target animal is already "); Serial.println(newTarget);
                  }
              } else {
                  Serial.println("ERROR: Invalid target. Use CAT or DOG.");
              }
              return; // Command processed
          }
          // If it started with 'S' but wasn't SET_TARGET, discard rest of line
          // (or add other 'S' commands)
          while (Serial.available() > 0 && Serial.read() != '\n'); // Discard rest of line
          return;
      }
      // --- Add Test Command to simulate flash ---
      else if (firstChar == 'T') {
          String command = Serial.readStringUntil('\n');
          command.trim();
          if (command.equalsIgnoreCase("TEST_FLASH")) {
             Serial.println("DEBUG: Manually starting LED Flash via Test command.");
             startLedFlash();
             return; // Command processed
          }
          // Discard rest of line if not TEST_FLASH
          while (Serial.available() > 0 && Serial.read() != '\n');
          return;
      }

      // If not 'S' or 'T', discard other unexpected serial input
      while (Serial.available() > 0) {
          Serial.read();
      }
  }
}

// --- Motor Control Placeholder ---
// void openBowl() { Serial.println("MOTOR: Opening bowl simulation..."); }
