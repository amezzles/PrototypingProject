 import serial
import time
import argparse
import sys
import atexit
from typing import List
from collections import defaultdict # For consistency check

import numpy as np
from picamera2 import Picamera2, CompletedRequest
from picamera2.devices import IMX500
from picamera2.devices.imx500 import NetworkIntrinsics
from picamera2.devices.imx500.postprocess import softmax

LOG_PREFIX = "[PET_FEEDER_PI] "

def log_message(message):
    print(f"{LOG_PREFIX}{message}")
    sys.stdout.flush() # FLUSH HERE
    sys.stderr.flush() # FLUSH HERE TOO

log_message("Script entry point reached (v2). Waiting 10 seconds before proceeding...")
time.sleep(45)
log_message("Wait finished (v2). Proceeding with initialization.")

# --- Configuration ---
SERIAL_PORT = '/dev/ttyACM0' # Or your actual port
BAUD_RATE = 9600

# AI Model and Label Configuration
DEFAULT_MODEL_PATH = "/home/pi/Projects/RPI_AI_Cam/Test_1/models/imx500_network_mobilenet_v2.rpk"
DEFAULT_LABELS_PATH = "/home/pi/Projects/RPI_AI_Cam/Test_1/assets/imagenet_labels.txt"

# UPDATE THESE KEYWORDS BASED ON YOUR IMAGENET_LABELS.TXT
CAT_KEYWORDS = {
    "tabby, tabby cat", "tiger cat", "persian cat", "siamese cat, siamese",
    "egyptian cat", "cat" # Generic 'cat' if it helps
}
DOG_KEYWORDS = {
    "chihuahua", "japanese spaniel", "maltese dog, maltese terrier, maltese",
    "golden retriever", "labrador retriever", "german shepherd, german shepherd dog, german police dog, alsatian",
    "poodle", "beagle", "bulldog", "boxer", "pug, pug-dog", "dog" # Generic 'dog'
}

# AI Processing Parameters
CONFIDENCE_THRESHOLD = 0.01 # Start low if model scores are generally low, adjust upwards if possible
TOP_N_RESULTS = 5        # Check more results if confidence is low

# Consistency Check Parameters
# When motion is detected, AI will run for this duration
AI_ACTIVE_DURATION_SECONDS = 15 # How long to run AI after a motion trigger
FRAMES_PER_SECOND_AI = 1 # How many classifications to attempt per second (approx)
MIN_CONSISTENT_DETECTIONS = 3 # How many times target animal must be seen in AI_ACTIVE_DURATION_SECONDS

# --- Global Variables ---
arduino = None
imx500_device = None
picam2_camera = None
network_intrinsics = None
loaded_labels = None
is_ai_ready = False
current_target_animal_name_from_arduino = None # "CAT" or "DOG" (UPPERCASE from Arduino)
last_motion_time = 0
ai_processing_active = False

# --- AI Helper Functions (Classification, load_ai_labels, initialize_ai_model_and_camera, perform_classification) ---
class Classification:
    def __init__(self, idx: int, score: float, label: str): self.idx = idx; self.score = score; self.label = label
    def __repr__(self): return f"Classification(label='{self.label}', score={self.score:.3f})"

def log_message(message):
    print(f"{LOG_PREFIX}{message}")

def load_ai_labels(labels_path: str):
    global loaded_labels
    try:
        with open(labels_path, "r") as f: loaded_labels = f.read().splitlines()
        if not loaded_labels: log_message(f"ERROR: Labels file '{labels_path}' is empty."); return False
        log_message(f"AI: Loaded {len(loaded_labels)} labels from {labels_path}")
        return True
    except FileNotFoundError: log_message(f"ERROR: Labels file not found: {labels_path}"); return False
    except Exception as e: log_message(f"ERROR: Loading labels: {e}"); return False

def initialize_ai_model_and_camera(model_path: str, labels_path: str, cmd_args_dict: dict = None):
    global imx500_device, picam2_camera, network_intrinsics, is_ai_ready
    
    # --- Start of Detailed Logging Section ---
    log_message("AI_INIT_FUNC: Entered.") # Step 0
    
    try:
        log_message("AI_INIT_FUNC: Step 1 - Creating IMX500 instance...")
        imx500_device = IMX500(model_path) # This is where libcamera might print "Loading firmware..."
        log_message("AI_INIT_FUNC: Step 2 - IMX500 instance created.")

        log_message("AI_INIT_FUNC: Step 3 - Getting network_intrinsics...")
        network_intrinsics = imx500_device.network_intrinsics
        log_message(f"AI_INIT_FUNC: Step 4 - network_intrinsics object: {'Exists' if network_intrinsics else 'None'}")
        log_message(f"AI_INIT_FUNC: Step 4a - intrinsics.task before default: {network_intrinsics.task if network_intrinsics and hasattr(network_intrinsics, 'task') else 'N/A or No Intrinsics'}")

        if not network_intrinsics:
            log_message("AI_INIT_FUNC: Step 4b - No intrinsics found, creating default.")
            network_intrinsics = NetworkIntrinsics()
            network_intrinsics.task = "classification"
        elif network_intrinsics.task != "classification":
            log_message(f"ERROR: Loaded model task is '{network_intrinsics.task}', not 'classification'.")
            return False # Exit early if wrong model type
        
        log_message("AI_INIT_FUNC: Step 5 - Processing cmd_args_dict...")
        if cmd_args_dict:
            if 'softmax' in cmd_args_dict:
                log_message(f"AI_INIT_FUNC: Setting softmax from cmd_args: {cmd_args_dict['softmax']}")
                network_intrinsics.softmax = cmd_args_dict['softmax']
            if 'labels_file_override' in cmd_args_dict and cmd_args_dict['labels_file_override'] is not None:
                log_message(f"AI_INIT_FUNC: Overriding labels_path from cmd_args to: {cmd_args_dict['labels_file_override']}")
                labels_path = cmd_args_dict['labels_file_override']
        log_message(f"AI_INIT_FUNC: Step 6 - Calling load_ai_labels with path: {labels_path}")
        if not load_ai_labels(labels_path):
            log_message("AI_INIT_FUNC: load_ai_labels returned False.")
            return False # Exit early if labels fail to load
        log_message("AI_INIT_FUNC: Step 7 - load_ai_labels successful.")

        log_message("AI_INIT_FUNC: Step 8 - Setting intrinsics.labels if needed...")
        if network_intrinsics.labels is None and loaded_labels:
            network_intrinsics.labels = loaded_labels
            log_message("AI_INIT_FUNC: Step 8a - intrinsics.labels was set from loaded_labels.")
        
        log_message("AI_INIT_FUNC: Step 9 - Updating intrinsics with defaults...")
        network_intrinsics.update_with_defaults()
        log_message("AI_INIT_FUNC: Step 10 - Intrinsics updated.")

        log_message("AI_INIT_FUNC: Step 11 - Creating Picamera2 instance...")
        picam2_camera = Picamera2(imx500_device.camera_num)
        log_message("AI_INIT_FUNC: Step 12 - Picamera2 instance created.")

        log_message("AI_INIT_FUNC: Step 13 - Creating camera config...")
        config = picam2_camera.create_still_configuration(
            main={"size": (640, 480)},
            controls={"FrameRate": network_intrinsics.inference_rate if hasattr(network_intrinsics, 'inference_rate') else 10},
            buffer_count=6
        )
        log_message("AI_INIT_FUNC: Step 14 - Camera config created.")
        
        log_message("AI_INIT_FUNC: Step 15 - Starting Picamera2 camera (picam2_camera.start)...")
        # External libcamera messages will appear around here in the journal
        picam2_camera.start(config, show_preview=False)
        log_message("AI_INIT_FUNC: Step 16 - picam2_camera.start() completed.") # <<< CRUCIAL: Does it get here?

        log_message("AI_INIT_FUNC: Step 17 - Setting auto aspect ratio if applicable...")
        if hasattr(network_intrinsics, 'preserve_aspect_ratio') and network_intrinsics.preserve_aspect_ratio:
            if hasattr(imx500_device, 'set_auto_aspect_ratio'):
                imx500_device.set_auto_aspect_ratio()
                log_message("AI_INIT_FUNC: Step 17a - Auto aspect ratio set.")
        
        log_message("AI_INIT_FUNC: Step 18 - Sleeping for AI warmup...")
        time.sleep(2)
        is_ai_ready = True
        log_message("AI_INIT_FUNC: Step 19 - AI Model and Camera Initialized Successfully (returning True).")
        return True
    except Exception as e:
        log_message(f"ERROR: Exception during AI Initialization in initialize_ai_model_and_camera: {e}")
        import traceback
        traceback.print_exc() # This prints to stderr, log_message flushes stderr
        is_ai_ready = False
        return False
    finally:
        log_message("AI_INIT_FUNC: Exiting.") # Step Z - will print even if error occurs in try
    # --- End of Detailed Logging Section ---

def perform_classification():
    if not is_ai_ready or not picam2_camera or not imx500_device:
        # log_message("AI_CLASSIFY: AI not ready for perform_classification call.") # Already handled by is_ai_ready check before calling run_ai_for_duration
        return []
    try:
        # log_message("AI_CLASSIFY: Attempting capture_request...") # Can be very verbose if called frequently
        request = picam2_camera.capture_request() # THIS IS A POTENTIAL HANG POINT
        
        if request is None:
            log_message("WARN: AI_CLASSIFY: Failed to capture request (returned None).")
            return []
        # log_message("AI_CLASSIFY: capture_request successful.")

        metadata = request.get_metadata()
        np_outputs = imx500_device.get_outputs(metadata)
        request.release()

        if np_outputs is None or len(np_outputs) == 0:
            # log_message("WARN: AI_CLASSIFY: No output tensor from model (np_outputs is None or empty).") # Verbose
            return []
        
        np_output = np_outputs[0].flatten()
        if hasattr(network_intrinsics, 'softmax') and network_intrinsics.softmax:
            np_output = softmax(np_output)
        
        num_results_to_get = min(TOP_N_RESULTS, len(np_output))
        if num_results_to_get == 0: return []

        top_indices = np.argpartition(-np_output, num_results_to_get-1)[:num_results_to_get]
        top_indices = top_indices[np.argsort(-np_output[top_indices])]
        
        classifications = []
        for index_from_model in top_indices:
            score = float(np_output[index_from_model])
            if score >= CONFIDENCE_THRESHOLD:
                if loaded_labels and 0 <= index_from_model < len(loaded_labels):
                    label = loaded_labels[index_from_model]
                    classifications.append(Classification(index_from_model, score, label))
        return classifications
    except Exception as e:
        log_message(f"ERROR: AI_CLASSIFY: Exception during classification: {e}")
        import traceback
        traceback.print_exc()
        # Consider re-initializing camera or flagging AI as not ready if this happens repeatedly
        # global is_ai_ready
        # is_ai_ready = False 
        # log_message("ERROR: AI_CLASSIFY: Setting is_ai_ready to False due to exception.")
        return []

def run_ai_for_duration():
    global ai_processing_active, current_target_animal_name_from_arduino # Make sure this is accessible
    log_message(f"AI: Starting AI processing for target '{current_target_animal_name_from_arduino}'. Will run for ~{AI_ACTIVE_DURATION_SECONDS}s.")
    ai_processing_active = True
    start_time = time.time()
    
    cat_detection_count = 0
    dog_detection_count = 0
    frames_processed_this_event = 0

    response_to_arduino = "NO_ANIMAL_DETECTED_BY_AI\n" 

    while time.time() - start_time < AI_ACTIVE_DURATION_SECONDS:
        frames_processed_this_event += 1
        frame_start_time = time.time() # For timing each frame
        log_message(f"AI_LOOP: Frame {frames_processed_this_event}, Time into event: {frame_start_time - start_time:.1f}s")

        results = perform_classification() # This calls the classification logic
        
        frame_cat_found = False
        frame_dog_found = False
        best_label_this_frame = "None"
        best_score_this_frame = 0.0

        if results:
            # Log top raw result for this frame for more insight
            best_label_this_frame = results[0].label
            best_score_this_frame = results[0].score
            log_message(f"AI_LOOP: Frame {frames_processed_this_event} - Top Raw Result: '{results[0].label}' (Score: {results[0].score:.4f})")

            for res_idx, res in enumerate(results): # Iterate through top N results
                label_lower = res.label.lower()
                # Check for CAT
                if any(keyword.lower() in label_lower for keyword in CAT_KEYWORDS):
                    frame_cat_found = True
                    log_message(f"AI_LOOP: Frame {frames_processed_this_event} - CAT Keyword Match: '{res.label}' (Score: {res.score:.4f})")
                    # No 'break' here if you want to see if other keywords also match in top N
                
                # Check for DOG
                if any(keyword.lower() in label_lower for keyword in DOG_KEYWORDS):
                    frame_dog_found = True
                    log_message(f"AI_LOOP: Frame {frames_processed_this_event} - DOG Keyword Match: '{res.label}' (Score: {res.score:.4f})")
                    # No 'break' here
        else:
            log_message(f"AI_LOOP: Frame {frames_processed_this_event} - No classification results (or all below threshold).")
        
        if frame_cat_found: cat_detection_count += 1
        if frame_dog_found: dog_detection_count += 1

        log_message(f"AI_LOOP: Frame {frames_processed_this_event} - Frame Cat: {frame_cat_found}, Frame Dog: {frame_dog_found}. Cumulative Cats: {cat_detection_count}, Dogs: {dog_detection_count}")

        # Check for early exit if consistently detected (already in your code)
        if current_target_animal_name_from_arduino == "CAT" and cat_detection_count >= MIN_CONSISTENT_DETECTIONS:
            log_message(f"AI: Consistent CAT detection ({cat_detection_count} times). Exiting AI loop early.")
            response_to_arduino = "CAT_CONFIRMED\n"
            break 
        if current_target_animal_name_from_arduino == "DOG" and dog_detection_count >= MIN_CONSISTENT_DETECTIONS:
            log_message(f"AI: Consistent DOG detection ({dog_detection_count} times). Exiting AI loop early.")
            response_to_arduino = "DOG_CONFIRMED\n"
            break
        
        frame_processing_time = time.time() - frame_start_time
        sleep_duration = (1.0 / FRAMES_PER_SECOND_AI) - frame_processing_time
        if sleep_duration > 0:
            time.sleep(sleep_duration)
        # else:
        #     log_message(f"AI_LOOP: Warning - Frame processing ({frame_processing_time:.2f}s) took longer than target interval ({1.0/FRAMES_PER_SECOND_AI:.2f}s)")   

# After AI_ACTIVE_DURATION_SECONDS or early exit, make final decision (already in your code)
    if response_to_arduino == "NO_ANIMAL_DETECTED_BY_AI\n": 
        if current_target_animal_name_from_arduino == "CAT":
            if cat_detection_count >= MIN_CONSISTENT_DETECTIONS: response_to_arduino = "CAT_CONFIRMED\n"
            elif dog_detection_count >= MIN_CONSISTENT_DETECTIONS : response_to_arduino = "WRONG_ANIMAL_DETECTED\n"
        elif current_target_animal_name_from_arduino == "DOG":
            if dog_detection_count >= MIN_CONSISTENT_DETECTIONS: response_to_arduino = "DOG_CONFIRMED\n"
            elif cat_detection_count >= MIN_CONSISTENT_DETECTIONS: response_to_arduino = "WRONG_ANIMAL_DETECTED\n"
        
    log_message(f"AI: Finished processing motion event. Final Cat count: {cat_detection_count}, Dog count: {dog_detection_count}. Sending: {response_to_arduino.strip()}")
    if arduino and arduino.is_open:
        try:
            arduino.write(response_to_arduino.encode())
        except serial.SerialException as se_write:
            log_message(f"ERROR: Serial: Failed to write to Arduino during AI response: {se_write}")
            # Handle potential disconnect during write
    ai_processing_active = False

def main():
    log_message("MAIN_FUNC: Entered main() function.")
    global current_target_animal_name_from_arduino, arduino, ai_processing_active, last_motion_time, is_ai_ready # Ensure is_ai_ready is global

    # --- Initialize AI with Retries ---
    max_ai_init_attempts = 3
    ai_init_attempt_delay = 15 # seconds between attempts
    ai_init_success = False

    for attempt in range(1, max_ai_init_attempts + 1):
        log_message(f"MAIN_FUNC: AI Initialization Attempt {attempt}/{max_ai_init_attempts}...")
        ai_args = {'softmax': True}
        try:
            if initialize_ai_model_and_camera(DEFAULT_MODEL_PATH, DEFAULT_LABELS_PATH, ai_args):
                ai_init_success = True
                log_message("MAIN_FUNC: initialize_ai_model_and_camera returned True.")
                break # Success, exit retry loop
            else:
                log_message(f"MAIN_FUNC: initialize_ai_model_and_camera attempt {attempt} returned False.")
        except Exception as e_outer_init:
            log_message(f"CRITICAL_MAIN: Exception during AI init attempt {attempt}: {e_outer_init}")
            import traceback
            traceback.print_exc()
        
        if not ai_init_success and attempt < max_ai_init_attempts:
            log_message(f"MAIN_FUNC: AI Init attempt {attempt} failed. Retrying in {ai_init_attempt_delay} seconds...")
            time.sleep(ai_init_attempt_delay)
        elif not ai_init_success:
            log_message("CRITICAL_MAIN: All AI Initialization attempts failed.")

    if not ai_init_success:
        log_message("CRITICAL_MAIN: AI Features will be disabled.")
        is_ai_ready = False # Explicitly set if all attempts failed
    else:
        log_message("MAIN_FUNC: AI Initialization Succeeded after retries (or on first attempt).")
        # is_ai_ready should have been set to True inside initialize_ai_model_and_camera

    # --- Initialize Serial ---
    while True: # Keep trying to connect to Arduino
        try:
            arduino = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            log_message(f"Serial: Connected to Arduino on {SERIAL_PORT}")
            time.sleep(2) # Allow Arduino to reset/initialize
            arduino.write("PI_READY\n".encode()) # Inform Arduino Pi is up
            break # Exit connection loop
        except serial.SerialException as e:
            log_message(f"Serial: Connection error: {e}. Retrying in 5 seconds...")
            time.sleep(5)
        except Exception as e_init:
            log_message(f"CRITICAL: Unexpected error during initial setup: {e_init}")
            time.sleep(10) # Wait longer before retrying on unknown critical error

    log_message("System ready. Waiting for commands from Arduino...")

    try:
        while True:
            # Handle incoming Arduino messages
            if arduino and arduino.in_waiting > 0:
                try:
                    message = arduino.readline().decode('utf-8').strip()
                    if not message: continue
                    log_message(f"Serial: Received from Arduino: '{message}'")

                    if message.startswith("TARGET_ANIMAL:"):
                        animal = message.split(":")[1].upper()
                        if animal in ["CAT", "DOG"]:
                            current_target_animal_name_from_arduino = animal
                            log_message(f"Config: Target animal set to: {current_target_animal_name_from_arduino}")
                            if arduino and arduino.is_open: # Acknowledge
                                arduino.write(f"PI_ACK_TARGET:{animal}\n".encode())
                        else:
                            log_message(f"WARN: Unknown target animal '{animal}' from Arduino.")
                            if arduino and arduino.is_open:
                                arduino.write(f"PI_ERR_UNK_TARGET:{animal}\n".encode())


                    elif message == "MOTION_DETECTED":
                        if not is_ai_ready:
                            log_message("WARN: Motion detected, but AI not ready.")
                            if arduino and arduino.is_open: arduino.write("AI_NOT_READY\n".encode())
                            continue
                        if current_target_animal_name_from_arduino is None:
                            log_message("WARN: Motion detected, but target animal not configured by Arduino.")
                            if arduino and arduino.is_open: arduino.write("NO_TARGET_CONFIGURED\n".encode())
                            continue
                        
                        if not ai_processing_active:
                            last_motion_time = time.time() # Record time of this trigger
                            run_ai_for_duration() # This function will handle AI and send result
                        else:
                            log_message("INFO: Motion re-triggered while AI active. Extending/Ignoring (currently ignores new trigger).")
                            # Potentially reset last_motion_time here if you want to extend AI run time
                            # last_motion_time = time.time() 

                    elif message == "ARDUINO_PING": # Example if Arduino sends pings
                        if arduino and arduino.is_open: arduino.write("PI_PONG\n".encode())

                except UnicodeDecodeError:
                    log_message("WARN: Serial: Received non-UTF-8 data from Arduino.")
                except serial.SerialException as se:
                    log_message(f"ERROR: Serial: Arduino disconnected or serial error: {se}. Attempting to reconnect...")
                    if arduino and arduino.is_open: arduino.close()
                    arduino = None # Flag for reconnection
                    # Reconnection attempt loop
                    while arduino is None:
                        time.sleep(5)
                        try:
                            arduino = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
                            log_message(f"Serial: Reconnected to Arduino on {SERIAL_PORT}")
                            time.sleep(2)
                            arduino.write("PI_READY\n".encode()) # Re-inform Arduino Pi is up
                            # Request current target from Arduino again after reconnect?
                            # arduino.write("REQUEST_TARGET_ANIMAL\n".encode()) # If Arduino supports this
                        except serial.SerialException:
                            log_message(f"Serial: Reconnection failed. Retrying...")
                except Exception as e:
                    log_message(f"ERROR: Processing Arduino message: {e}")
                    if arduino and arduino.is_open: arduino.write("PI_ERROR_PROCESSING\n".encode())
            
            # If Arduino disconnects and arduino object becomes None
            if arduino is None:
                log_message("WARN: Serial: Arduino appears disconnected. Attempting to reconnect in main loop...")
                time.sleep(5)
                try:
                    arduino = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
                    log_message(f"Serial: Reconnected to Arduino on {SERIAL_PORT}")
                    time.sleep(2)
                    arduino.write("PI_READY\n".encode())
                except serial.SerialException:
                    log_message(f"Serial: Reconnection failed in main loop. Will retry.")
                except Exception as e_reconnect:
                    log_message(f"ERROR: Unexpected error during reconnection attempt: {e_reconnect}")


            time.sleep(0.05) # Main loop delay

    except KeyboardInterrupt:
        log_message("Program interrupted by user.")
    except Exception as e_main:
        log_message(f"CRITICAL: An unexpected error occurred in main loop: {e_main}")
        import traceback
        traceback.print_exc()
    finally:
        log_message("Cleaning up before exit...")
        if picam2_camera and picam2_camera.started:
            log_message("AI: Stopping camera...")
            picam2_camera.stop()
        if arduino and arduino.is_open:
            arduino.write("PI_SHUTTING_DOWN\n".encode())
            time.sleep(0.1)
            arduino.close()
            log_message("Serial: Arduino connection closed.")
        log_message("Script terminated.")

if __name__ == '__main__':
    log_message("MAIN_BLOCK: __name__ == '__main__' block entered.") # <<< --- ADD THIS
    log_message("Pet Feeder Main Controller Script Starting Up...")
    log_message(f"Review Config: Model={DEFAULT_MODEL_PATH}")
    log_message(f"Review Config: Labels={DEFAULT_LABELS_PATH}")
    log_message(f"Review Config: CAT_KEYWORDS={CAT_KEYWORDS}")
    log_message(f"Review Config: DOG_KEYWORDS={DOG_KEYWORDS}")
    log_message(f"Review Config: CONFIDENCE_THRESHOLD={CONFIDENCE_THRESHOLD}")
    log_message(f"Review Config: AI_ACTIVE_DURATION_SECONDS={AI_ACTIVE_DURATION_SECONDS}")
    log_message(f"Review Config: MIN_CONSISTENT_DETECTIONS={MIN_CONSISTENT_DETECTIONS}")
    log_message("------------------------------------------------")
    log_message("MAIN_BLOCK: Calling main() function now...")
    main()
    log_message("MAIN_BLOCK: main() function has returned (should not happen in normal operation).")