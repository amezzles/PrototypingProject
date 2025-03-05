import time
import cv2
import numpy as np

from picamera2 import Picamera2
import tflite_runtime.interpreter as tflite

# Path to your TFLite model file
MODEL_PATH = "/home/pi/projects/cat_dog_model.tflite"

def load_labels():
    """
    For a custom model with 2 labels, you might have a 'labels.txt' containing:
      0 Cat
      1 Dog
    Return them as a list like ["Cat", "Dog"].
    """
    return ["Cat", "Dog"]

def main():
    # Initialize camera
    picam2 = Picamera2()
    # Let's create a preview (video) config or still config
    config = picam2.create_still_configuration()
    picam2.configure(config)
    picam2.start()
    time.sleep(2)  # Let camera warm up

    # Load TFLite model
    interpreter = tflite.Interpreter(model_path=MODEL_PATH)
    interpreter.allocate_tensors()

    input_details = interpreter.get_input_details()
    output_details = interpreter.get_output_details()

    # Typically, TFLite image classification models expect a certain input size, e.g. 224x224
    # Let's find the expected shape:
    input_shape = input_details[0]['shape']  # e.g., [1, 224, 224, 3]
    height = input_shape[1]
    width = input_shape[2]

    # Load label list
    labels = load_labels()

    print("Press Ctrl+C to stop...")
    while True:
        # Capture a frame from the camera
        frame = picam2.capture_array()

        # Convert BGR (OpenCV) to RGB if needed
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Resize to match model's expected input
        input_data = cv2.resize(rgb_frame, (width, height))
        # Convert to float32 and expand dims to [1, height, width, 3]
        input_data = np.expand_dims(input_data, axis=0).astype(np.float32)

        # For some models, you might need normalization, e.g. /255.0
        # input_data = input_data / 255.0

        # Run inference
        interpreter.set_tensor(input_details[0]['index'], input_data)
        interpreter.invoke()

        # Get output data
        output_data = interpreter.get_tensor(output_details[0]['index'])
        # For a simple 2-class model, output might be something like [ [0.85, 0.15] ] => 85% cat, 15% dog
        # Find the index of the highest probability
        predicted_index = np.argmax(output_data[0])
        confidence = output_data[0][predicted_index]

        label = labels[predicted_index]
        print(f"Prediction: {label} (Confidence: {confidence:.2f})")

        # Show the frame for reference (optional)
        cv2.putText(frame, f"{label} ({confidence*100:.1f}%)", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.imshow("Camera Feed", frame)

        # Press 'q' to exit loop in the display window
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    picam2.stop()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
