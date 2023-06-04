import cv2
import threading
import queue
import RPi.GPIO as GPIO
from picamera import PiCamera
from picamera.array import PiRGBArray
import time

# Set up GPIO and servo constants
SERVO_PIN_VERTICAL = 33  # GPIO pin for vertical servo
SERVO_PIN_HORIZONTAL = 32  # GPIO pin for horizontal servo
SERVO_FREQ = 50
SERVO_ANGLE_RANGE = (30., 150.)
SERVO_ANGLE_RANGE_AVG = float((SERVO_ANGLE_RANGE[0] + SERVO_ANGLE_RANGE[1]) / 2.)
SERVO_DUTY_RANGE = (5., 10.)

position_queue = queue.Queue()

def angle_to_duty_cycle(angle, angle_range = SERVO_ANGLE_RANGE, duty_range = SERVO_DUTY_RANGE):
    # Calculate the duty cycle within the duty range based on the angle within the angle range
    duty_cycle = ((duty_range[1] - duty_range[0]) * angle / (angle_range[1] - angle_range[0])) + duty_range[0]
    
    # Ensure the duty cycle is within the duty range
    duty_cycle = max(min(duty_cycle, duty_range[1]), duty_range[0])
    print("Angle, duty cycle", angle, duty_cycle)
    return duty_cycle

# Function to detect and recognize faces
def face_recognition_fun():
    face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

    # Initialize PiCamera
    camera = PiCamera()
    camera.resolution = (640, 480)
    camera.framerate = 30
    raw_capture = PiRGBArray(camera, size=(640, 480))
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(SERVO_PIN_VERTICAL, GPIO.OUT)
    GPIO.setup(SERVO_PIN_HORIZONTAL, GPIO.OUT)
    servo_vertical = GPIO.PWM(SERVO_PIN_VERTICAL, SERVO_FREQ)
    servo_horizontal = GPIO.PWM(SERVO_PIN_HORIZONTAL, SERVO_FREQ)
    servo_vertical.start(0)
    servo_horizontal.start(0)
    angle_vertical = SERVO_ANGLE_RANGE_AVG
    angle_horizontal = SERVO_ANGLE_RANGE_AVG
    servo_vertical.ChangeDutyCycle(8.)
    servo_horizontal.ChangeDutyCycle(8.)
    # Allow camera to warm up
    time.sleep(0.1)

    for frame in camera.capture_continuous(raw_capture, format='bgr', use_video_port=True):
        # Read frame from camera capture
        image = frame.array

        # Convert frame to grayscale for face detection
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Detect faces in the frame
        faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))
        # print(faces, len(faces))
        if len(faces) > 0:
        # Perform face recognition for each detected face
            for (x, y, w, h) in faces:

                # Calculate face position in percentage (0-100)
                face_x = (x + w/2)
                face_z = (y + h/2)
                print(face_x, face_z)
                
                    
        # Display the video frame with face rectangles
        cv2.imshow('Video', image)
        cv2.waitKey(1)

        # Clear the stream in preparation for the next frame
        raw_capture.truncate(0)

        # Exit the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the camera and destroy the windows
    camera.close()
    cv2.destroyAllWindows()

# Function to control the servomechanisms based on face position
def servomechanism_control_fun():
    # Set up GPIO
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(SERVO_PIN_VERTICAL, GPIO.OUT)
    GPIO.setup(SERVO_PIN_HORIZONTAL, GPIO.OUT)
    servo_vertical = GPIO.PWM(SERVO_PIN_VERTICAL, SERVO_FREQ)
    servo_horizontal = GPIO.PWM(SERVO_PIN_HORIZONTAL, SERVO_FREQ)
    servo_vertical.start(0)
    servo_horizontal.start(0)
    angle_vertical = SERVO_ANGLE_RANGE_AVG
    angle_horizontal = SERVO_ANGLE_RANGE_AVG
    servo_vertical.ChangeDutyCycle(8.)
    servo_horizontal.ChangeDutyCycle(8.)
    while True:
        # Wait for a face position to be available
        face_position = position_queue.get(block=True, timeout = 5.)
        angle_vertical = SERVO_ANGLE_RANGE_AVG* 2 * (100. - face_position[1])
        angle_horizontal = SERVO_ANGLE_RANGE_AVG *2* (100. - face_position[0])
        # Apply duty cycle to the vertical servo
        servo_vertical.ChangeDutyCycle(angle_to_duty_cycle(angle_vertical))
        servo_horizontal.ChangeDutyCycle(angle_to_duty_cycle(angle_horizontal))

# Create and start the face recognition thread
face_recognition_thread = threading.Thread(target=face_recognition_fun)
face_recognition_thread.start()

# Create and start the position control thread
# position_control_thread = threading.Thread(target=servomechanism_control_fun)
# position_control_thread.start()