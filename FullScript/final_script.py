import cv2
import threading
import queue
from picamera import PiCamera
from picamera.array import PiRGBArray
import time
import serial
import serial.tools.list_ports
import numpy as np

# Set up GPIO and servo constants
SERVO_PIN_VERTICAL = 33  # GPIO pin for vertical servo
SERVO_PIN_HORIZONTAL = 32  # GPIO pin for horizontal servo
SERVO_FREQ = 50
SERVO_ANGLE_RANGE = (30., 150.)

position_queue = queue.Queue()

def pos_to_angle(pos, cur_angle, angle_range = SERVO_ANGLE_RANGE):
    # function to calculate face position to angle
    CONST = 5
    BOX_LOWER_PERC = 45.
    BOX_HIGHER_PERC = 55.
    if BOX_LOWER_PERC > pos[0] or pos[0] > BOX_HIGHER_PERC:
        if(pos[0] >= 50.):
            angle_x = cur_angle[0] + CONST
        else:
            angle_x = cur_angle[0] - CONST
    else:
        angle_x = cur_angle[0]
    if BOX_LOWER_PERC > pos[1] or pos[1] > BOX_HIGHER_PERC:
        if(pos[1] >= 50.):
            angle_y = cur_angle[1] - CONST
        else:
            angle_y = cur_angle[1] + CONST
    else:
        angle_y = cur_angle[1]
    return (angle_x, angle_y)

def show_serial_ports() -> list:
    ports = serial.tools.list_ports.comports()
    prt = []
    for i, (port, desc, hwid) in enumerate(sorted(ports)):
        print("Number {}: {} {} [{}]".format(i, port, desc, hwid))
        prt.append(port)
    return prt

# Function to detect and recognize faces
def face_recognition_fun():
    face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

    # Initialize PiCamera
    camera = PiCamera()
    camera.resolution = (640, 480)
    camera.framerate = 30
    raw_capture = PiRGBArray(camera, size=(640, 480))

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
            (x, y, w, h) = faces[0]

                # Calculate face position in percentage (0-100)
            face_x = (x + w/2) / image.shape[1] * 100
            face_y = (y + h/2) / image.shape[0] * 100
            print(face_x, face_y, len(faces))
            position_queue.put((face_x, face_y))
                    
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
    available_ports = show_serial_ports()
    # sender_num = int(input('Insert the sender serial port: '))
    sender_serial = serial.Serial(available_ports[1], 115200)
    last_angles = (90.,90.)
    sender_serial.write(bytes(str(last_angles[0]) + ";" + str(last_angles[1]) + ';\n', 'ascii'))
    while(True):
        face_xy = position_queue.get()
        print(face_xy)
        last_angles = pos_to_angle(face_xy, last_angles)
        sender_serial.write(bytes(str(last_angles[0]) + ";" + str(last_angles[1]) + ';\n', 'ascii'))

    
# Create and start the face recognition thread
face_recognition_thread = threading.Thread(target=face_recognition_fun)
face_recognition_thread.start()

# Create and start the position control thread
position_control_thread = threading.Thread(target=servomechanism_control_fun)
position_control_thread.start()