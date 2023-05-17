from base64 import encode
import os
import sys
from sre_constants import SUCCESS
from pydoc import classname
import face_recognition
import cv2
import numpy as np
import math
from picamera.array import PiRGBArray
from picamera import PiCamera
import io 

path = 'Faces'
images = []
classnames = []

myList = os.listdir(path)
print(myList)


for cls in myList:
    curImg = cv2.imread(f'{path}/{cls}')
    images.append(curImg)

    classnames.append(os.path.splitext(cls)[0])
print(classnames)


def findEncodings(images):
    encodeList = []
    for img in images:
        # OpenCV standardowo korzysta z BGR i trzeba go zamieni� na RGB
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img_encoding = face_recognition.face_encodings(img)[0]
        encodeList.append(img_encoding)
    return encodeList

if __name__ == '__main__':
    encodeListKnown = findEncodings(images)
    print('Enkodowanie twarzy zakonczone')

    camera = PiCamera()
    camera.resolution = (640, 480)
    camera.framerate = 32
    rawCapture = PiRGBArray(camera, size=(640, 480))

    stream = io.BytesIO()

    for img in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        
        imgRaw = rawCapture.array
        imgS = cv2.resize(imgRaw, (0,0), None, 0.25, 0.25)
        imgS = cv2.cvtColor(imgS, cv2.COLOR_BGR2RGB)

        facesCurFrame = face_recognition.face_locations(imgRaw)

        encodeCurFrame = face_recognition.face_encodings(imgRaw, facesCurFrame)

        for encodeFace, faceLoc in zip(encodeCurFrame, facesCurFrame):
            matches = face_recognition.compare_faces(encodeListKnown, encodeFace)

            faceDis = face_recognition.face_distance(encodeListKnown, encodeFace)
            print(faceDis)
            matchIndex = np.argmin(faceDis)

            if matches[matchIndex]:
                name = classnames[matchIndex].upper()
                print(name)

                y1, x2, y2, x1 = faceLoc
                y1, x2, y2, x1 = y1*4, x2*4, y2*4, x1*4
                cv2.rectangle(imgRaw, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.rectangle(imgRaw, (x1, y2-35), (x2, y2), (0, 255, 0), cv2.FILLED)
                cv2.putText(imgRaw, name, (x1+6, y2-6),
                            cv2.FONT_HERSHEY_COMPLEX, 1, (255, 255, 255), 2)

        cv2.imshow('Webcam', imgRaw)
        if cv2.waitKey(1) == ord('q'):
            break
        rawCapture.truncate(0)
