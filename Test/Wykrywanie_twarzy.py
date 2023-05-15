from base64 import encode
import os, sys
from sre_constants import SUCCESS
from pydoc import classname
import face_recognition
import cv2
import numpy as np
import math


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
        img= cv2.cvtColor(img, cv2.COLOR_BGR2RGB) # OpenCV standardowo korzysta z BGR i trzeba go zamieni� na RGB
        img_encoding = face_recognition.face_encodings(img)[0]
        encodeList.append(img_encoding)
    return encodeList

encodeListKnown = findEncodings(images)
print('Enkodowanie twarzy zakonczone')




cap = cv2.VideoCapture(0)

while True:
    SUCCESS, img = cap.read()
   
    imgS = cv2.resize(img,(0,0), None, 0.25, 0.25)
    imgS= cv2.cvtColor(imgS, cv2.COLOR_BGR2RGB)


    
    facesCurFrame  = face_recognition.face_locations(imgS) 

    
    encodeCurFrame = face_recognition.face_encodings(imgS, facesCurFrame)


    
    for encodeFace, faceLoc in zip(encodeCurFrame, facesCurFrame):
        matches = face_recognition.compare_faces(encodeListKnown, encodeFace)
        
        faceDis = face_recognition.face_distance(encodeListKnown, encodeFace)
        print(faceDis)
        matchIndex = np.argmin(faceDis) 

        if matches[matchIndex]:
            name = classnames[matchIndex].upper()
            print(name)
            
            
            y1,x2,y2,x1 = faceLoc
            y1, x2, y2, x1 = y1*4, x2*4, y2*4, x1*4
            cv2.rectangle(img,(x1,y1),(x2,y2), (0,255,0), 2)
            cv2.rectangle(img,(x1,y2-35),(x2,y2), (0,255,0), cv2.FILLED)
            cv2.putText(img,name,(x1+6,y2-6),cv2.FONT_HERSHEY_COMPLEX,1,(255,255,255),2)

    cv2.imshow('Webcam',img)
    if cv2.waitKey(1) == ord('q'): 
        break