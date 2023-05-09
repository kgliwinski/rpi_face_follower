import os, sys
import face_recognition
import cv2
import numpy as np
import math


img = face_recognition.load_image_file('Faces/Donald Trump.jpg')
img= cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
# kodujemy twarz, zeby moc ja porownywac do innych
img_encoding = face_recognition.face_encodings(img)[0]

img2 = face_recognition.load_image_file('Faces/Trump2.jpg')
img2 = cv2.cvtColor(img2, cv2.COLOR_BGR2RGB)
img_encoding2 = face_recognition.face_encodings(img2)[0]

faceLoc = face_recognition.face_locations(img)[0] #face_locations zwraca 4 wartoœci: góra, prawo, dó³, lewo
cv2.rectangle(img,(faceLoc[3],faceLoc[0]),(faceLoc[1],faceLoc[2]),(255,0,255),2)


result = face_recognition.compare_faces([img_encoding], img_encoding2)
faceDistance = face_recognition.face_distance([img_encoding], img_encoding2) # jak bardzo te dwie twarze sa podobne, im mniejsza wartosc, tym wieksze dopasowanie
print("Result: ", result , "Distance: ", faceDistance)
 

cv2.imshow("Img", img)
cv2.imshow("Img 2", img2)
cv2.waitKey(0)





