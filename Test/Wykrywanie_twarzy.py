from base64 import encode
import os, sys
from sre_constants import SUCCESS
from pydoc import classname
import face_recognition
import cv2
import numpy as np
import math


path = 'Faces' # Œcie¿ka do folderu ze zdjêciami
images = [] # Lista wszystkich zaimportowanych obrazów
classnames = [] # Lista imion 

myList = os.listdir(path)
print(myList)

# dodajemy po kolei obrazy do listy
for cls in myList:
    curImg = cv2.imread(f'{path}/{cls}')
    images.append(curImg)
    # Ucinamy z ka¿dej nazwy .jpg ¿eby otrzymaæ wy³¹cznie imiona i nazwiska
    classnames.append(os.path.splitext(cls)[0])
print(classnames)


# Teraz enkodujemy po kolei twarze poprzez stworzenie funkcji findEncodings
def findEncodings(images):
    encodeList = []
    for img in images:
        img= cv2.cvtColor(img, cv2.COLOR_BGR2RGB) # OpenCV standardowo korzysta z BGR i trzeba go zamieniæ na RGB
        img_encoding = face_recognition.face_encodings(img)[0]
        encodeList.append(img_encoding)
    return encodeList

encodeListKnown = findEncodings(images)
print('Enkodowanie twarzy zakonczone')


# Teraz musimy znaleŸæ twarze, które s¹ do siebie podobne. Nie mamy Ÿród³a, sk¹d weŸmiemy twarze do porównywania, ale w naszym projekcie jest to kamera

cap = cv2.VideoCapture(0)

# Pêtla while, aby dostaæ ka¿d¹ kolejn¹ klatkê
while True:
    SUCCESS, img = cap.read()
    # Porównywanie twarzy bêdzie siê odbywaæ live, wiêc trzeba zredukowaæ obliczenia poprzez zmniejszenie obrazu
    imgS = cv2.resize(img,(0,0), None, 0.25, 0.25) # 0.25 to skala pomniejszenia
    imgS= cv2.cvtColor(imgS, cv2.COLOR_BGR2RGB)


    # Na kamerce mo¿e byæ kilka twarzy, wiêc zbieramy ich lokalizacje
    facesCurFrame  = face_recognition.face_locations(imgS) # Tutaj nie dodajemy [0], bo chcemy dostaæ wszystkie lokalizacje

    # Teraz enkodujemy obraz z kamerki
    encodeCurFrame = face_recognition.face_encodings(imgS, facesCurFrame)


    # Nastêpnie porównujemy znalezione twarze do tych, które znajduj¹ siê w naszym folderze z twarzami
    for encodeFace, faceLoc in zip(encodeCurFrame, facesCurFrame):
        matches = face_recognition.compare_faces(encodeListKnown, encodeFace)
        # Poniewa¿ dajemy do face_distance teraz listê, to zwróci równie¿ listê. Bêdzie to lista dopasowañ twarzy z kamerki do tych w folderze
        faceDis = face_recognition.face_distance(encodeListKnown, encodeFace)
        print(faceDis)
        matchIndex = np.argmin(faceDis) # to zwraca nam indeks najmniejszej wartoœci dopasowania z listy twarzy, czyli najlepsze dopasowanie

        if matches[matchIndex]:
            name = classnames[matchIndex].upper()
            print(name)
            # W tym momencie mamy b³ad, poniewa¿ zmniejszyliœmy rozmiar obrazu czterokrotnie, ale patrzymy nadal tak, jakby wymiary siê nie zmieni³y
            # Trzeba teraz pomno¿yæ przez odwrotnoœæ pomniejszenia wszystkie wartoœci wspó³rzêdnych
            y1,x2,y2,x1 = faceLoc
            y1, x2, y2, x1 = y1*4, x2*4, y2*4, x1*4
            cv2.rectangle(img,(x1,y1),(x2,y2), (0,255,0), 2)
            cv2.rectangle(img,(x1,y2-35),(x2,y2), (0,255,0), cv2.FILLED)
            cv2.putText(img,name,(x1+6,y2-6),cv2.FONT_HERSHEY_COMPLEX,1,(255,255,255),2)

    cv2.imshow('Webcam',img)
    if cv2.waitKey(1) == ord('q'): # program zakoñczy dzia³anie, gdy naciœniemy klawisz 'q'
        break