from base64 import encode
import os, sys
from sre_constants import SUCCESS
from pydoc import classname
import face_recognition
import cv2
import numpy as np
import math


path = 'Faces' # �cie�ka do folderu ze zdj�ciami
images = [] # Lista wszystkich zaimportowanych obraz�w
classnames = [] # Lista imion 

myList = os.listdir(path)
print(myList)

# dodajemy po kolei obrazy do listy
for cls in myList:
    curImg = cv2.imread(f'{path}/{cls}')
    images.append(curImg)
    # Ucinamy z ka�dej nazwy .jpg �eby otrzyma� wy��cznie imiona i nazwiska
    classnames.append(os.path.splitext(cls)[0])
print(classnames)


# Teraz enkodujemy po kolei twarze poprzez stworzenie funkcji findEncodings
def findEncodings(images):
    encodeList = []
    for img in images:
        img= cv2.cvtColor(img, cv2.COLOR_BGR2RGB) # OpenCV standardowo korzysta z BGR i trzeba go zamieni� na RGB
        img_encoding = face_recognition.face_encodings(img)[0]
        encodeList.append(img_encoding)
    return encodeList

encodeListKnown = findEncodings(images)
print('Enkodowanie twarzy zakonczone')


# Teraz musimy znale�� twarze, kt�re s� do siebie podobne. Nie mamy �r�d�a, sk�d we�miemy twarze do por�wnywania, ale w naszym projekcie jest to kamera

cap = cv2.VideoCapture(0)

# P�tla while, aby dosta� ka�d� kolejn� klatk�
while True:
    SUCCESS, img = cap.read()
    # Por�wnywanie twarzy b�dzie si� odbywa� live, wi�c trzeba zredukowa� obliczenia poprzez zmniejszenie obrazu
    imgS = cv2.resize(img,(0,0), None, 0.25, 0.25) # 0.25 to skala pomniejszenia
    imgS= cv2.cvtColor(imgS, cv2.COLOR_BGR2RGB)


    # Na kamerce mo�e by� kilka twarzy, wi�c zbieramy ich lokalizacje
    facesCurFrame  = face_recognition.face_locations(imgS) # Tutaj nie dodajemy [0], bo chcemy dosta� wszystkie lokalizacje

    # Teraz enkodujemy obraz z kamerki
    encodeCurFrame = face_recognition.face_encodings(imgS, facesCurFrame)


    # Nast�pnie por�wnujemy znalezione twarze do tych, kt�re znajduj� si� w naszym folderze z twarzami
    for encodeFace, faceLoc in zip(encodeCurFrame, facesCurFrame):
        matches = face_recognition.compare_faces(encodeListKnown, encodeFace)
        # Poniewa� dajemy do face_distance teraz list�, to zwr�ci r�wnie� list�. B�dzie to lista dopasowa� twarzy z kamerki do tych w folderze
        faceDis = face_recognition.face_distance(encodeListKnown, encodeFace)
        print(faceDis)
        matchIndex = np.argmin(faceDis) # to zwraca nam indeks najmniejszej warto�ci dopasowania z listy twarzy, czyli najlepsze dopasowanie

        if matches[matchIndex]:
            name = classnames[matchIndex].upper()
            print(name)
            # W tym momencie mamy b�ad, poniewa� zmniejszyli�my rozmiar obrazu czterokrotnie, ale patrzymy nadal tak, jakby wymiary si� nie zmieni�y
            # Trzeba teraz pomno�y� przez odwrotno�� pomniejszenia wszystkie warto�ci wsp�rz�dnych
            y1,x2,y2,x1 = faceLoc
            y1, x2, y2, x1 = y1*4, x2*4, y2*4, x1*4
            cv2.rectangle(img,(x1,y1),(x2,y2), (0,255,0), 2)
            cv2.rectangle(img,(x1,y2-35),(x2,y2), (0,255,0), cv2.FILLED)
            cv2.putText(img,name,(x1+6,y2-6),cv2.FONT_HERSHEY_COMPLEX,1,(255,255,255),2)

    cv2.imshow('Webcam',img)
    if cv2.waitKey(1) == ord('q'): # program zako�czy dzia�anie, gdy naci�niemy klawisz 'q'
        break