from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
import numpy as np
import RPi.GPIO as GPIO
import time
def imagem(img):
    #retorno < 0: direita
    #        > 0: esquerda
    #        = 0: para
    height = np.size(img, 0)  #obtem altura da imagem
    width  = np.size(img, 1)  #obtem largura da imagem
    qntCotorno = 0
    direcao    = 0
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (21,21), 0)
    frameBinarizado = cv2.threshold(gray, limiarBinarizacao, 255, cv2.THRESH_BINARY)[1]
    frameBinarizado = cv2.dilate(frameBinarizado, None, iterations=2)
    frameBinarizado = cv2.bitwise_not(frameBinarizado)

    _, cnts, _ = cv2.findContours(frameBinarizado.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(img, cnts, -1, (255,0,255), 3)
    for c in cnts:
        if cv2.contourArea(c) < areaContorno:
            continue
        
        qntCotorno = qntCotorno + 1
        (x,y,w,h) = cv2.boundingRect(c)
        cv2.rectangle(img, (x,y), (x + w, y + h), (0,255,0), 2)
        cordx = int((x + x + w)/2)
        cordy = int((y + y + h)/2)
        centro = (cordx, cordy)
        cv2.circle(img, centro, 1, (0,0,0), 5)
        direcao = cordx - (width/2)
        
    cv2.line(img, (int(width/2), 0), (int(width/2), height), (255,0,0), 2)
    if qntCotorno > 0:
        cv2.line(img, centro, (int(width/2), cordy), (0,255,0), 1)
    
    
    cv2.imshow('rota', img)
    cv2.waitKey(10)
    return direcao, qntCotorno
        
    
def detect():
    camera = PiCamera()
    camera.resolution = (320, 240)
    camera.framerate = 15
    rawCapture = PiRGBArray(camera, size=(320, 240))
    display_window = cv2.namedWindow("Pare")
    face_cascade = cv2.CascadeClassifier('pare.xml')
    time.sleep(1)
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        image = frame.array
        try:
            direction, qntLinhas = imagem(image)
            if qntLinhas == 0:
                print("parar")
                continue
            
            if direction > 0:
                print("direita  "+str(abs(direction))+"  pixels")
                    
            if direction < 0:
                print("esquerda   "+str(abs(direction))+"  pixels")  
            
            if direction == 0:
                print("na linha")
            
        except(KeyboardInterrupt):
            print("paro tudo")
            
        GPIO.output(pare, False)
        gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        stop = face_cascade.detectMultiScale(
            gray,
            scaleFactor=1.1,
            minNeighbors=5,
            minSize=(30,30)
        )
        for (x,y,w,h) in stop:
            cv2.rectangle(image,(x,y),(x+w,y+h),(0,255,0),2)
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(image, "stop", (x,y),font, 0.5,(11,255,255),2,cv2.LINE_AA)
            GPIO.output(acende, True)
        GPIO.output(acende, False)
        cv2.imshow("STOP", image)
        key = cv2.waitKey(1)
        rawCapture.truncate(0)
        if key == 27:
            camera.close()
            cv2.destroyAllWindows()
            GPIO.cleanup()
            break
        
if __name__ == '__main__':
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    trigger = 23
    echo    = 24
    GPIO.setup(trigger, GPIO.OUT)
    GPIO.setup(echo, GPIO.IN)
    GPIO.output(trigger, False)
    acende = 17
    pare = 27
    GPIO.setup(acende, GPIO.OUT)
    GPIO.setup(pare, GPIO.OUT)
    GPIO.output(acende, False)
    GPIO.output(pare, False)
    time.sleep(0.1)
    limiarBinarizacao = 125
    areaContorno = 5000
    detect()
    
    

