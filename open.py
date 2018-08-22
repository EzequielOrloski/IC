from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
import RPi.GPIO as GPIO
import time

def sensor():
    GPIO.output(trigger, True)
    time.sleep(0.00001)
    GPIO.output(trigger, False)
    start = time.time()
    while GPIO.input(echo) == 0:
        start = time.time()
                
    while GPIO.input(echo) == 1:
        stop = time.time()
                
    dif = stop - start
    distancia = (dif * 34300)/2
    if distancia < 15.0:
        GPIO.output(pare, True)
    print("Distancia: %.1f cm" % distancia)

def detect():
    camera = PiCamera()
    camera.resolution = (640, 480)
    camera.framerate = 15
    rawCapture = PiRGBArray(camera, size=(640, 480))
    display_window = cv2.namedWindow("ais")
    face_cascade = cv2.CascadeClassifier('as_de_copas.xml')
    time.sleep(1)
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        #sensor()
        GPIO.output(pare, False)
        image = frame.array
        gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        stop = face_cascade.detectMultiScale(
            gray,
            scaleFactor=1.1,
            minNeighbors=5,
            minSize=(100,100)
        )
        for (x,y,w,h) in stop:
            cv2.rectangle(image,(x,y),(x+w,y+h),(0,255,0),2)
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(image, "ais de copa",(x,y),font, 0.5,(11,255,255),2,cv2.LINE_AA)
            GPIO.output(acende, True)
        GPIO.output(acende, False)
        cv2.imshow("ais", image)
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
    detect()
    
    
