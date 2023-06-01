#import the GPIO and time package
import RPi.GPIO as GPIO
#import time
import RPi.GPIO as GPIO # Import Raspberry Pi GPIO library

GPIO.setwarnings(False) # Ignore warning for now
GPIO.setmode(GPIO.BOARD) # Use physical pin numbering
GPIO.setup(15, GPIO.IN, pull_up_down=GPIO.PUD_UP) #Gas pin, Set pin 10 to be an input pin and set initial value to be pulled low (off)
GPIO.setup(7, GPIO.OUT) #green LED
GPIO.setup(11, GPIO.OUT) #red
GPIO.setup(13, GPIO.OUT) #buz

import cv2
import numpy as np
import time
import serial

# from gpiozero import Servo
# from time import sleep
# 
# servo = Servo(12)
# val = -1


fire_cascade = cv2.CascadeClassifier('fire_detection.xml')

cap = cv2.VideoCapture(0)

while(True):
    _ , frame=cap.read()
    qudrant1 = frame[1: 240, 0:1020]
    cv2.imshow('Q1', qudrant1)
    gray = cv2.cvtColor(qudrant1, cv2.COLOR_BGR2GRAY)
    fire = fire_cascade.detectMultiScale(qudrant1, 1.2, 5)
    
    gray_frame = cv2.bitwise_not(gray)
    _, threshold = cv2.threshold(gray_frame, 180, 255, cv2.THRESH_BINARY)

    for (x,y,w,h) in fire:
        cv2.rectangle(qudrant1,(x-20,y-20),(x+w+20,y+h+20),(255,0,0),2)
        roi_gray = gray[y:y+h, x:x+w]
        roi_color = qudrant1[y:y+h, x:x+w]
        print("fire is detected in Q1")
        
        
        GPIO.output(7,False)
        GPIO.output(11,True)
        GPIO.output(13,True)
        
#         for i in range (10):            
#             servo.value = val
#             sleep(0.1)
#             val = val + 0.1
#             if val > 1:
#                 val = -1
        time.sleep(0.5)
        GPIO.output(11,False)
        GPIO.output(13,False)
        time.sleep(0.1)

    _ , frame2=cap.read()
    qudrant2 = frame2[240: 470, 0:1020]
    cv2.imshow('Q2', qudrant2)
    gray2 = cv2.cvtColor(qudrant2, cv2.COLOR_BGR2GRAY)
    fire2 = fire_cascade.detectMultiScale(qudrant2, 1.2, 5)
    
    gray_frame2 = cv2.bitwise_not(gray2)
    _, threshold2 = cv2.threshold(gray_frame2, 180, 255, cv2.THRESH_BINARY)

    for (x,y,w,h) in fire2:
        cv2.rectangle(qudrant2,(x-20,y-20),(x+w+20,y+h+20),(255,0,0),2)
        roi_gray2 = gray2[y:y+h, x:x+w]
        roi_color2 = qudrant2[y:y+h, x:x+w]
        print("fire is detected in Q2")

        GPIO.output(7,False)
        GPIO.output(11,True)
        GPIO.output(13,True)
        
#         for i in range (10):                        
#             servo.value = val
#             sleep(0.1)
#             val = val + 0.1
#             if val > 1:
#                 val = -1
        time.sleep(0.5)        
        GPIO.output(11,False)
        GPIO.output(13,False)
        time.sleep(0.1)

        
    #cv2.imshow('frame', frame)
    #cv2.imshow('gray', gray)
    cv2.imshow('threshold', threshold)
    
    if GPIO.input(15) == GPIO.LOW:
        print("Smoke Detected!")
        
        GPIO.output(7,False)
        GPIO.output(11,True)
        GPIO.output(13,True)
        time.sleep(0.5)
        GPIO.output(11,False)
        GPIO.output(13,False)
        time.sleep(0.1)
    else:
        print("No Smoke & No fire")
        GPIO.output(7,True)
        GPIO.output(11,False)
        GPIO.output(13,False)
        time.sleep(0.2)
    
    fired=0
        
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    
