from distance import distance 
from gpiozero import PWMOutputDevice
from gpiozero import DigitalOutputDevice
from time import sleep
import RPi.GPIO as GPIO
import time



#GPIO Mode (BOARD / BCM)
GPIO.setmode(GPIO.BCM)
 
#set GPIO Pins
GPIO_TRIGGER = 18
GPIO_ECHO = 24
 
#set GPIO direction (IN / OUT)
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)
#///////////////// Define Motor Driver GPIO Pins /////////////////
# Motor A, Left Side GPIO CONSTANTS
PWM_DRIVE_LEFT = 9		# ENA - H-Bridge enable pin
FORWARD_LEFT_PIN = 4	# IN1 - Forward Drive
REVERSE_LEFT_PIN = 17	# IN2 - Reverse Drive
# Motor B, Right Side GPIO CONSTANTS
PWM_DRIVE_RIGHT = 3		# ENB - H-Bridge enable pin
FORWARD_RIGHT_PIN = 27	# IN1 - Forward Drive
REVERSE_RIGHT_PIN = 22	# IN2 - Reverse Drive
 
# Initialise objects for H-Bridge GPIO PWM pins
# Set initial duty cycle to 0 and frequency to 1000
driveLeft = PWMOutputDevice(PWM_DRIVE_LEFT, True, 0, 1)
driveRight = PWMOutputDevice(PWM_DRIVE_RIGHT, True, 0, 1)
forwardLeft = PWMOutputDevice(FORWARD_LEFT_PIN,True,0,1)
reverseLeft = PWMOutputDevice(REVERSE_LEFT_PIN,True,0,1)
forwardRight = PWMOutputDevice(FORWARD_RIGHT_PIN,True,0,1)
reverseRight = PWMOutputDevice(REVERSE_RIGHT_PIN,True,0,1)
 


'''
# Initialise objects for H-Bridge digital GPIO pins
forwardLeft = PWMOutputDevice(FORWARD_LEFT_PIN)
reverseLeft = PWMOutputDevice(REVERSE_LEFT_PIN)
forwardRight = PWMOutputDevice(FORWARD_RIGHT_PIN)
reverseRight = PWMOutputDevice(REVERSE_RIGHT_PIN)
 '''
def allStop():
	forwardLeft.value = False
	reverseLeft.value = False
	forwardRight.value = False
	reverseRight.value = False
	driveLeft.value = 0
	driveRight.value = 0
 
def forwardDrive():
	forwardLeft.value = True
	reverseLeft.value = False
	forwardRight.value = True
	reverseRight.value = False
	driveLeft.value = 1.0
	driveRight.value = 1.0
 
def reverseDrive():
	forwardLeft.value = False
	reverseLeft.value = True
	forwardRight.value = False
	reverseRight.value = True
	driveLeft.value = 1.0
	driveRight.value = 1.0
 
def SpinLeft():
	forwardLeft.value = False
	reverseLeft.value = True
	forwardRight.value = True
	reverseRight.value = False
	driveLeft.value = 1.0
	driveRight.value = 1.0
 
def SpinRight():
	forwardLeft.value = True
	reverseLeft.value = False
	forwardRight.value = False
	reverseRight.value = True
	driveLeft.value = 1.0
	driveRight.value = 1.0
 
def forwardTurnLeft():
	forwardLeft.value = 0.3
	reverseLeft.value = False
	forwardRight.value = 1.0
	reverseRight.value = False
	driveLeft.value =1.0
	driveRight.value = 1.0
 
def forwardTurnRight():
	forwardLeft.value = 1.0
	reverseLeft.value = False
	forwardRight.value = 0.3
	reverseRight.value = False
	driveLeft.value = 1.00
	driveRight.value =1.00
 
def reverseTurnLeft():
	forwardLeft.value = False
	reverseLeft.value = True
	forwardRight.value = False
	reverseRight.value = True
	driveLeft.value = 0.2
	driveRight.value = 0.8
 
def reverseTurnRight():
	forwardLeft.value = False
	reverseLeft.value = True
	forwardRight.value = False
	reverseRight.value = True
	driveLeft.value = 0.8
	driveRight.value = 0.2
 
def checkpath():
    dd=2
    SpinLeft()
    sleep(dd)
    k=distance()
    if(k>15):
        return (-1)
    SpinRight()
    sleep(dd)
    SpinRight()
    sleep(dd)
    if(k>15):
        return 1
def main():
	allStop()
	forwardDrive()
	sleep(5)
	reverseDrive()
	sleep(5)
	spinLeft()
	sleep(5)
	SpinRight()
	sleep(5)
	forwardTurnLeft()
	sleep(5)
	forwardTurnRight()
	sleep(5)
	reverseTurnLeft()
	sleep(5)
	reverseTurnRight()
	sleep(5)
	allStop()
 
 
#main()
##################################
import cv2
import numpy as np
from line_intersection import slope
def rescale_frame(frame, percent=75):
    width = int(frame.shape[1] * percent/ 100)
    height = int(frame.shape[0] * percent/ 100)
    dim = (width, height)
    return cv2.resize(frame, dim, interpolation =cv2.INTER_AREA)


#top,bottom,left,right=500,550,390,800

top,bottom,left,right=270,350,10,470
centerx=(left+right)//2
font=cv2.FONT_HERSHEY_SIMPLEX
video = cv2.VideoCapture(0)


forwardDrive()
while True:
	print 'DISTANCE :',distance()
	if distance()<16.0:
            reverseDrive()
            sleep(4)
            k=checkpath()
            if(k==1):
                forwardDrive()
            elif(k== -1):
                forwardDrive()
            else:
                pass
                
	ret, img =video.read()
	if not ret:
            continue
        img=rescale_frame(img,percent=75)
	org=img.copy()
	#print(img.shape)
	img=cv2.flip(img,0)
	img=cv2.flip(img,1)    
        croppedimg=img[top:bottom,left:right]
        cv2.imshow('croped',croppedimg)
	img2=np.zeros(img.shape,dtype = "uint8")
	gray=cv2.cvtColor(croppedimg,cv2.COLOR_BGR2GRAY)
	cv2.rectangle(img,(left-5,top-10),(right+5,bottom+5),(0,255,0),4)

	cv2.imshow('gray',gray)
	
	edges=cv2.Canny(croppedimg,75,150)
	cv2.imshow('edges',edges)
	lines=cv2.HoughLinesP(edges,1,np.pi/180,40,maxLineGap=43)
	#img2[top:bottom,left:right]=255
	if lines is None:
            forwardDrive()
	
	else:
            line=lines[0]
            #print(len(line),line)
            #if(len(line)>3):
             #   reverseDrive()
              #  sleep(4)
            i=1
            for line1 in line:
                x1,y1,x2,y2=line1
                cv2.line(img,(x1+left,y1+top),(x2+left,y2+top),(0,0,255),3)
                print('drawn',str(i),line1)
                i=i+1
                slop=slope(x1,y1,x2,y2)
                print(slop)

                cv2.putText(img,str(slop),(100,200),font,1,(2,2,255),2,cv2.CV_AA)
                if(x1>centerx & x2 >centerx):
                    cv2.putText(img,"Turn LEFT: ",((left+right)/2,top-10),font,1,(2,2,255),3,cv2.CV_AA)
                    forwardTurnLeft()
                elif(x1<centerx and x2<centerx):
                    cv2.putText(img,"Turn RIGHT: ",((left+right)/2,top-10),font,1,(2,2,255),3,cv2.CV_AA)
                    forwardTurnRight()
                #elif(slop==0):
                 #   reverseDrive()
                elif(slop<0):
                    cv2.putText(img,"Turn Right: ",((left+right)/2,top-10),font,1,(2,2,255),3,cv2.CV_AA)
                    forwardTurnRight()
                elif(slop>0):
                    cv2.putText(img,"Turn Left: ",((left+right)/2,top-10),font,1,(2,2,255),3,cv2.CV_AA)
                    forwardTurnLeft()	
                else:
                    pass
            #print(line[0])
	cv2.imshow("S",img)
	#cv2.imshow("Sample",img2)
	
        '''
	img_and = cv2.bitwise_and(img,img2)
	#print("image and",img_and.shape)
	# define range of blue color in HSV
	lower_red = np.array([0,0,253])
	upper_red = np.array([0,0,255])
	
	#cv2.imshow("and",img_and)
	# Threshold the HSV image to get only red colors
	mask = cv2.inRange(img_and, lower_red, upper_red)
	#print(mask.shape)
	#show only and only red lines
	mask = cv2.bitwise_and(img,img,mask=mask)
	edges=cv2.Canny(mask,75,150)
	#cv2.imshow("chan3mask",edges)
	lines=cv2.HoughLinesP(edges,1,np.pi/180,30,maxLineGap=43)
	if lines is not None:
		for line in lines:
			x1,y1,x2,y2=line[0]
			cv2.line(mask,(x1,y1),(x2,y2),(0,0,255),3)
			print(line[0],)
			slop=slope(x1,y1,x2,y2)
			print(slop)

			cv2.putText(org,str(slop*slop),(100,200),font,1,(2,2,255),2,cv2.LINE_AA)
			if(slop<0):
				cv2.putText(org,"Turn Right: ",((left+right)/2,top-10),font,1,(2,2,255),3,cv2.LINE_AA)
			else:
				cv2.putText(org,"Turn LEFT: ",((left+right)/2,top-10),font,1,(2,2,255),3,cv2.LINE_AA)
			
	#plt.imshow(mask)
	#plt.show()
	cv2.rectangle(org,(left,top),(right,bottom),(0,255,0),5)
	final=cv2.addWeighted(org,0.5,mask,0.8,0)
	#mask2=mask[np.where((mask==[255]).all(axis=0))] = [200,200,200]
	#cv2.imshow("mask",chan3mask)
	#cv2.imshow("final",final)'''
        if cv2.waitKey(1) & 0xFF== ord('q'):
		break
            
video.release()
cv2.destroyAllWindows()



#####################





##end












