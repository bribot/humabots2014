# -*- coding: utf-8 -*-
"""
Created on Tue Oct 28 12:18:18 2014

@author: bri
"""

from naoqi import *
import numpy as np
import cv2
import cv2.cv as cv
import time

from naoqi import ALBroker
from naoqi import ALProxy
from naoqi import ALModule

from optparse import OptionParser
from vision_definitions import *
width,height = 640,480
actuators = ['HeadYaw','HeadPitch']
maxSpeed=0.05
dyaw=0.1
nameId="app"

pip="tanyuu.local"
pport=9560

naoBroker = ALBroker("BriApp","0.0.0.0",1234,pip,pport)
tts = ALProxy('ALTextToSpeech')
motionProxy = ALProxy('ALMotion')
robotP = ALProxy('ALRobotPosture')
camProxy = ALProxy('ALVideoDevice')


def main():
	motionProxy.wakeUp()
	robotP.goToPosture("Stand",0.5)
	tts.say("I'm alive")
	ty,tx=circlesRobot()
	print tx
	print ty
	armInit()
	grab(tx,ty)
	#grab(ty,tx)
	#motionProxy.moveToward(0,0,-0.5)
	time.sleep(1)
	#c = cv2.VideoCapture(0)
    #ballx,bally=circlesRobot()
	#motionProxy.rest()

    
def armInit():
	# Example showing how to use positionInterpolations
	space        = motion.FRAME_ROBOT
	isAbsolute   = True

	# Motion of Arms with block process
	effectorList = ["RArm"]
	axisMaskList = [63]
	timeList     = [[5,15]]         # seconds
	pathList=[[[0.03204779699444771, -0.28268617391586304, 0.3796444535255432, -0.10186757147312164, 0.30915567278862, -1.143007516860962],[-0.017801012843847275, -0.08808867633342743, 0.6452454328536987, -1.6209142208099365, -1.3366206884384155, 2.8844151496887207]]]
	#pathList=[[[0.1, -0.3, 0.4, 0.0, 0.0, -1.3],[0.1, -0.3, 0.6, 2.4, -1.34, -0.36],[0.0, -0.11, 0.65, -2.88, -1.46, 2.87]]]
	motionProxy.positionInterpolations(effectorList, space, pathList,
	axisMaskList, timeList, isAbsolute)

def grab(x,y):
	motionProxy.openHand("RHand")
	# Example showing how to use positionInterpolations
	space        = motion.FRAME_ROBOT
	isAbsolute   = True

	# Motion of Arms with block process
	effectorList = ["RArm"]
	axisMaskList = [7]
	timeList     = [[5]]         # seconds
	#Calibrar tamaño de tomate (0.03)
	pathList=[[[x-0.03, y-0.03, 0.45,0,0,0]]]

	motionProxy.positionInterpolations(effectorList, space, pathList,
	axisMaskList, timeList, isAbsolute)

	print "lowering..."
	isAbsolute   = False
	timeList     = [[10]]         # seconds
	#Calibrar parametro de descenso (-0.07)
	pathList=[[[0.0, 0.0, -0.07,0,0,0]]]
	motionProxy.positionInterpolations(effectorList, space, pathList,
	axisMaskList, timeList, isAbsolute)
	
	motionProxy.closeHand("RHand")
	
	print "Got it"
	isAbsolute   = False
	timeList     = [[10,20]]         # seconds
	#Calibrar movimiento lateral en el segundo vector (0.2)
	pathList=[[[0.0, 0.0, 0.1,0,0,0],[-0.05,0.2,0.0,0.0,0.0,0.0]]]
	motionProxy.positionInterpolations(effectorList, space, pathList,
	axisMaskList, timeList, isAbsolute)
	
	motionProxy.openHand("RHand")
	print "let it go!"

def pCalculate(p1,p2,theta):
    mwidth = 48.0
    measureD =1.0
    pxwidth = float(p2-p1)
    gwidth = mwidth/(pxwidth*measureD)
    print 'rho ', gwidth
    x = gwidth*np.sin(theta)
    y = gwidth*np.cos(theta)
    return x,y

def circlesRobot():
    setHead(-0.4,0.6)
    connectCamera(1)
    #---------
    minH=0;
    maxH=20;
    minS=150;
    maxS=255;
    minV=50;
    maxV=255;
    #-----------
    while(1):
        cimg = getImage(1)
        #cimg = cv2.flip(cimg,1)
        img_bin=preparaImagen(cimg,minH,maxH,minS,maxS,minV,maxV)
        (cimg,p_x,p_y,rad)=detectaCirculo(img_bin,cimg,width,height)
        if p_x!=-1:
            if moveHead(p_x,p_y):
                x,y = calculateXY()
                #x=np.round(x)
                #y=np.round(y)
                print 'Pelota coordenada x: ', x
                print 'Pelota coordenada y: ', y
                #tts.say('the ball is at x = %i' % x)
                #tts.say('centimeters')
                #tts.say('and y = %i' % y)
                #tts.say('centimeters from my frame of reference')
                break
        else:
            panHead()
        cv2.imshow('detected circles',cimg)
        #cv2.imshow('Binaria',img_bin)
        tecla=cv2.waitKey(25)
        #print "tecla=",tecla
        if   tecla== 27:
            x=0
            y=0
            break
    cv2.imwrite('detectedC.jpg',cimg)
    cv2.destroyAllWindows()
    disconnectCamera()
    return x,y

def umbralizaImagen(hsv,min,max):
    #orange = cv2.inRange(hsv,np.array((5,70,110)),np.array((20,250,220)))
    orange = cv2.inRange(hsv,min,max)
    both=orange
    return both

def preparaImagen(cimg,minH,maxH,minS,maxS,minV,maxV):
    img=cv2.cvtColor(cimg,cv2.COLOR_BGR2GRAY,)
    img_temp=img.copy()
    img = cv2.GaussianBlur(img,(11,11),0)
    img_h=cv2.cvtColor(cimg,cv2.COLOR_BGR2HSV)
    img_bin=umbralizaImagen(img_h,np.array((minH, minS, minV)),np.array((maxH, maxS, maxV)))
    img_gray=img_temp.copy()
    disk=cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(7,7))
    erode = cv2.erode(img_bin,disk,iterations = 80)
    dilate = cv2.dilate(erode,disk,iterations = 80)
    img_bin=cv2.medianBlur(img_bin,7)
    #cv2.bitwise_and(img_temp,img_bin,img_temp)
    #cv2.imshow('IMG',img_temp)
    return img_bin

def detectaCirculo(img_bin,cimg,width,height):
    circles = cv2.HoughCircles(img_bin,cv.CV_HOUGH_GRADIENT,1,height/2,param1=200,param2=15,minRadius=0,maxRadius=0)
    max_rad=0
    k=0
    x=-1
    y=-1
    r=-1
    if circles is None:
        #print "List is empty\n",
        k=0
    else:
        circles = np.uint16(np.around(circles))
        for i in circles[0,:]:
            if i[2]>max_rad:
                max_rad=i[2]
                k=i
            else:
                # circulo
                cv2.circle(cimg,(i[0],i[1]),i[2],(0,255,0),2)
                # dibuja centro
                cv2.circle(cimg,(i[0],i[1]),2,(0,0,255),3)
        cv2.circle(cimg,(k[0],k[1]),k[2],(0, 165, 255),2)
        x=k[0]
        y=k[1]
        r=k[2]
    return (cimg,x,y,r)

def elim_dist(img, cam):
    #cam==0 cam_up, 
    if cam==0:
        fx= 770.62891039
        fy=767.76331171
        cx = 348.51844174
        cy=264.31492807
        kc = [0.33982982, -1.2359181,0.00400206,0.01930419, 1.2398553]
        kc=np.array(kc)
        cam_mat=[fx, 0, cx ],[0,fy,cy],[0, 0,1]
        cam_mat=np.array(cam_mat)
        h,  w = img.shape[:2]
        newcameramtx, roi=cv2.getOptimalNewCameraMatrix(cam_mat,kc,(w,h),1,(w,h))
        dst = cv2.undistort(img, cam_mat, kc, None, newcameramtx)
        x,y,w,h = roi
        dst = dst[y:y+h, x:x+w]
    elif cam==1:
        kc=[4.54670020e-01 , -2.35928947, -2.82117633e-03, 4.25070591e-03, 4.75239956]#OpenCV
        kc=np.array(kc)
        cam_mat=[795.80585538,0,326.03217012],[0, 796.41452639, 230.2267145],[0,0,1]#OpenCV
        cam_mat=np.array(cam_mat)
        h,  w = img.shape[:2]
        newcameramtx, roi=cv2.getOptimalNewCameraMatrix(cam_mat,kc,(w,h),1,(w,h))
        dst = cv2.undistort(img, cam_mat, kc, None, newcameramtx)
        x,y,w,h = roi
        dst = dst[y:y+h, x:x+w]
    else:
        print "Error no hay opcion de camara"
    return dst

def moveHead(ballx,bally):
    #TEST THE NEXT LINE OF CODE BEFORE IMPLEMENTATION
    motionProxy.killAll()
    actuators = ['HeadYaw','HeadPitch']
    maxSpeed=0.1
    dx = width/2 - ballx
    dy = height/2 - bally
    kx = 0.05/width
    ky = -0.05/height
    dAng = [dx*kx, dy*ky]
    #print 'dx: ',dx
    #print 'dy: ',dy
    if (np.abs(dx)< 20)& (np.abs(dy)<20):
        return True
    #currentAngle = motionProxy.getAngles('Head',True)
    motionProxy.changeAngles(actuators,dAng,maxSpeed)
    return False

    
def setHead(yaw,pitch):
    angles=[yaw,pitch]
    motionProxy.setStiffnesses('Head',1.0)
    motionProxy.setAngles(actuators,angles,maxSpeed)
    time.sleep(0.05)

def calculateXY():
    #CHECK THE REAL DISTANCE
    psi,theta = motionProxy.getAngles('Head',True)
    #x = 0.3*np.tan(np.radians(50)-theta)*np.sin(psi)
    #y = 0.3*np.tan(np.radians(50)-theta)*np.cos(psi)
    x = 0.15*np.tan(np.radians(50)-theta)*np.sin(psi)
    y = 0.15*np.tan(np.radians(50)-theta)*np.cos(psi)
    return x,y

def preparaImagenP(cimg,minH,maxH,minS,maxS,minV,maxV):
    img_h = cv2.GaussianBlur(cimg,(11,11),0)
    img_h=cv2.cvtColor(img_h,cv2.COLOR_BGR2HSV)
    img_bin=umbralizaImagen(img_h,np.array((minH, minS, minV)),np.array((maxH, maxS, maxV)))
    disk=cv2.getStructuringElement(cv2.MORPH_RECT,(21,21))
    erode = cv2.erode(img_bin,disk,iterations = 8)
    dilate = cv2.dilate(erode,disk,iterations = 8)
    #cv2.bitwise_and(img_temp,img_bin,img_temp)
    #cv2.imshow('IMG',img_temp)
    return img_bin

def panHead():
    maxSpeed=0.01
    global dyaw
    actuators = ['HeadYaw','HeadPitch']
    currentAngle=motionProxy.getAngles('Head',True)
    if currentAngle[0] > 0.6:
        dyaw=-0.1
    elif currentAngle[0] < -0.6:
        dyaw=0.1
    dAng=[dyaw,0]
    motionProxy.changeAngles(actuators,dAng,maxSpeed)
    print 'weeeee'
    time.sleep(0.5)

def connectCamera(nCamera):
    resolution = kVGA
    spaceColor = kBGRColorSpace
    fps = 30
    try:
        camProxy.subscribeCamera(nameId,nCamera,resolution,spaceColor,fps)
        camProxy.setParam(18,nCamera)
        time.sleep(1.0)
    except BaseException, err:
        print ("ERR: connectCamera: catching error: %s!" % err)

def disconnectCamera():
    try:
        camProxy.unsubscribe(nameId) 
    except BaseException, err:
        print ("ERR: disconnectCamera: catching error: %s!" % err)

def getImage(nCamera):
    try:
        i = camProxy.getImageRemote(nameId)
        if i != None:
            image = (np.reshape(np.frombuffer(i[6],dtype='%iuint8' % i[2]),(i[1],i[0],i[2])))
            elim_dist(image,nCamera)
            return image
    except BaseException, err:
        print ("ERR: getImage: catching error: %s!" % err)
    return None;


if __name__=="__main__":
    main()