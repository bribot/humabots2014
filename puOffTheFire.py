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
dyaw=0.05
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
	time.sleep(5)
	print '------------'
	tx,ty=circlesRobot()
	if ty > 0.5:
		flag=1
		print "the furtest burner is on"
	else:
		flag=0
		print "the nearest burner is on"	
	print tx
	print ty
	print '------------'
	bx,by=buttonRobot()
	print bx
	print by
	armInit()
	arm(by,bx,.45,flag)
	time.sleep(1)
	motionProxy.rest()

    
def armInit():
	# Example showing how to use positionInterpolations
	space        = motion.FRAME_ROBOT
	isAbsolute   = True

	# Motion of Arms with block process
	effectorList = ["RArm"]
	axisMaskList = [motion.AXIS_MASK_VEL]
	timeList     = [[5,10,12]]         # seconds
	pathList=[[[0.043720584362745285, -0.3127600848674774, 0.4208180904388428, 0.08055577427148819, -0.0016285453457385302, -1.291792392730713],[0.04403337836265564, -0.11809729784727097, 0.6466386318206787, 2.3986752033233643, -1.3457955121994019, -0.3654128611087799],[-0.00899630505591631, -0.11099281907081604, 0.651690661907196, -2.8810505867004395, -1.4697915315628052, 2.879744052886963]]]
	motionProxy.positionInterpolations(effectorList, space, pathList,
	axisMaskList, timeList, isAbsolute)

def arm(x,y,z,f):
	space        = motion.FRAME_ROBOT
	isAbsolute   = True

	# Motion of Arms with block process
	effectorList = ["RArm"]
	axisMaskList = [motion.AXIS_MASK_VEL]
	timeList     = [[10]]         # seconds
	pathList=[[[x, y, z, 0.0005081882700324059, 0.0033288884442299604, 0.02605888806283474]]]
	motionProxy.positionInterpolations(effectorList, space, pathList,
	axisMaskList, timeList, isAbsolute)
	
	isAbsolute   = False
	timeList     = [[1,10,15]]         # seconds
	pathList=[[[0, -0.02*f, 0, 0.0, 0.0, 0.0],[0, 0.0, -0.05, 0.0, 0.0, 0.0],[0, 0.0, 0.05, 0.0, 0.0, 0.0]]]
	motionProxy.positionInterpolations(effectorList, space, pathList,
	axisMaskList, timeList, isAbsolute)
	
	isAbsolute   = True
	timeList     = [[10,15]]
	pathList=[[[0.05, -0.3, 0.5, 0.08055577427148819, -0.0016285453457385302, -1.291792392730713],[0.04433935880661011, -0.10319031774997711, 0.22838051617145538, 1.2318189144134521, 1.1105034351348877, -0.019054466858506203]]]
	motionProxy.positionInterpolations(effectorList, space, pathList,
	axisMaskList, timeList, isAbsolute)

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
	#-------------
	minH= 0
	maxH= 20
	minS= 200
	maxS= 255
	minV= 200
	maxV= 255
	#-------------
	setHead(0.6,0.3)
	connectCamera(0)
	while(1):
		cimg = getImage(0)
		cv2.imshow('blah',cimg)
		img_bin=preparaImagen(cimg,minH,maxH,minS,maxS,minV,maxV)
		(cimg,p_x,p_y,rad)=detectaCirculo(img_bin,cimg,width,height)
		if p_x!=-1:
			if moveHead(p_x,p_y):
				x,y = calculateXY()
				#x=np.round(x)
				#y=np.round(y)
				print 'coordenada x: ', x
				print 'coordenada y: ', y
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
    #img=cv2.cvtColor(cimg,cv2.COLOR_BGR2GRAY,)
    #img_temp=img.copy()
    cimg = cv2.GaussianBlur(cimg,(11,11),0)
    img_h=cv2.cvtColor(cimg,cv2.COLOR_BGR2HSV)
    cv2.imshow("hsv",img_h)
    img_bin=umbralizaImagen(img_h,np.array((minH, minS, minV)),np.array((maxH, maxS, maxV)))
    #img_bin=cv2.medianBlur(img_bin,7)
    #img_gray=img_temp.copy()
    disk=cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(3,5))
    img_bin = cv2.dilate(img_bin,disk,iterations = 5)
    #disk=cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(3,3))
    #img_bin = cv2.erode(img_bin,disk,iterations = 10)
    #cv2.bitwise_and(img_temp,img_bin,img_temp)
    #cv2.imshow('IMG',img_temp)
    cv2.imshow("bin",img_bin)
    return img_bin

def preparaImagen2(cimg,minH,maxH,minS,maxS,minV,maxV):
    #img=cv2.cvtColor(cimg,cv2.COLOR_BGR2GRAY,)
    #img_temp=img.copy()
    cimg = cv2.GaussianBlur(cimg,(11,11),0)
    img_h=cv2.cvtColor(cimg,cv2.COLOR_BGR2HSV)
    cv2.imshow("hsv",img_h)
    img_bin=umbralizaImagen(img_h,np.array((minH, minS, minV)),np.array((maxH, maxS, maxV)))
    img_bin=cv2.medianBlur(img_bin,7)
    #img_gray=img_temp.copy()
    #disk=cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,7))
    #img_bin = cv2.dilate(img_bin,disk,iterations = 10)
    #disk=cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,5))
    #img_bin = cv2.erode(img_bin,disk,iterations = 10)
    #cv2.bitwise_and(img_temp,img_bin,img_temp)
    #cv2.imshow('IMG',img_temp)
    cv2.imshow("bin",img_bin)
    return img_bin

def detectaCirculo(img_bin,cimg,width,height):
    circles = cv2.HoughCircles(img_bin,cv.CV_HOUGH_GRADIENT,1,height/2,param1=20,param2=7,minRadius=0,maxRadius=0)
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

def buttonRobot():
	#-------------
	minH= 20
	maxH= 30
	minS= 0
	maxS= 100
	minV= 0
	maxV= 255
	#-------------
	setHead(-0.3,0.3)
	connectCamera(0)
	while(1):
		cimg = getImage(0)
		cv2.imshow('blah',cimg)
		img_bin=preparaImagen2(cimg,minH,maxH,minS,maxS,minV,maxV)
		(cimg, p1min,p1max,p2min,p2max,pymax,pymin)=detectaCuadro(img_bin,cimg)
		if p1min!=-1:
			p_x=p1min+(p1max-p1min)/2
			p_y=pymin+(pymax-pymin)/2
			#print p_x
			#print p_y
		else:
			p_x=-1
		if p_x!=-1:
			if moveHead(p_x,p_y):
				x,y = calculateXY()
				#x=np.round(x)
				#y=np.round(y)
				print 'coordenada x: ', x
				print 'coordenada y: ', y
				#tts.say('the ball is at x = %i' % x)
				#tts.say('centimeters')
				#tts.say('and y = %i' % y)
				#tts.say('centimeters from my frame of reference')
				break
		else:
			panHead()
		cv2.imshow('detected square',cimg)
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
	

def detectaCuadro(img_bin,cimg):
    #img_canny=cv2.Canny(img_bin, 190, 120)
    #cv2.imshow('Canny',img_canny) 
    #lines=cv2.HoughLines(img_bin, 1.1, np.pi,200)[0]

    lines = cv2.HoughLinesP(img_bin,1,np.pi,10, minLineLength = 50, maxLineGap = 10)
    if lines is not None:
        lines = lines[0]
        for i in range(lines.shape[0]):
         #   cv2.line(img, (lines[i][0],0), (lines[i][0],320), (i*40,255,0))
            cv2.line(cimg, (lines[i][0],lines[i][1]), (lines[i][2],lines[i][3]), (255,255,0)) 
        lines_sort=np.sort(lines, axis=0)
        lines_c=[]
        k=0
        for i in range(lines_sort.shape[0]-1):
            lx1=lines_sort[i][0]
            lx2=lines_sort[i+1][0]
            if lx2-lx1<=10:
                lines_c.append(k)
            else:
                lines_c.append(k)
                k+=1
        lines_c=np.array(lines_c)
        #print "Lines_c=", lines_c
        ind=[]
        c=np.bincount(lines_c)
        ind.append(np.argmax(c))
        c[ind[0]]=0
        ind.append(np.argmax(c))
        ind=np.sort(np.array(ind))
        p1=[]
        p2=[]
        for i in range(lines_c.shape[0]):
            if lines_c[i] == ind[0]:
                p1.append(lines_sort[i][0])
            elif lines_c[i]==ind[1]:
                p2.append(lines_sort[i][0])

        p1=np.array(p1)
        p2=np.array(p2)
        if p1.size>1: 
            p1max=p1[p1.size-1]
            p1min=p1[0]
            cv2.line(cimg, (p1[0],0), (p1[0],480), (0,255,255),2)
            cv2.line(cimg, (p1[p1.size-1],0), (p1[p1.size-1],480), (0,255,255),2)
        else:
            plmax=0
            plmin=0
        if p2.size>1: 
            p2max=p2[p2.size-1]
            p2min=p2[0]
            cv2.line(cimg, (p2[0],0), (p2[0],480), (0,255,255),2)
            cv2.line(cimg, (p2[p2.size-1],0), (p2[p2.size-1],480), (0,255,255),2)
        else:
            p2min=0
            p2max=0
        #tp1=p1[p1.size-1]-p1[0]
        #tp2=p2[p2.size-1]-p2[0]
        #print "Poste1=",tp1," Poste2= ",tp2

        #img_gray=cv2.cvtColor(img_temp,cv2.COLOR_BGR2HSV)
        #cv2.imshow('Lineas',cimg)
        pymax=np.max(lines[:,1])
        pymin=np.min(lines[:,2])
        #print pymax
        #print pymin
        return (cimg, p1min,p1max,p2min,p2max,pymax,pymin)
    else:
        return cimg,-1,-1,-1,-1,-1,-1

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
    kx = 0.1/width
    ky = -0.1/height
    dAng = [dx*kx, dy*ky]
    #print 'dx: ',dx
    #print 'dy: ',dy
    if (np.abs(dx)< 20)& (np.abs(dy)<40):
         return True
         #return False
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
    x = 0.1*np.tan(np.radians(90)-theta)*np.sin(psi)
    y = 0.1*np.tan(np.radians(90)-theta)*np.cos(psi)
    return x,y


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
        print "Camera Disconnected"
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