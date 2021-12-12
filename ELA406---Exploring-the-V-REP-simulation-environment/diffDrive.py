# -*- coding: utf-8 -*-
"""
Created on Tue Jul 17 14:14:21 2018

@author: Ulrik Åkesson
"""

import vrep, time, sys, array
from PIL import Image as I
from matplotlib import pyplot as plt
import numpy as np

l = 0.22 # Distance between the wheels
r = 0.05 # Radius of the wheels
f2w = 0.125 # Distance from front of robot to front of the wheel
L = 0.2 # Length of the robot
maxSteeringAngle = 75*np.pi/180
minSteeringAngle = -1*maxSteeringAngle

dt = 0.05

def diffDrive(clientID):
    
    
    
    
    visionLeft = [-1, -1, -1]
    visionRight = [-1, -1, -1]
    
    imagesLeft = [-1, -1, -1]
    imagesRight = [-1, -1, -1]
    
    detectionLeft = [-1, -1, -1]
    detectionRight = [-1, -1, -1]
    
    usFront = [-1, -1]
    objectFront = [-1, -1]
    distanceFront = [-1, -1]
    
    usLeft = [-1, -1]
    usRight = [-1, -1]


    desiredVelocity = 0 #m/s??!!
    desiredDirection = 0
    leftVelocity = (2*desiredVelocity-desiredDirection*l)/(2*r)
    rightVelocity = (2*desiredVelocity+desiredDirection*l)/(2*r)
    
    error = 0
    prevError = 0
    integral = 0
    
    KpSteering = 5
    KiSteering = 0
    KdSteering = 0
    
    KpSpeed = 1.2
    KiSpeed = 0
    KdSpeed = 0
    
    maxVelocity = 2
    minVelocity = 0.10
    maxE = 0.07
    

    

    
# =============================================================================
#     Initilize connection to motors and visions
# =============================================================================
    returnCode, motorLeft = vrep.simxGetObjectHandle(clientID, 'leftMotor', vrep.simx_opmode_blocking)
    if returnCode != vrep.simx_return_ok:
        sys.exit('Could not get handle to the left motor')
    returnCode, motorRight = vrep.simxGetObjectHandle(clientID, 'rightMotor', vrep.simx_opmode_blocking)
    if returnCode != vrep.simx_return_ok:
        sys.exit('Could not get handle to the right motor')
        
    returnCode, visionMiddle = vrep.simxGetObjectHandle(clientID, 'visionMiddle', vrep.simx_opmode_blocking)
    if returnCode != vrep.simx_return_ok:
        sys.exit('Could not get handle to the middle vision')
        
    for i in range(3):
        returnCode, visionLeft[i] = vrep.simxGetObjectHandle(clientID, 'visionLeft%d' % (i), vrep.simx_opmode_blocking)
        if returnCode != vrep.simx_return_ok:
            sys.exit('Could not get handle to the left vision %d' %(i))
        returnCode, visionRight[i] = vrep.simxGetObjectHandle(clientID, 'visionRight%d' % (i), vrep.simx_opmode_blocking)
        if returnCode != vrep.simx_return_ok:
            sys.exit('Could not get handle to the right vision %d' %(i))
        
    for i in range(2):  
        returnCode, usFront[i] = vrep.simxGetObjectHandle(clientID, 'distanceFront%d' % (i), vrep.simx_opmode_blocking)
        if returnCode != vrep.simx_return_ok:
            sys.exit('Could not get handle to front us %d' %(i))
        returnCode, usLeft[i] = vrep.simxGetObjectHandle(clientID, 'distanceLeft%d' % (i), vrep.simx_opmode_blocking)
        if returnCode != vrep.simx_return_ok:
            sys.exit('Could not get handle to the left us')
        returnCode, usRight[i] = vrep.simxGetObjectHandle(clientID, 'distanceRight%d' % (i), vrep.simx_opmode_blocking)
        if returnCode != vrep.simx_return_ok:
            sys.exit('Could not get handle to the right us')
        
# =============================================================================
# Set initial velocity of motors
# =============================================================================
    returnCode = vrep.simxSetJointTargetVelocity(clientID, motorLeft, leftVelocity, vrep.simx_opmode_streaming)
    returnCode = vrep.simxSetJointTargetVelocity(clientID, motorRight, rightVelocity, vrep.simx_opmode_streaming)
# =============================================================================
#   Initialize video and ultrasonic stream
# =============================================================================
   # =============================================================================
#     Initialize video stream
# =============================================================================
    returnCode, resolution, imageMiddle = vrep.simxGetVisionSensorImage(clientID, visionMiddle, 1, vrep.simx_opmode_streaming)
    for i in range(3):
        returnCode, resolution, imagesLeft[i] = vrep.simxGetVisionSensorImage(clientID, visionLeft[i], 1, vrep.simx_opmode_streaming)
        returnCode, resolution, imagesRight[i] = vrep.simxGetVisionSensorImage(clientID, visionRight[i], 1, vrep.simx_opmode_streaming)
# =============================================================================
#     Initialize proximity stream
# =============================================================================
    for i in range(2):
        returnCode, detectionStateForward, detectedPointForward, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(clientID, usFront[i], vrep.simx_opmode_streaming)
        returnCode, detectionStateLeft, detectedPointLeft, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(clientID, usLeft[i], vrep.simx_opmode_streaming)
        returnCode, detectionStateRight, detectedPointRight, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(clientID, usRight[i], vrep.simx_opmode_streaming)
    time.sleep(1) # Allow vrep som time to start streaming data
    while vrep.simxGetConnectionId(clientID) != -1:
# =============================================================================
#     Check the proximity sensors to decide the behaviour 
# =============================================================================
        for i in range(2):
            objectFront[i], distanceFront[i] = detectObject(clientID, usFront[i])

        if objectFront[0] or objectFront[1]:
# =============================================================================
#             Run obstacle avoidance behaviour
# =============================================================================
            maxVelocity = 0.5
            # Detect which of the fron sensors that got triggered, used to measure distance
            if objectFront[0]:
                i = 0
            else:
                i = 1
            if desiredDirection == 0:
                desiredDirection = 1
            # Figure out where the robot came from, since that area must be free from obstacles
            prevDirection = 2*np.absolute(desiredDirection)/(-1*desiredDirection)
            desiredDirection = prevDirection*np.arctan((f2w+distanceFront[i])/(l/2))
            desiredDirection = np.arctan2(np.sin(desiredDirection), np.cos(desiredDirection))
            desiredDirection = checkAngele(desiredDirection)
            # Turn away from obstacle
            prevDirection = desiredDirection
            desiredVelocity = 0
# =============================================================================
#             print("Turn away from obstacle")
# =============================================================================
            setVelocity(clientID, desiredVelocity, desiredDirection, motorLeft, motorRight, maxVelocity)
            
            # Turn until corner sensor detects the obstacle
            if (prevDirection/np.absolute(prevDirection)) == -1:
                objectTurn, distanceTurn = detectObject(clientID, usLeft[0])
            elif (prevDirection/np.absolute(prevDirection)) == 1:
                objectTurn, distanceTurn = detectObject(clientID, usRight[0])
            while not objectTurn:
                if (prevDirection/np.absolute(prevDirection)) == -1:
                    objectTurn, distanceTurn = detectObject(clientID, usLeft[0])
                elif (prevDirection/np.absolute(prevDirection)) == 1:
                    objectTurn, distanceTurn = detectObject(clientID, usRight[0])
            
            # Drive until edge sensor detects the obstacle
            desiredDirection = 0
            desiredVelocity = 1
# =============================================================================
#             print("Drive until edge sensor detects the obstacle")
# =============================================================================
            setVelocity(clientID, desiredVelocity, desiredDirection, motorLeft, motorRight)
            
            objectLeft, distanceLeft = detectObject(clientID, usLeft[1])
            objectRight, distanceRight = detectObject(clientID, usRight[1])
            while not objectLeft and not objectRight:
                objectLeft, distanceLeft = detectObject(clientID, usLeft[1])
                objectRight, distanceRight = detectObject(clientID, usRight[1])
            
            # Turn in the oposite direction until corner sensor detects the obstacle
            desiredDirection = -prevDirection
            desiredVelocity = 0
# =============================================================================
#             print("Turn in the oposite direction until corner sensor detects the obstacle")
# =============================================================================
            setVelocity(clientID, desiredVelocity, desiredDirection, motorLeft, motorRight, maxVelocity)
            if (prevDirection/np.absolute(prevDirection)) == -1:
                objectTurn, distanceTurn = detectObject(clientID, usLeft[0])
            elif (prevDirection/np.absolute(prevDirection)) == 1:
                objectTurn, distanceTurn = detectObject(clientID, usRight[0])
            while not objectTurn:
                if (prevDirection/np.absolute(prevDirection)) == -1:
                    objectTurn, distanceTurn = detectObject(clientID, usLeft[0])
                elif (prevDirection/np.absolute(prevDirection)) == 1:
                    objectTurn, distanceTurn = detectObject(clientID, usRight[0])
                    
             # Drive until edge sensor detects the obstacle
            desiredDirection = 0
            desiredVelocity = 1
# =============================================================================
#             print("Drive until edge sensor detects the obstacle")
# =============================================================================
            setVelocity(clientID, desiredVelocity, desiredDirection, motorLeft, motorRight)
            objectLeft, distanceLeft = detectObject(clientID, usLeft[1])
            objectRight, distanceRight = detectObject(clientID, usRight[1])
            while not objectLeft and not objectRight:
                objectLeft, distanceLeft = detectObject(clientID, usLeft[1])
                objectRight, distanceRight = detectObject(clientID, usRight[1])
                
            # Continue forward untill the obstacle is cleared
            while objectLeft or objectRight:
                objectLeft, distanceLeft = detectObject(clientID, usLeft[1])
                objectRight, distanceRight = detectObject(clientID, usRight[1])
            
             # Turn towards and drive until line is detected
            desiredDirection = -prevDirection
            desiredVelocity = 0.5
# =============================================================================
#             print("Turn towards and drive until line is detected")
# =============================================================================
            setVelocity(clientID, desiredVelocity, desiredDirection, motorLeft, motorRight)
            detectionMiddle = detectLine(clientID, visionMiddle)
            for i in range(3):
                detectionLeft[i] = detectLine(clientID, visionLeft[i])
                detectionRight[i] = detectLine(clientID, visionRight[i])
            while (not detectionMiddle and not detectionLeft[0] and not detectionLeft[1]
                   and not detectionLeft[2] and not detectionRight[0] and not detectionRight[1] and not detectionRight[2]):
                setVelocity(clientID, desiredVelocity, desiredDirection, motorLeft, motorRight)
# =============================================================================
#                 print(desiredDirection)
# =============================================================================
                detectionMiddle = detectLine(clientID, visionMiddle)
                for i in range(3):
                    detectionLeft[i] = detectLine(clientID, visionLeft[i])
                    detectionRight[i] = detectLine(clientID, visionRight[i])
                

            

            
        else: 
# =============================================================================
#             Run follow line behaviour
# =============================================================================
            desiredVelocity = 1
            detectionMiddle = detectLine(clientID, visionMiddle)
            for i in range(3):
                detectionLeft[i] = detectLine(clientID, visionLeft[i])
                detectionRight[i] = detectLine(clientID, visionRight[i])
# =============================================================================
#         Determine the error based on the sensor values
# =============================================================================
            if detectionMiddle:
                error = 0
            if detectionLeft[0]:
                error = (-7/3)*pow(10,-2)
            if detectionRight[0]:
                error = (7/3)*pow(10,-2)
            if detectionLeft[1]:
                error = (-14/3)*pow(10,-2)
            if detectionRight[1]:
                error = (14/3)*pow(10,-2)
            if detectionLeft[2]:
                error = -7*pow(10,-2)
            if detectionRight[2]:
                error = 7*pow(10,-2)
                
            integral = integral+error*dt
            de = error-prevError
            derivative = (de)/dt


# =============================================================================
#       PID controller
# =============================================================================
            desiredDirection = KpSteering*error+KiSteering*integral+KdSteering*derivative
            desiredDirection = np.arctan2(np.sin(desiredDirection), np.cos(desiredDirection))
            desiredDirection = checkAngele(desiredDirection)
            
# =============================================================================
#             print("Desired direction", (desiredDirection*180)/np.pi)    
# =============================================================================
            KpSpeed = (maxVelocity-minVelocity)/maxE
            desiredVelocity = maxVelocity-np.absolute(KpSpeed*desiredDirection)
            if desiredVelocity < minVelocity:
                desiredVelocity = minVelocity
# =============================================================================
#             print("Desired velocity", desiredVelocity)    
# =============================================================================
            prevError = error
            
            setVelocity(clientID, desiredVelocity, desiredDirection, motorLeft, motorRight)
            
            
    
    print('End of simulation')
    
    
def checkAngele(desiredDirection):
    if desiredDirection > maxSteeringAngle:
        desiredDirection = maxSteeringAngle
    elif desiredDirection < minSteeringAngle:
        desiredDirection = minSteeringAngle
    return desiredDirection

def setVelocity(clientID, desiredVelocity, desiredDirection, motorLeft, motorRight, maxVelocity = 5):
    leftVelocity = (2*desiredVelocity-desiredDirection*l)/(2*r)
    rightVelocity = (2*desiredVelocity+desiredDirection*l)/(2*r)
# =============================================================================
#     print("Vl = {}, Vr = {}".format(leftVelocity, rightVelocity))
# =============================================================================
    
    if np.absolute(leftVelocity) > np.absolute(rightVelocity):
        ratio = rightVelocity/leftVelocity
        if np.absolute(leftVelocity) > maxVelocity:
            leftVelocity = (leftVelocity/np.absolute(leftVelocity))*maxVelocity
            rightVelocity = ratio*(leftVelocity/np.absolute(leftVelocity))*maxVelocity
    elif np.absolute(rightVelocity) > np.absolute(leftVelocity):
        ratio = leftVelocity/rightVelocity
        if np.absolute(rightVelocity) > maxVelocity:
            rightVelocity = (rightVelocity/np.absolute(rightVelocity))*maxVelocity
            leftVelocity = ratio*(rightVelocity/np.absolute(rightVelocity))*maxVelocity
    else:
        if np.absolute(leftVelocity) > maxVelocity:
            leftVelocity = (leftVelocity/np.absolute(leftVelocity))*maxVelocity
        if np.absolute(rightVelocity) > maxVelocity:
            rightVelocity = (rightVelocity/np.absolute(rightVelocity))*maxVelocity
# =============================================================================
#     print("Vl = {}, Vr = {}".format(leftVelocity, rightVelocity))
# =============================================================================
    
    returnCode = vrep.simxSetJointTargetVelocity(clientID, motorLeft, leftVelocity, vrep.simx_opmode_streaming)
    if returnCode != vrep.simx_return_ok:
        sys.exit('Could not set velocity')
    returnCode = vrep.simxSetJointTargetVelocity(clientID, motorRight, rightVelocity, vrep.simx_opmode_streaming)
    if returnCode != vrep.simx_return_ok:
        sys.exit('Could not set velocity')
        
        
def detectLine(clientID, visionHandle):
    returnCode, resolution, image = vrep.simxGetVisionSensorImage(clientID, visionHandle, 1, vrep.simx_opmode_buffer)
    if returnCode == vrep.simx_return_ok:
        im = np.array(image, dtype=np.uint8)
        return im < 30
    elif returnCode == vrep.simx_return_novalue_flag:
        print('No data yet')
        return False
    else:
        sys.exit('Could not get image')
        
def detectObject(clientID, visionHandle):
    returnCode, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(clientID, visionHandle, vrep.simx_opmode_buffer)
    if returnCode == vrep.simx_return_ok:
        return detectionState, detectedPoint[2]
    elif returnCode == vrep.simx_return_novalue_flag:
        print('No value distance value yet')
        return False, -1
    else:
        sys.exit('Could not get distance reading')
    
if __name__ == '__main__':
# =============================================================================
#   Open connection to V-REP
# =============================================================================
    vrep.simxFinish(-1)
    clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5)
    if clientID!=-1:
        print('Connected to remote API server')
        diffDrive(clientID)

    else:
        print('Connection non successful')
        sys.exit('Could not connect')