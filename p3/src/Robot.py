#!/usr/bin/python
# -*- coding: UTF-8 -*-
from __future__ import print_function # use python 3 syntax but make it compatible with python 2
from __future__ import division       #                           ''

import brickpi3 # import the BrickPi3 drivers
import csv      # write csv
import datetime # timestamps
import time     # import the time library for the sleep function
import sys
import numpy as np
import cv2
import picamera
from picamera.array import PiRGBArray
from geometry import Vector3, Transform

# tambien se podria utilizar el paquete de threading
from multiprocessing import Process, Value, Array, Lock

class Robot:
    def __init__(self, init_position=[0.0, 0.0, 0.0], resolution=(320, 240), framerate=32):
        """
        Initialize basic robot params. \

        Initialize Motors and Sensors according to the set up in your robot
        """
        # Robot construction parameters
        self.R = 2.8   # Radio de las ruedas (uds en cm)
        self.L = 15.25 # Longitud del eje de las ruedas (uds en cm)
        # self. ...
        ##################################################
        # Motors and sensors setup
        # Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.
        self.BP = brickpi3.BrickPi3()
        # Configure sensors, for example a touch sensor.                                              
        self.BP.set_sensor_type(self.BP.PORT_1, self.BP.SENSOR_TYPE.EV3_GYRO_ABS_DPS)
        #self.BP.set_sensor_type(self.BP.PORT_1, self.BP.SENSOR_TYPE.TOUCH)
        self.PORT_LEFT_MOTOR  = self.BP.PORT_D
        self.PORT_RIGHT_MOTOR = self.BP.PORT_A
        # Reset encoder B and C (or all the motors you are using)
        self.BP.offset_motor_encoder(self.PORT_RIGHT_MOTOR, self.BP.get_motor_encoder(self.PORT_RIGHT_MOTOR))
        self.BP.offset_motor_encoder(self.PORT_LEFT_MOTOR, self.BP.get_motor_encoder(self.PORT_LEFT_MOTOR))
        ##################################################
        # Odometry shared memory values
        self.x  = Value('d', init_position[0]) # Robot X coordinate.
        self.y  = Value('d', init_position[1]) # Robot Y coordinate.
        self.th = Value('d', init_position[2]) # Robot orientation
        self.sD = Value('i', 0)                # Latest stored RIGHT encoder value.
        self.sI = Value('i', 0)                # Latest stored LEFT encoder value.
        self.sC = Value('i', 0)                # Latest stored BASKET encoder value.
        self.v  = Value('d', 0.0)              # Robot linear theorical speed (or tangential if rotating).
        self.w  = Value('d', 0.0)              # Robot angular theorical speed.
        self.wi = Value('d', 0.0)              # Robot left wheel theorical speed.
        self.wd = Value('d', 0.0)              # Robot right wheel theorical speed.
        self.finished = Value('b', 1)          # Boolean to show if odometry updates are finished
        # Se pueden borrar alegremente cuando ya no se necesiten o se mejoren (son de prueba)
        self.dir     = [1, 0]                  # Orientacion original del Robot.
        self.ndir_x  = Value('d', 0.0)         # Orientacion actual (coordenada x).
        self.ndir_y  = Value('d', 0.0)         # Orientacion actual (coordenada y).
        self.tLength = Value('d',0.0)          # Total recorrido por el robot (linealmente).
        # If we want to block several instructions to be run together, we may want to use an explicit Lock
        self.lock_odometry = Lock()
        # Odometry update period
        self.P = 0.01                          # in seconds
        # Camera
        self.resolution = resolution           # Resolucion de la camara
        self.center     = Vector2(resolution[0]//2, resolution[1]//2)
                                               # Centro de la camara (en base a la resolucion en pixeles)
        self.framerate  = framerate            # Framerate

    def setSpeed(self, v: float, w: float):
        """
        Set the new robot speed. \
        
        - self: The robot itself.
        - v:    Robot linear speed.
        - w:    Robot angular speed.
        """
        # Calculate motors angular speeds 
        A        = np.array([[1/self.R, self.L/(2*self.R)], [1/self.R, -self.L/(2*self.R)]])
        vc       = np.array([[v],[w]])
        w_motors = np.dot(A, vc) # Wheels angular speed: [wd, wi]
        #print("Velocidad: %.5f, Velocidad Angular: %.5f" %(v, w))
        #print("Rueda derecha (Init): %.2f, Rueda izquierda (Init): %.2f" %(w_motors[0], w_motors[1]))

        # Set each motor speed
        self.BP.set_motor_dps(self.PORT_RIGHT_MOTOR, np.rad2deg(w_motors[0])) 
        self.BP.set_motor_dps(self.PORT_LEFT_MOTOR,  np.rad2deg(w_motors[1]))

        # Store speed data (really necessary?)
        # Realmente, podria ser necesario a la hora de corregir la velocidad, pero 
        # lo usaremos mas adelante.
        self.lock_odometry.acquire()
        self.v.value = v
        self.w.value = w
        self.lock_odometry.release()


    def readSpeed(self):
        """
        Get the robot real speed.
        """
        try:
            # Each of the following BP.get_motor_encoder functions returns the encoder value
            # (what we want to store).
            #sys.stdout.write("Reading encoder values .... \n")
            left_encoder  = self.BP.get_motor_encoder(self.PORT_LEFT_MOTOR)
            right_encoder = self.BP.get_motor_encoder(self.PORT_RIGHT_MOTOR)
        except IOError as error:
            #print(error)
            sys.stdout.write(error)

        # Calculate the arc of circumfrence traveled by each wheel
        left_offset         = left_encoder - self.sI.value
        #left_offset_length  = np.deg2rad(left_offset) * self.R
        right_offset        = right_encoder - self.sD.value
        #right_offset_length = np.deg2rad(right_offset) * self.R

        # calculate real speed
        wi     = np.deg2rad(left_offset) / self.P
        wd     = np.deg2rad(right_offset) / self.P
        [v, w] = np.dot(np.array([[self.R/2, self.R/2],[self.R/self.L, -self.R/self.L]]), np.array([[wd],[wi]]))
        return [wi, wd, v, w] 

    def readOdometry(self):
        """ Returns current value of odometry estimation """
        #print("Grados Rueda Izquierda: %.2f, Grados rueda Derecha: %.2f" %(np.rad2deg(self.sI.value), np.rad2deg(self.sD.value)))
        return self.x.value, self.y.value, self.th.value # [self.wi.value, self.wd.value, self.v.value, self.w.value]

    def startOdometry(self):
        """ This starts a new process/thread that will be updating the odometry periodically """
        self.finished.value = False
        self.p = Process(target=self.updateOdometry, args=()) #additional_params?))
        self.p.start()
        print("PID: ", self.p.pid)

    # You may want to pass additional shared variables besides the odometry values and stop flag
    def updateOdometry(self): #, additional_params?):

        LOG_NAME = "ODOMETRYLOG_" + datetime.datetime.now().strftime("%H-%M-%S_%Y-%m-%d") + ".csv"
        with open(LOG_NAME, "w", newline="") as LOG:
            # Logger
            LOG_WRITER = csv.writer(LOG)
            LOG_WRITER.writerow(["Timestamp", "Posición en x", "Posición en y", "Angulo de giro"])
            # Bucle principal
            while not self.finished.value:
                # current processor time in a floating point value, in seconds
                tIni = time.clock()

                # Compute updates
                # Update odometry uses values that require mutex, they are declared as value, so lock
                # is implicitly done for atomic operations (BUT =+ is NOT atomic)
                try:
                    # Each of the following BP.get_motor_encoder functions returns the encoder value
                    # (what we want to store).
                    sys.stdout.write("Reading encoder values .... \n")
                    left_encoder  = self.BP.get_motor_encoder(self.PORT_LEFT_MOTOR)
                    right_encoder = self.BP.get_motor_encoder(self.PORT_RIGHT_MOTOR)
                except IOError as error:
                    #print(error)
                    sys.stdout.write(error)

                # Calculate the arc of circumfrence traveled by each wheel
                left_offset         = left_encoder - self.sI.value
                #left_offset_length  = np.deg2rad(left_offset) * self.R
                right_offset        = right_encoder - self.sD.value
                #right_offset_length = np.deg2rad(right_offset) * self.R

                # Calculate real speed
                wi = np.deg2rad(left_offset) / self.P  # Left wheel angular real speed
                wd = np.deg2rad(right_offset) / self.P # Right wheel angular real speed
                # Robot linear and angular real speed
                vw = np.dot(np.array([[self.R/2, self.R/2],[self.R/self.L, -self.R/self.L]]), np.array([[wd],[wi]]))
                # Calculate real delta th. (extraer del giroscopio más adelante)
                delta_th = vw[1] * self.P
                #delta_th = (right_offset_length - left_offset_length) / self.L
                th = self.th.value + delta_th

                # Calculo de delta xWR. Calculo de delta s (depends on w) (diapo 14)
                delta_x, delta_y = 0, 0
                if vw[1] != 0:
                    delta_s = (vw[0]/vw[1]) * delta_th
                    delta_x = delta_s * np.cos(th + (delta_th * 0.5))
                    delta_y = delta_s * np.sin(th + (delta_th * 0.5))
                else:
                    delta_s = vw[0] * self.P
                    delta_x = delta_s * np.cos(th)
                    delta_y = delta_s * np.sin(th)

                # To "lock" a whole set of operations, we can use a "mutex"
                # Update new xWR+
                self.lock_odometry.acquire()
                self.x.value += delta_x
                self.y.value += delta_y
                
                self.th.value = th
                self.sI.value = left_encoder
                self.sD.value = right_encoder
                #self.tLength.value += delta_s
                self.lock_odometry.release()

                # Save LOG
                LOG_WRITER.writerow([datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"), round(self.x.value, 4), round(self.y.value, 4), round(self.th.value, 4)])

                ######## UPDATE UNTIL HERE with your code ########
                tEnd = time.clock()
                time.sleep(self.P - (tEnd-tIni))

        #print("Stopping odometry ... X= %d" %(self.x.value))
        sys.stdout.write("Stopping odometry ... X=  %.2f, \
                Y=  %.2f, th=  %.2f \n" %(self.x.value, self.y.value, self.th.value))

    # Stop the odometry thread.
    def stopOdometry(self):
        self.finished.value = True
        self.BP.reset_all()


    #--------- Tracking Object ------------
        
    def _get_best_blob(self, blobs):
        # Criterio: más grande y más centrado si son iguales, sino el más grande
        
        ind_centrado = max(enumerate(blobs), key=lambda x: x[1].pt[0])[0]
        ind_grande = max(enumerate(blobs), key=lambda x: x[1].size)[0]

        

        # devuelve el blob más grande si es el más centrado, sino el más grande
        best = blobs[ind_centrado] if ind_centrado == ind_grande else blobs[ind_grande]
        print("\nBest blob: ", best.pt[0], best.pt[1], best.size)
        return best


    def _init_my_blob_detector(self):
        # Setup default values for SimpleBlobDetector parameters.
        params = cv2.SimpleBlobDetector_Params()

        # These are just examples, tune your own if needed
        # Change thresholds
        # indica el brillo del píxel
        # params.minThreshold = 10
        # params.maxThreshold = 200

        # Filter by Area (en píxeles cuadrados)
        params.filterByArea = True
        params.minArea = 40
        params.maxArea = 5000

        # Filter by Circularity
        params.filterByCircularity = False
        params.minCircularity = 0.1

        # Filter by Color 
        params.filterByColor = False
        # not directly color, but intensity on the channel input
        # This filter compares the intensity of a binary image at the center of a blob to blobColor.
        params.blobColor = 100

        params.filterByConvexity = False
        params.filterByInertia = False


        # Create a detector with the parameters
        ver = (cv2.__version__).split('.')
        if int(ver[0]) < 3 :
            detector = cv2.SimpleBlobDetector(params)
        else :
            detector = cv2.SimpleBlobDetector_create(params)

        return detector	
    

    def _blobs_capture(self, img, detector, mask):
        '''
            Capture the blobs

            Parameters:
                img = image
                detector = blob detector
                colorRangeMin = minimum color range
                colorRangeMax = maximum color range
            
            Returns:
                keypoints_red = keypoints of the red blobs
        '''
        # define red mask
        #mask = cv2.inRange(img, colorRangeMin, colorRangeMax)

        # detector finds "dark" blobs by default, so invert image for results with same detector
        keypoints = detector.detect(cv2.bitwise_not(mask))

        # kp.p[0] = x coordenate on the image 
        # kp.p[1] = y coordenate on the image
        # kp.size = diameter of the blob
        print("\n New frame")
        for kp in keypoints:
            print(kp.pt[0], kp.pt[1], kp.size)

        return keypoints
    
    def _init_camera(self):
        '''
            Initialize the camera

            Parameters:
                resolution = resolution of the camera
                framerate = framerate of the camera

            Returns:
                cam = camera object
                rawCapture = object to manage the camera
        '''
        cam = picamera.PiCamera()
        cam.resolution = self.resolution # (640, 480)
        cam.framerate = self.framerate # 32
        rawCapture = PiRGBArray(cam, size=self.resolution)
        # rawCapture = PiRGBArray(cam, size=(640, 480))

        # Allow the camera to warmup
        time.sleep(0.1)

        return cam, rawCapture

    def trackObject(self, colorRangeMin, colorRangeMax):
        # targetSize=??, target=??, catch=??, ...)
        '''
            Track the object

            Parameters:
                colorRangeMin = minimum color range
                colorRangeMax = maximum color range

            Returns:
                finished = True if the tracking is finished
        '''
        # flags
        finished = False
        targetFound = False
        targetPositionReached = False

        # Initializations
        detector = self._init_my_blob_detector()

        # Get blob mask
        mask = cv2.inRange(frame, colorRangeMin[0], colorRangeMax[0])
        for i in range(1,len(colorRangeMin)):
            other = cv2.inRange(frame, colorRangeMin[i], colorRangeMax[i])
            mask  = cv2.bitwise_or(mask, other)
        
        cam, rawCapture = self._init_camera(resolution=resolution, framerate=32)

        side = 1 # Left = -1, Right = 1
        wmax = np.log10(resolution_center[0])
        a    = 0

        # main loop
        while not finished:

            for img in cam.capture_continuous(rawCapture, format="bgr", use_video_port=True):

                frame  = cv2.cvtColor(img.array, cv2.COLOR_BGR2HSV)
                
                # detect blobs
                kp = self._blobs_capture(frame, detector, mask)
    
                # img_show = cv2.drawKeypoints(frame, kp, np.array([]), (255,255,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
                # cv2.imshow('Captura', img_show)
                epsilon = 10
                # if no blobs found, rotate until found
                if kp:
                    print("Blob found")
                    # 1. Search the most promising blob ..

                    best_blob = self._get_best_blob(kp)
                    transform = Transform(Vector3(x=best_blob.pt[0]-center.x, y=0)) #best_blob.pt[1]-center[1]
                    # w = np.log(abs(x)) - 1
                    # w = np.abs(np.exp(0.02*x) - 1)
                    # print("W=%.2f" % w)

                    # Center the ball
                    if transform == Transform(Vector2.zero, CUSTOM_POSITION_ERROR=2):
                        transform.position.y = best_blob.pt[1]-center.y
                        A = 2 * np.pi * ((best_blob.size/2)**2)
                        if transform == Transform(Vector2.zero, CUSTOM_POSITION_ERROR=2):
                            self.setSpeed(0, 0)
                            # catch
                        else:
                            self.setSpeed(np.log10(1/sqrt(A - a)), 0)
                            a = A
                    else:
                        # Velocidad correspondiente:
                        # Si sign < 0, esta a la izquierda, por lo que w ha de ser positivo -1*sign*w = -1*-1*w
                        # Si sign > 0, esta a la derecha, por lo que w ha de ser negativo -1*sign*w = -1*1*w
                        self.setSpeed(0, -np.sign(transform.position.x) * np.log10(abs(transform.position.x) - 1))
                        # Ultima posicion vista:
                        # Si sign < 0, esta a la izquierda
                        # Si sign > 0, esta a la derecha
                        side = np.sign(transform.position.x)
                else:
                    print("Blob not found, rotating ...")
                    a = 0                    
                    self.setSpeed(0, side*wmax)
                     
                rawCapture.truncate(0)   # clear the stream in preparation for the next frame
                
                # 'ESC' = 27
                if cv2.waitKey(1) & 0xff == 27:
                    cam.close()
                    finished = True
                    break

            print("Fin tracking")
            finished = True
             
            # while not targetPositionReached:
                # 2. decide v and w for the robot to get closer to target position

               
                # a, d = get_blob_parameters(blob)
                # if d == 0
                #     self.setSpeed(0,0)
                #     targetPositionReached = True
                #     targetFound = True
                #     finished = True
                # else:
                #     v, w = getNewSpeed(a, d)
                #     self.setSpeed(v, w)

        return finished

    def catch(self):
        # decide the strategy to catch the ball once you have reached the target position
        self.setNestSpeed(0,np.pi / 2)

    