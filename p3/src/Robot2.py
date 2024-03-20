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
from geometry import Vector2, Transform

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
        self.PORT_BASKET_MOTOR = self.BP.PORT_B
        # Reset encoder B and C (or all the motors you are using)
        self.BP.offset_motor_encoder(self.PORT_RIGHT_MOTOR, self.BP.get_motor_encoder(self.PORT_RIGHT_MOTOR))
        self.BP.offset_motor_encoder(self.PORT_LEFT_MOTOR, self.BP.get_motor_encoder(self.PORT_LEFT_MOTOR))
        self.BP.offset_motor_encoder(self.PORT_BASKET_MOTOR, self.BP.get_motor_encoder(self.PORT_BASKET_MOTOR))
        ##################################################
        # Odometry shared memory values
        self.x  = Value('d', init_position[0]) # Robot X coordinate.
        self.y  = Value('d', init_position[1]) # Robot Y coordinate.
        self.th = Value('d', init_position[2]) # Robot orientation.
        self.bh  = Value('d', 0)               # Robot basket angle [0 a ~90º].
        self.sD = Value('i', 0)                # Latest stored RIGHT encoder value.
        self.sI = Value('i', 0)                # Latest stored LEFT encoder value.
        self.sC = Value('i', 0)                # Latest stored BASKET encoder value.
        self.v  = Value('d', 0.0)              # Robot linear theorical speed (or tangential if rotating).
        self.w  = Value('d', 0.0)              # Robot angular theorical speed.
        self.wb = Value('d', 0.0)              # Robot basket angular theorical speed.
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
        # Blob detector
        self.blob_detector_params = {
            "minThreshold": 10,
            "maxThreshold": 200,
            "filterByArea": True,
            "minArea": 40,
            "maxArea": 5000,
            "filterByCircularity": False,
            "minCircularity": 0.1,
            "filterByColor": False,
            "blobColor": 100,
            "filterByConvexity": False,
            "filterByInertia": False
        }
        print(self.blob_detector_params)
        # Camera servoing
        self.resolution = resolution           # Resolucion de la camara
        self.cam_center = Vector2(resolution[0]//2, resolution[1]//2)
                                               # Centro de la camara (en base a la resolucion en pixeles)
        self.cam_xv     = lambda x,v: v * (np.sqrt(x/self.blob_detector_params["maxArea"]) + 1)
                                               # Formula para obtener la constante 'c' necesaria para que en el 
                                               # area 'x' haya una velocidad 'v'. Nace de la idea de que 1/sqrt(x)
                                               # hace que las velocidades sean pequeñas y decrezcan demasiado
                                               # rapido.
        self.cam_fv     = lambda x: self.cam_xv(1000, 5)/(np.sqrt(x/self.blob_detector_params["maxArea"])+1)
                                               # Formula para la velocidad respecto del area
                                               #  v = c / (√(x/max_area) + 1)
                                               #  - La c es una constante que ajusta la altura de la función. Con
                                               #    varias pruebas se ha visto que en distancias medias el area
                                               #    rondaban los 1000pixeles^2, entonces, forzamos una c tal que
                                               #    haya 5cm/s para que al decrecer no sea demasiado baja.
                                               #  - La x es el area.
                                               #  - Se divide por el area maxima para relajar la restriccion del
                                               #    denominador y no salga una velocidad muy pequeña.
                                               #  - El +1 es para evitar indefinido si x=0.
        self.cam_fw     = lambda x: -2*x/self.cam_center.x
        self.cam_xrev   = 10                   # Distancia que debe recorrer el robot hacia atras si no la ve.
        self.cam_wmax   = self.cam_fw(self.cam_center.x)
        self.framerate  = framerate            # Framerate

    def setSpeed(self, v: float, w: float, wb: float):
        """
        Set the new robot speed. \
        
        - self: The robot itself.
        - v:    Robot linear speed.
        - w:    Robot angular speed.
        - wb:   Robot basket angular speed.
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
        self.BP.set_motor_dps(self.PORT_BASKET_MOTOR, np.rad2deg(wb))

        # Store speed data (really necessary?)
        # Realmente, podria ser necesario a la hora de corregir la velocidad, pero 
        # lo usaremos mas adelante.
        self.lock_odometry.acquire()
        self.v.value = v
        self.w.value = w
        self.wb.value = wb
        self.lock_odometry.release()
    
    def readSpeed(self):
        """
        Get the robot real speed.
        """
        try:
            # Each of the following BP.get_motor_encoder functions returns the encoder value
            # (what we want to store).
            #sys.stdout.write("Reading encoder values .... \n")
            left_encoder   = self.BP.get_motor_encoder(self.PORT_LEFT_MOTOR)
            right_encoder  = self.BP.get_motor_encoder(self.PORT_RIGHT_MOTOR)
            basket_encoder = self.BP.get_motor_encoder(self.PORT_BASKET_MOTOR) 
        except IOError as error:
            #print(error)
            sys.stdout.write(error)

        # Calculate the arc of circumfrence traveled by each wheel
        left_offset   = left_encoder   - self.sI.value
        #left_offset_length  = np.deg2rad(left_offset) * self.R
        right_offset  = right_encoder  - self.sD.value
        #right_offset_length = np.deg2rad(right_offset) * self.R
        basket_offset = basket_encoder - self.sC.value
        # Calculate real speed
        wi     = np.deg2rad(left_offset)   / self.P # Left wheel angular real speed
        wd     = np.deg2rad(right_offset)  / self.P # Right wheel angular real speed
        wb     = np.deg2rad(basket_offset) / self.P # Basket angular real speed
        # Robot linear and angular real speed
        [v, w] = np.dot(np.array([[self.R/2, self.R/2],[self.R/self.L, -self.R/self.L]]), np.array([[wd],[wi]]))
        return [left_encoder, right_encoder, basket_encoder, wi, wd, wb, v, w] 

    def readOdometry(self):
        """ Returns current value of odometry estimation """
        #print("Grados Rueda Izquierda: %.2f, Grados rueda Derecha: %.2f" %(np.rad2deg(self.sI.value), np.rad2deg(self.sD.value)))
        return self.x.value, self.y.value, self.th.value, self.bh.value

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
                # Update odometry uses values that require mutex, they are declared as value, so lock
                # is implicitly done for atomic operations (BUT =+ is NOT atomic)
                # Calculate the arc of circumfrence traveled by each wheel
                left_encoder, right_encoder, basket_encoder, wi, wd, _, v, w = self.readSpeed()
                # Calculate real delta th. (extraer del giroscopio más adelante)
                delta_th = w * self.P
                th = self.th.value + delta_th
                bh = self.bh.value + (basket_encoder - self.sC.value)
                # Calculo de delta xWR. Calculo de delta s (depends on w) (diapo 14)
                delta_x, delta_y = 0, 0
                if w != 0:
                    delta_s = (v/w) * delta_th
                    delta_x = delta_s * np.cos(th + (delta_th * 0.5))
                    delta_y = delta_s * np.sin(th + (delta_th * 0.5))
                else:
                    delta_s = v * self.P
                    delta_x = delta_s * np.cos(th)
                    delta_y = delta_s * np.sin(th)

                # To "lock" a whole set of operations, we can use a "mutex"
                # Update new xWR+
                self.lock_odometry.acquire()
                self.x.value += delta_x
                self.y.value += delta_y
                self.th.value = th
                self.bh.value = bh
                self.sD.value = right_encoder
                self.sI.value = left_encoder
                self.sC.value = basket_encoder
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
    def initCamera(self):
        '''
            Initialize the camera

            Returns:
                cam = camera object
                rawCapture = object to manage the camera
        '''
        # Init camera
        cam = picamera.PiCamera()
        cam.resolution = self.resolution # (640, 480)
        cam.framerate = self.framerate   # 32
        rawCapture = PiRGBArray(cam, size=self.resolution)
        # Allow the camera to warmup
        time.sleep(0.1)

        return cam, rawCapture

    def initBlobDetector(self):
        '''
            Initialize the camera

            Returns:
                detector = blob detector object
        '''
        # Setup default values for SimpleBlobDetector parameters.
        params = cv2.SimpleBlobDetector_Params()
        # Change thresholds. Indica el brillo del píxel
        # params.minThreshold = self.blob_detector_params["minThreshold"]
        # params.maxThreshold = self.blob_detector_params["maxThreshold"]
        # Filter by Area (en píxeles cuadrados)
        params.filterByArea = self.blob_detector_params["filterByArea"]
        params.minArea = self.blob_detector_params["minArea"]
        params.maxArea = self.blob_detector_params["maxArea"]
        # Filter by Circularity
        params.filterByCircularity = self.blob_detector_params["filterByCircularity"]
        params.minCircularity = self.blob_detector_params["minCircularity"]
        # Filter by Color 
        params.filterByColor = self.blob_detector_params["filterByColor"]
        # Not directly color, but intensity on the channel input
        # This filter compares the intensity of a binary image at the center of a blob to blobColor.
        params.blobColor = self.blob_detector_params["blobColor"]
        # Other
        params.filterByConvexity = self.blob_detector_params["filterByConvexity"]
        params.filterByInertia = self.blob_detector_params["filterByInertia"]
        # Create a detector with the parameters
        ver = (cv2.__version__).split('.')
        if int(ver[0]) < 3 :
            return cv2.SimpleBlobDetector(params)
        else :
            return cv2.SimpleBlobDetector_create(params)

    def getBestBlob(self, keypoints):
        '''
            Track the object

            Parameters:
                keypoints = keypoints de la imagen

            Returns:
                best_blob = best keypoint or None if didn't meets the filter.
        '''
        # Si esta vacio, no hay mejor blob
        if not keypoints:
            return None

        best_kp = None
        for kp in keypoints:
            # Filtro
            if kp.pt[1] >= self.cam_center.y:
                print("Current filtered blob:", kp.pt, kp.size)
                if not best_kp:
                    best_kp = kp
                    continue
                else:
                    if best_kp.size < kp.size:
                        best_kp = kp
                    elif (best_kp.size == kp.size) and (abs(best_kp.pt[0] - self.cam_center.x) > abs(kp.pt[0] - self.cam_center.x)):
                        best_kp = kp
        # kp.pt[0] = x coordenate on the image 
        # kp.pt[1] = y coordenate on the image
        # kp.size = diameter of the blob
        if best_kp:
            print("\nBest blob found: ", best_kp.pt[0], best_kp.pt[1], best_kp.size)
        return best_kp

    def trackObject(self, colorRangeMin, colorRangeMax, showFrame=False):
        # targetSize=??, target=??, catch=??, ...)
        '''
            Track the object

            Parameters:
                colorRangeMin = minimum color range
                colorRangeMax = maximum color range
                showFrame     = show captured frame

            Returns:
                finished = True if the tracking is finished
        '''
        # Flags
        #targetRotationReached = False
        # Initializations
        cam, rawCapture = self.initCamera()
        detector = self.initBlobDetector()
        # Servoing values
        side      = 1
        #last_blob = None
        #last_odom = None
        # main loop
        while True:
            for img in cam.capture_continuous(rawCapture, format="bgr", use_video_port=True):
                # Get a new frame
                frame = cv2.cvtColor(img.array, cv2.COLOR_BGR2HSV)
                mask  = cv2.inRange(frame, colorRangeMin[0], colorRangeMax[0])
                for i in range(1,len(colorRangeMin)):
                    other = cv2.inRange(frame, colorRangeMin[i], colorRangeMax[i])
                    mask  = cv2.bitwise_or(mask, other)
                # Detect all blobs. Detector finds "dark" blobs by default, so invert image for results with same detector
                keypoints = detector.detect(cv2.bitwise_not(mask))
                # Search for the most promising blob...
                best_blob = self.getBestBlob(keypoints)
                if showFrame and best_blob:
                    image = cv2.drawKeypoints(frame, keypoints, np.array([]), (255,255,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
                    cv2.imshow('Captura', image)
                # Decide v, w and wb for the robot to get closer to target position
                v = 0; w = 0; wb = 0
                if best_blob:
                    # Targer rotation
                    blob_transform = Transform(Vector2(x=best_blob.pt[0], y=0))
                    if not blob_transform == Transform(Vector2.zero, CUSTOM_POSITION_ERROR=2):
                        # Ultima posicion vista. Si sign < 0, esta a la izquierda, si sign > 0, esta a la derecha
                        side = np.sign(blob_transform.position.x)
                        # Velocidades. Si sign < 0 -> w > 0 (-1*sign*w = -1*-1*w), si sign > 0 -> w < 0 (-1*sign*w = -1*1*w)
                        w = self.cam_fw(blob_transform.position.x)
                    #else:
                    #    targetRotationReached = True
                    # Target position
                    #blob_transform = Transform(Vector2(x=0, y=best_blob.pt[1]))
                    ## if targetRotationReached and blob_transform == cam_center_transform:
                    #if not blob_transform == Transform(Vector2(0, self.center.y*3/4)):
                    #    # Calculamos el area del blob
                    #    blob_area = np.pi * (best_blob.size/2)**2
                    #    # Calculamos la velocidad correspondiente (siempre positiva, el area es inversamente proporcional al area
                    #    v = self.cam_fv(blob_area)
                    #else:
                    #    return True
                    #last_blob = best_blob
                    #last_odom = None
                else:
                    # b. Rotate until found
                    # Get robot odometry
                    #x, y, th, bh = self.readOdometry()
                    ## Si la ultima vez que se ha visto la pelota ha sido por debajo de la imagen,
                    ## se retrocede un poco para ver si se ve otra vez
                    #if last_blob and last_blob.pt[1] < 0:
                    #    if not last_odom:
                    #        last_odom = Vector2(x, y)
                    #    elif (Vector2(x,y) - last_odom).magnitude() <= self.cam_xrev:
                    #        v = -2
                    ## Girar hasta encontrar la pelota
                    w = side * self.cam_wmax
                    ## Si la cesta se ha bajado, se vuelve a subir
                    #if bh > 5:
                    #    wb = -5

                # Robot speed
                self.setSpeed(v, w, wb)
                # Clear the stream in preparation for the next frame
                rawCapture.truncate(0)

    def catch(self):
        # Decide the strategy to catch the ball once you have reached the target position
        while True:
            _, _, _, bh = self.readOdometry()
            if bh < 85:
                self.setSpeed(0,0,5)
            else:
                self.setSpeed(0,0,0)
                break
