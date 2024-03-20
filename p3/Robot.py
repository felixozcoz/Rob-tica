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
    def __init__(self, init_position=[0.0, 0.0, 0.0], resolution=(640, 480), framerate=32):
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
        self.b  = Value('d', 0)                # Robot basket angle [0 a ~90º].
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
        # Blob detector
        self.blob_detector_params = {
            "minThreshold": 10,
            "maxThreshold": 200,
            "filterByArea": True,
            "minArea": 10,
            "maxArea": 80000,
            "filterByCircularity": False,
            "minCircularity": 0.1,
            "filterByColor": False,
            "blobColor": 100,
            "filterByConvexity": False,
            "filterByInertia": False
        }
        # Camera
        self.resolution = resolution           # Resolucion de la camara
        self.cam_center = Vector2(resolution[0]//2, resolution[1]//2)
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
        return self.x.value, self.y.value, self.th.value

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
                b  = self.b.value  + basket_encoder - self.sC.value
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
                self.b.value  = b
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
        cam.framerate = self.framerate   # 32
        rawCapture = PiRGBArray(cam, size=self.resolution)

        # Allow the camera to warmup
        time.sleep(0.1)

        return cam, rawCapture

    def _init_my_blob_detector(self):
        # Setup default values for SimpleBlobDetector parameters.
        params = cv2.SimpleBlobDetector_Params()

        # These are just examples, tune your own if needed
        # Change thresholds
        # indica el brillo del píxel
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
            detector = cv2.SimpleBlobDetector(params)
        else :
            detector = cv2.SimpleBlobDetector_create(params)

        return detector	
    

    def _get_best_blob(self, blobs):
        # Si no hay blobs, no hay mejor
        if not blobs:
            return None
        # Criterio: más grande y más centrado si son iguales, sino el más grande
        best_blob = None
        for blob in blobs:
            # Filtro
            if blob.pt[1] >= (1-3/4)*(self.cam_center.y):
                print("Current filtered blob:", blob.pt, blob.size)
                if not best_blob:
                    best_blob = blob
                    continue
                else:
                    if best_blob.size < blob.size:
                        best_blob = blob
                    elif (best_blob.size == blob.size) and (abs(best_blob.pt[0]) > abs(blob.pt[0])):
                        best_blob = blob
        if best_blob:
            print("\nBest blob found: ", best_blob.pt[0], best_blob.pt[1], best_blob.size)
            
        return best_blob

    
    def trackObject(self, colorRangeMin, colorRangeMax, showFrame=False):
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
        #targetFound = False
        targetRotationReached = False
        targetPositionReached = False
        targetCatched = False

        # Initializations
        cam, rawCapture = self._init_camera()
        cam_center_transform = Transform(Vector2.zero, CUSTOM_POSITION_ERROR=5)
        detector = self._init_my_blob_detector()

        side = 1            # indica el sentido la última vez que se vio la bola (-1 = izq, 1 = dcha)
        wmax = (-2 * self.cam_center.x)/self.cam_center.x     # w cuando no encuentra ningún

        # main loop
        while not finished:
            for img in cam.capture_continuous(rawCapture, format="bgr", use_video_port=True):
                # 0. Get a new frame
                # frame = cv2.cvtColor(img.array, cv2.COLOR_BGR2HSV)
                frame = cv2.bitwise_not(img.array)  # invertir imagen a negativo
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) 
                mask = cv2.inRange(frame, colorRangeMin, colorRangeMax) 

                # red mask
                # mask1 = cv2.inRange(frame, (0, 70, 50), (10, 255, 255))
                # mask2 = cv2.inRange(frame, (170, 70, 50), (180, 255, 255))
                # mask3 = cv2.inRange(frame, (70, 20, 155), (90, 40, 185))
                #mask  = cv2.bitwise_or(mask1, mask2, mask3)

                #for i in range(1,len(colorRangeMin)):
                #    other = cv2.inRange(frame, colorRangeMin[i], colorRangeMax[i])
                #    mask  = cv2.bitwise_or(mask, other)
                
                # 1. Detect all blobs
                keypoints = detector.detect(mask)

                # 2. Search for the most promising blob...
                best_blob = self._get_best_blob(keypoints)
                # if showFrame:
                #     img = cv2.bitwise_and(frame, frame, mask=mask)
                #     image = cv2.drawKeypoints(img, keypoints, np.array([]), (255,255,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
                #     cv2.imshow('Captura', image)
                #     if cv2.waitKey(1) & 0xff == 27:
                #         cam.close()
                #         finished = True
                #         break

                rawCapture.truncate(0)  # remove img.array content
                # 3. Decide v and w for the robot to get closer to target position
                v = 0; w = 0
                if best_blob:
                    # Targer rotation
                    blob_transform = Transform(Vector2(x=best_blob.pt[0]-self.cam_center.x, y=0))
                    if not cam_center_transform == blob_transform:
                        # Ultima posicion vista. Si sign < 0, esta a la izquierda, si sign > 0, esta a la derecha
                        side = np.sign(blob_transform.position.x)
                        # Velocidades. Si sign < 0 -> w > 0 (-1*sign*w = -1*-1*w), si sign > 0 -> w < 0 (-1*sign*w = -1*1*w)
                        w = (-2*blob_transform.position.x)/self.cam_center.x
                    else:
                        targetRotationReached = True

                    # Target position
                    blob_transform = Transform(Vector2(x=0, y=best_blob.pt[1]))
                    catch_position = Transform(Vector2(x=0, y = 460), CUSTOM_POSITION_ERROR= 10) # Esto es correcto ??? Si
                    #if targetRotationReached and blob_transform == cam_center_transform:
                    if targetRotationReached and not catch_position == blob_transform:
                        # Calculamos el area del blob
                        # blob_area = np.pi * (best_blob.size/2)**2
                        # Calculamos la velocidad correspondiente (siempre positiva, el area es inversamente proporcional al area
                        #v= 8/(np.sqrt(blob_area/(self.resolution[0] * self.resolution[1]))+2)
                        v = 3 # Calcular con la distancia al borde de la imagen
                    else:
                        #print("Hola")
                        targetPositionReached = True
                        finished = True
                else:
                    # b. Rotate until found
                    print("Blob not found, rotating ...")
                    w = side * wmax
                    #targetPositionReached = False
                    #targetCatched = False

                # Robot speed
                self.setSpeed(v, w)
                # Capture the target
                #if targetPositionReached:
                #    if not targetCatched:
                #        self.BP.set_motor_dps(self.PORT_BASKET_MOTOR, np.rad2deg( 5))
                #    if self.b.value >= 85:
                #        targetCatched = True
                #elif self.b.value > 5:
                #        self.BP.set_motor_dps(self.PORT_BASKET_MOTOR, np.rad2deg(-5))
                # Clear the stream in preparation for the next frame
                # rawCapture.truncate(0)
                # Show the captured frame if needed

            print("Fin tracking")
            finished = True

        return finished

    def catch(self):
        # decide the strategy to catch the ball once you have reached the target position
        self.setNestSpeed(0,np.pi / 2)