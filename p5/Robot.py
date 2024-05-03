#!/usr/bin/python
# -*- coding: UTF-8 -*-
from __future__ import print_function # use python 3 syntax but make it compatible with python 2
from __future__ import division       #                           ''

import brickpi3 # import the BrickPi3 drivers
import csv      # write csv
import cv2
import datetime # timestamps
import numpy as np
import picamera
import sys
import time     # import the time library for the sleep function
from picamera.array import PiRGBArray
from decimal import Decimal
from geometry import Vector2, Matrix2, Transform
from ReMapLib import Map
from scipy.interpolate import PchipInterpolator

# Threading package could be used too
from multiprocessing import Process, Value, Array, Lock

class Robot:
    def __init__(self, local_reference=[0.0, 0.0, 0.0], resolution=(320, 240), framerate=32):
        """
        Initialize basic robot params. \

        Initialize Motors and Sensors according to the set up in your robot
        """
        # Robot construction parameters
        self.R = 2.8   # Wheels' radius (cm)
        self.L = 15.25 # Wheels' axis length (cm)
        ##################################################
        # Motors and sensors setup
        # Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.
        self.BP = brickpi3.BrickPi3()
        # Configure sensors, for example a touch sensor.
        #self.BP.set_sensor_type(self.BP.PORT_1, self.BP.SENSOR_TYPE.TOUCH)
        # Configure utltrasonic sensors
        self.PORT_ULTRASONIC_EV3 = self.BP.PORT_1
        self.PORT_ULTRASONIC_NXT = self.BP.PORT_2
        self.BP.set_sensor_type(self.PORT_ULTRASONIC_EV3, self.BP.SENSOR_TYPE.EV3_ULTRASONIC_CM)
        self.BP.set_sensor_type(self.PORT_ULTRASONIC_NXT, self.BP.SENSOR_TYPE.NXT_ULTRASONIC)
        # Configure color sensor
        self.PORT_COLOR_SENSOR   = self.BP.PORT_3
        self.BP.set_sensor_type(self.PORT_COLOR_SENSOR, self.BP.EV3_COLOR_COLOR)
        # Configure gyroscope
        self.PORT_GYROSCOPE      = self.BP.PORT_4
        self.BP.set_sensor_type(self.PORT_GYROSCOPE, self.BP.SENSOR_TYPE.EV3_GYRO_ABS_DPS)
        # Configure motors
        self.PORT_LEFT_MOTOR     = self.BP.PORT_D
        self.PORT_RIGHT_MOTOR    = self.BP.PORT_A
        self.PORT_BASKET_MOTOR   = self.BP.PORT_B
        # Reset encoder B and C (or all the motors you are using)
        self.BP.offset_motor_encoder(self.PORT_RIGHT_MOTOR, self.BP.get_motor_encoder(self.PORT_RIGHT_MOTOR))
        self.BP.offset_motor_encoder(self.PORT_LEFT_MOTOR, self.BP.get_motor_encoder(self.PORT_LEFT_MOTOR))
        self.BP.offset_motor_encoder(self.PORT_BASKET_MOTOR, self.BP.get_motor_encoder(self.PORT_BASKET_MOTOR))
        ##################################################
        # ODOMETRY
        # - Odometry shared memory values
        self.x  = Value('d', local_reference[0])
                                               # Robot X coordinate.
        self.y  = Value('d', local_reference[1])
                                               # Robot Y coordinate.
        self.th = Value('d', local_reference[2])
                                               # Robot orientation.
        self.us_ev3 = Value('d', 0)            # Latest distances ultrasonic EV3 sensor stored
        #self.us_nxt = Value('d', 0)            # Latest distances ultrasonic NXT sensor stored
        self.bh = Value('d', 0)                # Robot basket angle [0 to ~90º].
        self.sD = Value('i', 0)                # Latest stored RIGHT encoder value.
        self.sI = Value('i', 0)                # Latest stored LEFT encoder value.
        self.sC = Value('i', 0)                # Latest stored BASKET encoder value.
        self.v  = Value('d', 0.0)              # Robot linear theorical speed (or tangential if rotating).
        self.w  = Value('d', 0.0)              # Robot angular theorical speed.
        self.wi = Value('d', 0.0)              # Robot left wheel theorical speed.
        self.wd = Value('d', 0.0)              # Robot right wheel theorical speed.
        self.wb = Value('d', 0.0)              # Robot basket theorical angular speed.
        self.finished = Value('b', 1)          # Boolean to show if odometry updates are finished
        # - If we want to block several instructions to be run together, we may want to use an explicit Lock
        self.lock_odometry = Lock()
        # - Odometry update period
        self.P = 0.01                          # In seconds
        ##################################################
        # VISUAL SERVOING
        # - Camera
        self.resolution = resolution           # Camera resolution.
        self.cam_center = Vector2(resolution[0]//2, resolution[1]//2)
                                               # Camera centre (pixel resolution based)
        self.framerate  = framerate            # Framerate
        # - Detector
        self.blob_detector_params = {          # Blob detector parameters
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
        self.blob_detector_ymin = 3/4*self.cam_center.y
                                               # Minimum distance in y to detect a good blob.
        # - Robot control
        self.backwards_dist = 10               # Distance that the robot will retreat (cm) in case of losing the ball nearby.
        self.xmin_to_rotate = self.cam_center.x//4
                                               # Minimum distance in the image for it to rotate again.
        self.ymin_to_stop   = self.cam_center.y - 10
                                               # Minimum distance in the image to stop advancing.
        # [DEPRECATED]
        #self.fc = lambda x,v: v * (np.sqrt(x/self.blob_detector_params["maxArea"]) + 1)
                                               # Function to obtain the constant 'c' necessary so that in the
                                               # area 'x' has a speed 'v'. It is born from the idea that 1/sqrt(x)
                                               # causes speeds to be small and decrease too much
                                               # fast.
        self.fv = lambda y : -0.14*y + 25      # Function for tangential speed.
                                               # - y is the difference between the component and the best blob
                                               # and the center point of the image 
        self.fw = lambda x : -2*x/self.cam_center.x
                                               # Function for angular velocity.
        self.fw_max = self.fw(self.cam_center.x)
                                               # Maximum angular speed.
        self.fv_max = self.fv(0)               # Maximum linear speed.
        ##################################################


    # -- Velocidad -------------------------
    def setSpeed(self, v: float, w: float, wb: float = 0):
        """
        Set the new robot speed. \
            
        - v:    Robot linear speed.
        - w:    Robot angular speed.
        - wb:   Robot basket angular speed.
        """
        # Calculate motors angular speeds 
        A        = np.array([[1/self.R, self.L/(2*self.R)], [1/self.R, -self.L/(2*self.R)]])
        vc       = np.array([[v],[w]])
        w_motors = np.dot(A, vc) # Wheels angular speed: [wd, wi]

        # Set each motor speed
        self.BP.set_motor_dps(self.PORT_RIGHT_MOTOR, np.rad2deg(w_motors[0])) 
        self.BP.set_motor_dps(self.PORT_LEFT_MOTOR,  np.rad2deg(w_motors[1]))
        self.BP.set_motor_dps(self.PORT_BASKET_MOTOR, np.rad2deg(wb))

        # Set v, w speed
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

    #-- Odometria --------------------------
    def startOdometry(self):
        """ This starts a new process/thread that will be updating the odometry periodically """

        self.finished.value = False
        self.p = Process(target=self.updateOdometry, args=())
        self.p.start()
        for i in range(3):
            print(self.us_ev3.value)
            time.sleep(self.P)

    def readOdometry(self):
        """ Returns current value of odometry estimation """
        
        return self.x.value, self.y.value, self.th.value, self.bh.value

    def updateOdometry(self): 
        """ This function calculates and updates the odometry of the robot """
        LOG_NAME = "ODOMETRYLOG_" + datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S") + ".csv"
        with open(LOG_NAME, "w", newline="") as LOG:
            # Logger
            LOG_WRITER = csv.writer(LOG)
            LOG_WRITER.writerow(["Timestamp", "X position", "Y position", "Orientation"])
            # Main odometry loop
            while not self.finished.value:
                # current processor time in a floating point value, in seconds
                tIni = time.clock()
                # Update odometry uses values that require mutex, they are declared as value, so lock
                # is implicitly done for atomic operations (BUT =+ is NOT atomic)
                # Calculate the arc of circumfrence traveled by each wheel
                left_encoder, right_encoder, basket_encoder, _, _, _, v, w = self.readSpeed()
                
                #Theta with Gyros
                th, _ = self.BP.get_sensor(self.PORT_GYROSCOPE)

                #th_rad = np.deg2rad(th)
                #delta_th = np.deg2rad(th - self.th.value)

                # Theta with odometry
                delta_th = w * self.P
                th_rad   = np.deg2rad(self.th.value + delta_th)
                bh       = self.bh.value  + basket_encoder - self.sC.value
                # Calculate delta xWR. Calculate delta s (depends on w) (slide 14)
                delta_x, delta_y = 0, 0
                if w != 0:
                    delta_s = (v/w) * delta_th
                    delta_x = delta_s * np.cos(th_rad + (delta_th * 0.5))
                    delta_y = delta_s * np.sin(th_rad + (delta_th * 0.5))
                else:
                    delta_s = v * self.P
                    delta_x = delta_s * np.cos(th_rad)
                    delta_y = delta_s * np.sin(th_rad)

                # To "lock" a whole set of operations, we can use a "mutex"
                # Update new xWR+
                self.lock_odometry.acquire()
                # update odometry
                self.x.value += delta_x
                self.y.value += delta_y
                self.th.value = -th
                # update motors values
                self.bh.value = bh
                self.sD.value = right_encoder
                self.sI.value = left_encoder
                self.sC.value = basket_encoder
                # update ultrasonic sensors value
                self.us_ev3.value = self.BP.get_sensor(self.PORT_ULTRASONIC_EV3) 
                self.lock_odometry.release()

                # Save LOG
                LOG_WRITER.writerow([datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"), round(self.x.value, 4), round(self.y.value, 4), round(self.th.value, 4)])

                ######## UPDATE UNTIL HERE with your code ########
                tEnd = time.clock()
                time.sleep(self.P - (tEnd-tIni))

        #print("Stopping odometry ... X= %d" %(self.x.value))
        sys.stdout.write("Stopping odometry ... X=  %.2f, \
                Y=  %.2f, th=  %.2f \n" %(self.x.value, self.y.value, self.th.value))

    def stopOdometry(self):
        """ Stop the odometry thread """
        self.finished.value = True
        self.BP.reset_all()

    #-- Varios -----------------------------
    def getColor(self):
        """ Obtener color detectado"""
        try:
            return self.BP.get_sensor(self.PORT_COLOR_SENSOR)
        except:
            return (-1,-1,-1) 

    #-- Generacion de trayectorias ---------
    def playTrajectory(self, points, segments, showPlot=False):
        # . Separar coordenadas de la trayectoria
        x = [point[0] for point in points]
        y = [point[1] for point in points]
        # . (DEPRECATED) Función interpolada respecto de los
        #  puntos dados mediante interpolación polinómica
        # deg          = len(x)-1
        # coefficients = np.polyfit(x,y,deg)
        # trajectory   = np.poly1d(coefficients)
        # . Función interpolada respecto de los puntos dados
        #  mediante PCHIP (quita los minimos y maximos que 
        #  añade la polinómica)
        trajectory = PchipInterpolator(x, y)
        x_values   = np.linspace(min(x), max(x), segments)
        y_values   = trajectory(x_values)
        #w_values  = np.arctan(trajectory.derivative()(y_values)/trajectory.derivative()(x_values))
        if showPlot:
            plt.figure(figsize=(8,6))
            plt.plot(x_values, y_values, label="Polinomio interpolado", color="blue")
            #plt.plot(x_values, w, label="Velocidad angular", color="green")
            plt.scatter(x, y, label="Puntos conocidos", color="red")
            plt.xlabel("x")
            plt.ylabel("y")
            plt.title("Interpolacion polinomica")
            plt.legend()
            plt.axis("equal")
            plt.grid(True)
            plt.show()

        # Recorrer trayectoria
        state    = "START_SEGMENT_ADVENTURE"
        position = Vector2(x_values[0], y_values[0], 1)
        position_transform = Transform(Vector2.zero)
        v = 10
        w = 0.75

        for i in range(1, segments):
            # Leer odometria
            x, y, th, _ = self.readOdometry()
            transform   = Transform(Vector2(x, y), th)
            forward     = Vector2.right.rotate(th)
            # Estados
            if state == "START_SEGMENT_ADVENTURE":
                next_position      = Vector2(x_values[i], y_values[i])
                direction          = next_position - position
                position_transform = Transform(next_position, 0)
                self.setSpeed(v, forward.sense(direction) * w)
                state = "GO"
                print("START_SEGMENT_ADVENTURE -> GO")
            elif state == "GO":
                if position_transform == transform:
                    state = "START_SEGMENT_ADVENTURE"
                    print("GO -> START_SEGMENT_ADVENTURE")

        self.setSpeed(0,0)


    #-- Seguimiento de objetos -------------
    def initCamera(self):
        '''
            Initialize the camera

            Returns:
                cam = camera object
                rawCapture = object to manage the camera
        '''
        # Init camera
        cam = picamera.PiCamera()
        cam.resolution = self.resolution # default (640, 480)
        cam.framerate = self.framerate   # default 32
        rawCapture = PiRGBArray(cam, size=self.resolution)

        # Allow the camera to warmup
        time.sleep(0.1)

        return cam, rawCapture

    def initMyBlobDetector(self):
        '''
            Initialize the blob detector

            Returns:
                detector = SimpleBlobDetector object
        '''
        # https://learnopencv.com/blob-detection-using-opencv-python-c/
        # Setup default values for SimpleBlobDetector parameters.
        params = cv2.SimpleBlobDetector_Params()

        # Change thresholds. 
        # params.minThreshold = self.blob_detector_params["minThreshold"]
        # params.maxThreshold = self.blob_detector_params["maxThreshold"]
        # Filter by Area (square pixels)
        params.filterByArea        = self.blob_detector_params["filterByArea"]
        params.minArea             = self.blob_detector_params["minArea"]
        params.maxArea             = self.blob_detector_params["maxArea"]
        # Filter by Circularity
        params.filterByCircularity = self.blob_detector_params["filterByCircularity"]
        params.minCircularity      = self.blob_detector_params["minCircularity"]
        # Filter by Color. Not directly color, but intensity on the channel
        # input. This filter compares the intensity of a binary image at the
        # center of a blob to blobColor.
        params.filterByColor       = self.blob_detector_params["filterByColor"]
        params.blobColor           = self.blob_detector_params["blobColor"]
        # Filter by convexity
        params.filterByConvexity   = self.blob_detector_params["filterByConvexity"]
        #params.minConvexity        = self.blob_detector_params["minConvexity"]
        #params.maxConvexity        = self.blob_detector_params["maxConvexity"]
        # Filter by inertia
        params.filterByInertia     = self.blob_detector_params["filterByInertia"]
        #params.minInertiaRatio     = self.blob_detector_params["minInertiaRatio"]
        #params.maxInertiaRatio     = self.blob_detector_params["maxInertiaRatio"]

        # Create a detector with the parameters
        ver = (cv2.__version__).split('.')
        if int(ver[0]) < 3 :
            detector = cv2.SimpleBlobDetector(params)
        else :
            detector = cv2.SimpleBlobDetector_create(params)

        return detector	

    def getBestBlob(self, blobs):
        '''
            Track the object

            Parameters:
                blobs = blobs de la imagen

            Returns:
                best_blob = best keypoint or None if didn't meets the filter.
        '''
        # If there are no blobs, there is no better blob
        if not blobs:
            return None
            
        # Criterion: largest and most centered if equal, if not largest
        best_blob = None

        for blob in blobs:
            # Filter blobs by y position
            if blob.pt[1] >= self.blob_detector_ymin:
                print("Current filtered blob:", blob.pt, blob.size)
                # Initialize best blob
                if not best_blob:
                    best_blob = blob
                    continue
                else:
                    # If new blob is bigger
                    if best_blob.size < blob.size:
                        best_blob = blob
                    # If new blob is equal in size but more centered
                    elif (best_blob.size == blob.size) and (abs(best_blob.pt[0] - self.cam_center.x) > abs(blob.pt[0] - self.cam_center.x)):
                        best_blob = blob
        
        # Center blob
        if best_blob:
            best_blob.pt = (best_blob.pt[0] - self.cam_center.x, best_blob.pt[1] - self.cam_center.y)
            #print("\nBest blob found: ", best_blob.pt[0], best_blob.pt[1], best_blob.size)
            
        return best_blob

    def trackObject(self, colorRangeMin, colorRangeMax, showFrame=False):
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
        targetRotationReached = False   # True if the robot is aligned with the object on the x-axis.

        # Initializations
        cam, rawCapture = self.initCamera()
        detector = self.initMyBlobDetector()

        # Transform constraints
        rotation_transform      = Transform(Vector2.zero, CUSTOM_POSITION_ERROR=15)
        outbound_transform_xmin = Transform(Vector2.zero, CUSTOM_POSITION_ERROR=self.xmin_to_rotate) 
        outbound_transform_xmax = Transform(Vector2.zero, CUSTOM_POSITION_ERROR=self.cam_center.x)  
        outbound_transform_y    = Transform(Vector2(x=0, y=self.cam_center.y//4), CUSTOM_POSITION_ERROR=10)
        fixed_position_transform      = Transform(Vector2(x=0, y=self.ymin_to_stop), CUSTOM_POSITION_ERROR=10)
        
        # Object positional information
        side = 1 # Last side the ball was seen (-1 = left, 1 = right).
        backwards_refpos = None
        nextToMe = False
        
        # Main loop
        while True:
            for img in cam.capture_continuous(rawCapture, format="bgr", use_video_port=True):

                # Get robot odometry
                x, y, _, bh = self.readOdometry()

                # Get a new frame
                # https://stackoverflow.com/questions/32522989/opencv-better-detection-of-red-color
                # Negative image + Blue mask
                frame = cv2.bitwise_not(img.array)  # invertir imagen a negativo
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) 
                mask  = cv2.inRange(frame, colorRangeMin, colorRangeMax) 

                # Detect all blobs
                keypoints = detector.detect(mask)

                # Search for the most promising blob...
                best_blob = self.getBestBlob(keypoints)
                
                # Show camera frame if asked
                if showFrame:
                    image = cv2.bitwise_and(frame, frame, mask=mask)
                    image = cv2.drawKeypoints(image, keypoints, np.array([]), (255,255,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
                    cv2.imshow('Captura', image)
                    if cv2.waitKey(1) & 0xff == 27:
                        cam.close()
                rawCapture.truncate(0)  # remove img.array content
                
                # Decide v and w for the robot to get closer to target position
                v = 0; w = 0; wb = 0
                
                if best_blob:
                    # Define transforms (catchment regions)
                    blob_rotation = Transform( Vector2(x=best_blob.pt[0], y=0) )  # Location to rotate to
                    blob_position = Transform( Vector2(x=0, y=best_blob.pt[1]) )  # Location to go to
                    nextToMe = False
                    backwards_refpos = None

                    if not targetRotationReached:
                        # Target rotation
                        if not rotation_transform == blob_rotation:
                            # Last side detected. If sign < 0, the ball is to the left, if sign > 0, to the right
                            side = np.sign(blob_rotation.position.x)
                            # Speeds. If sign < 0 -> w > 0 (-1*sign*w = -1*-1*w), if sign > 0 -> w < 0 (-1*sign*w = -1*1*w)
                            w = self.fw(blob_rotation.position.x)
                        else:
                            targetRotationReached = True

                    # Target position
                    if not fixed_position_transform == blob_position:
                        v = self.fv(blob_position.position.y)
                        # Raise the basket till intial position (0º)
                        wb = int(bh > 5) * -1
                    else:
                        nextToMe = not outbound_transform_xmin == blob_rotation
                        # Object found. Lower the basket 90º
                        if bh < 90:
                            wb = 1
                        else:
                            # The basket has caught the object
                            print("Ball caught")
                            return

                    # Checking y coordinate location of the blob within the catchment region
                    # El objetivo estará alineado con el robot cuando:
                    # 1. Este en el centro de la imagen en x y la zona inferior en y
                    # 2. Este a lo largo de toda la imagen en x y en la ultima franja en y
                    targetRotationReached = (not outbound_transform_y == blob_position and outbound_transform_xmin == blob_rotation) or outbound_transform_xmax == blob_rotation
          
                else: 
                    # Retrocede un par de cm si la pelota estaba al lado de la camara la ultima vez que se vio
                    if nextToMe:
                        if not backwards_refpos:
                            backwards_refpos = Vector2(x,y)
                        elif (Vector2(x,y) - backwards_refpos).magnitude() <= self.backwards_dist:
                            v = -self.fv_max
                        else:
                            nextToMe = False
                    # Rotate until ball found. Set max angular speed.
                    w = side * self.fw_max
                    # If the basket has been lowered, it will be raised again.
                    wb = int(bh > 5) * -1
                    targetRotationReached = False
    
                # Robot speed
                self.setSpeed(v, w, wb)


    #-- Navegacion -------------------------
    def playNavigation(self, rmap, global_ref, start, goal):
        """
            Ejecutar mapa de navegacion cargado
        """
        # 
        self.rmap   = rmap
        gx, gy, gth = [rmap.sizeCell*global_ref[0] + rmap.halfCell, rmap.sizeCell*global_ref[1] + rmap.halfCell, global_ref[2]]
        ltow        = Matrix2.transform(Vector2(gx, gy, 0), gth)
        wtol        = ltow.invert()
        return


    # def playNavigation(self, rMap):
    #     '''
    #         Ejectuar navegacion del mapa cargado
    #     '''
    #     # MAPS & PATH TRAYECTORY
    #     #
    #     self.us_ev3_obstacle = lambda : 0.5 < self.us_ev3.value < (self.rMap.halfCell+5)
    #     #self.us_ev3_stop     = lambda : (10 <= self.us_ev3.value <= 11) or (12 <= self.us_ev3.value <= 13)

    #     # Estado inicial
    #     state = "START_CELL_ADVENTURE"
    #     # Posicion inicial
    #     _, cell, pos = self.rMap.travel()
    #     # Si la celda actual es la meta, hemos terminado
    #     if cell == self.rMap.goal:
    #         print("Goal reached!", cell, self.rMap.goal)
    #         return
    #     # Transformaciones iniciales
    #     #light_error = self.rMap.sizeCell//10
    #     rotation_transform = Transform(Vector2.zero)
    #     #light_position_transform = Transform(Vector2.zero)
    #     fixed_position_transform = Transform(Vector2.zero)
    #     position_reached   = False
    #     dynamic_walls      = []
    #     # Ultrasonic values
    #     us_index  = 0
    #     us_values = [self.us_ev3.value, self.us_ev3.value, self.us_ev3.value, self.us_ev3.value]
    #     # Velocidades
    #     v = 10
    #     w = 0.75

    #     if rMap.neighborhood == 4:
    #         after_recogn = "RECOGN"
    #         # Recorrido del path:
    #         while True:
    #             # Se obtiene la odometria del robot
    #             x, y, th, _ = self.readOdometry()
    #             # Se obtiene la transformacion correspondiente:
    #             # 1. La posicion es local por defecto, hay que transformarla en global.
    #             # 2. La orientacion del robot es local por defecto:
    #             #   - Primero se rota el eje X [1,0] segun la orientacion del robot th en local.
    #             #   - Despues se pasa a global.
    #             gpos        = self.ltow * Vector2(x, y, 1)
    #             gfor        = self.ltow * Vector2.right.rotate(th)
    #             # A. Estado de inicializacion. Obtiene los parametros para la siguiente aventura
    #             if state == "START_CELL_ADVENTURE":
    #                 # Obtenemos los datos de la siguiente celda
    #                 _, next_cell, next_pos = self.rMap.travel()
    #                 print(next_pos, gpos)
    #                 dir = (next_pos - pos).normalize()
    #                 # Obtenemos las transformaciones representativas del destino
    #                 rotation_transform       = Transform(Vector2.zero, forward=dir)
    #                 #light_position_transform = Transform(next_pos, CUSTOM_POSITION_ERROR=light_error)
    #                 fixed_position_transform = Transform(next_pos)
    #                 # Si la rotacion ya coincide, pasamos a reconocimiento
    #                 if rotation_transform == Transform(Vector2.zero, forward=gfor):
    #                     state = "RECOGN"
    #                     print("START_CELL_ADVENTURE -> RECOGN")
    #                 # Si no, primero rotamos el robot
    #                 else:
    #                     state = "ROTATION"
    #                     print("START_CELL_ADVENTURE -> ROTATION")
    #                     self.setSpeed(0, gfor.angle_sense(dir) * w)
        
    #             # B. Estado de rotacion
    #             elif state == "ROTATION":
    #                 transform = Transform(Vector2.zero, forward=gfor)
    #                 if rotation_transform == transform:
    #                     state = "RECOGN"
    #                     print("ROTATION -> RECOGN")
    #                     self.setSpeed(0, 0)

    #             # C. Estado de reconocimiento del entorno
    #             elif state == "RECOGN":
    #                 # A. DESCOMENTAR LA DE ABAJO
    #                 #if after_recogn == "RECOGN":
    #                     # Antes de avanzar, comprobamos si hay obstáculo
    #                     shift         = gfor.normalize()
    #                     dx, dy        = int(round(shift.y)), int(round(shift.x))
    #                     conn          = [2*cell[0]+1, 2*cell[1]+1]
    #                     neighbor_conn = [conn[0]+dx, conn[1]+dy]
    #                     neighbor_left = [neighbor_conn[0]-dy, neighbor_conn[1]-dx]
    #                     neighbor_rght = [neighbor_conn[0]+dy, neighbor_conn[1]+dx]
    #                     wall          = [neighbor_conn, neighbor_rght, neighbor_left]
    #                     #print(gfor, neighbor_left, neighbor_conn, neighbor_rght)
    #                     # Si detecto obstucalo
    #                     if self.us_ev3_obstacle():
    #                         if self.rMap.connectionMatrix[neighbor_conn[0]][neighbor_conn[1]]:
    #                             self.rMap.connectionMatrix[neighbor_conn[0]][neighbor_conn[1]] = 0
    #                             self.rMap.connectionMatrix[neighbor_left[0]][neighbor_left[1]] = 0
    #                             self.rMap.connectionMatrix[neighbor_rght[0]][neighbor_rght[1]] = 0
    #                             self.rMap.replanPath_4N(cell)
    #                             if not self.rMap.path:
    #                                 if dynamic_walls:
    #                                     dynamic_walls.append(wall)
    #                                     # A. COMENTAR LA DE ABAJO
    #                                     state = "BACKTRACKING"
    #                                     # A. DESCOMENTAR LA DE ABAJO
    #                                     #after_recogn = "BACKTRACKING"
    #                                     print("RECOGN -> BACKTRACKING")
    #                                 else:
    #                                     print("RECOGN -> RECOGN")
    #                             else:
    #                                 dynamic_walls.append(wall)
    #                                 # A. COMENTAR LA DE ABAJO
    #                                 state = "START_CELL_ADVENTURE"
    #                                 # A. DESCOMENTAR LA DE ABAJO
    #                                 #after_recogn = "START_CELL_ADVENTURE"
    #                                 print("RECOGN -> START_CELL_ADVENTURE")
    #                         #self.setSpeed(-v/2, 0)
    #                     # Si no detecto obstaculo pero lo habia antes:
    #                     elif not self.rMap.connectionMatrix[neighbor_conn[0]][neighbor_conn[1]]:
    #                         if wall in dynamic_walls:
    #                             dynamic_walls.remove(wall)
    #                         self.rMap.connectionMatrix[neighbor_conn[0]][neighbor_conn[1]]     = 1
    #                         if self.rMap.connectionSource[neighbor_left[0]][neighbor_left[1]]:
    #                             self.rMap.connectionMatrix[neighbor_left[0]][neighbor_left[1]] = 1
    #                         if self.rMap.connectionSource[neighbor_rght[0]][neighbor_rght[1]]:
    #                             self.rMap.connectionMatrix[neighbor_rght[0]][neighbor_rght[1]] = 1
    #                         self.rMap.replanPath_4N(cell)
    #                         if not self.rMap.path:
    #                             if dynamic_walls:
    #                                 dynamic_walls.append(wall)
    #                                 # A. COMENTAR LA DE ABAJO
    #                                 state = "BACKTRACKING"
    #                                 # A. DESCOMENTAR LA DE ABAJO
    #                                 #after_recogn = "BACKTRACKING"
    #                                 print("RECOGN -> BACKTRACKING")
    #                             else:
    #                                 print("RECOGN -> RECOGN")
    #                         else:
    #                             # A. COMENTAR LA DE ABAJO
    #                             state = "START_CELL_ADVENTURE"
    #                             # A. DESCOMENTAR LA DE ABAJO
    #                             #after_recogn = "START_CELL_ADVENTURE"    
    #                             print("RECOGN -> START_CELL_ADVENTURE")
    #                         # A. DESCOMENTAR LA DE ABAJO
    #                         #self.setSpeed(-v/2, 0)
    #                     # Si ni una ni otra, sigo adelante
    #                     else:
    #                         state = "FORWARD"
    #                         print("RECOGN -> FORWARD")
    #                         self.setSpeed(v,0)
    #                 # A. DESCOMENTAR ELSE
    #                 #else:
    #                 #    us_values[us_index] = self.us_ev3.value
    #                 #    us_index = (us_index + 1) % len(us_values)
    #                 #    if Decimal(np.median(us_values)) % Decimal(self.rMap.halfCell) <= 12:
    #                 #        self.setSpeed(0,0)
    #                 #        state = after_recogn
    #                 #        after_recogn = "RECOGN"

    #             # D. Estado de backtracking. Vuelve a un camino anterior
    #             elif state == "BACKTRACKING":
    #                 neighbor_conn, neighbor_rght, neighbor_left = dynamic_walls.pop(0)
    #                 self.rMap.connectionMatrix[neighbor_conn[0]][neighbor_conn[1]]     = 1
    #                 if self.rMap.connectionSource[neighbor_left[0]][neighbor_left[1]] and not any(neighbor_left in c for c in dynamic_walls):
    #                     self.rMap.connectionMatrix[neighbor_left[0]][neighbor_left[1]] = 1
    #                 if self.rMap.connectionSource[neighbor_rght[0]][neighbor_rght[1]] and not any(neighbor_rght in c for c in dynamic_walls):
    #                     self.rMap.connectionMatrix[neighbor_rght[0]][neighbor_rght[1]] = 1
    #                 self.rMap.replanPath_4N(cell)
    #                 if not self.rMap.path:
    #                     if not dynamic_walls:
    #                         state = "RECOGN"
    #                         print("BACKTRACKING -> RECOGN")
    #                     else:
    #                         print("BACKTRACKING -> BACKTRACKING")
    #                 else:
    #                     state = "START_CELL_ADVENTURE"
    #                     print("BACKTRACKING -> START_CELL_ADVENTURE")
                
    #             # E. Estado de avance hacia la siguiente celda
    #             elif state == "FORWARD":
    #                 #gdir = (next_pos - gpos).normalize()
    #                 # Si el vector orientacion del robot no coincide con el vector direccion de 
    #                 # la posicion actual a la siguiente, corrige trayectoria
    #                 if not rotation_transform == Transform(Vector2.zero, forward=gfor):
    #                     #print("MAL")
    #                     self.setSpeed(v, gfor.angle_sense(rotation_transform.forward)*0.25)
    #                 # Si el vector de la posicion del robot a la siguiente posicion no coincide
    #                 # con el vector direccion de la posicion actual a la siguiente, tambie corrige
    #                 #if not rotation_transform == Transform(Vector2.zero, forward=gdir):
    #                 #    self.setSpeed(v, gdir.angle_sense(rotation_transform.forward)*w)
    #                     #print("BIEN")
    #                 # Si ambos coinciden, no necesita aplicar correccion
    #                 else:
    #                     self.setSpeed(v, 0)

    #                 # El ultrasonido esta a unos 15 cm maximos debido a su posicion en el
    #                 # robot, entonces, si haciendo modulo de la mitad de la celda da un
    #                 # valor menor que estos 15 cm maximos, esta en el centro de la celda (+-)
    #                 # B. COMENTAR LA DE ABAJO
    #                 us_cell_center = Decimal(self.us_ev3.value) % Decimal(self.rMap.halfCell) <= 13.5
    #                 # B. DESCOMENTAR LAS 3 DE ABAJO
    #                 us_values[us_index] = self.us_ev3.value
    #                 us_index = (us_index + 1) % len(us_values)
    #                 us_cell_center = True
    #                 for us_value in us_values:
    #                     us_cell_center &= Decimal(us_value) % Decimal(self.rMap.halfCell) <= 15.5
    #                 #us_cell_center = Decimal(np.mean(us_values)) % Decimal(self.rMap.halfCell) <= 16
    #                 # Si la posicion YA ha sido alcanzada o es alcanzada en odometria
    #                 transform = Transform(gpos)
    #                 if position_reached or fixed_position_transform == transform:
    #                     # Si el ultrasonido NO INDICA que sea el centro sigue avanzando
    #                     if not us_cell_center:
    #                         position_reached = True
    #                     else:
    #                         position_reached = False
    #                         # self.x.value = abs(next_pos.y * rotation_transform.forward.y)
    #                         # self.y.value = abs(next_pos.x * rotation_transform.forward.x)
    #                         cell  = next_cell
    #                         pos   = next_pos
    #                         if cell == self.rMap.goal:
    #                             print("Goal reached!: ", cell, self.rMap.goal)
    #                             break
    #                         state = "START_CELL_ADVENTURE"    
    #                         print("FORWARD -> START_CELL_ADVENTURE")
    #                         self.setSpeed(0,0)
    #                 # Si la posicion NO ha sido alcanzada y el ultrasonido TAMPOCO LO INDICA
    #                 # PERO, si se ha movido escasos cm de la posicion actual podria dar por
    #                 # valido esta condicion si se encuentra un muro, entonces se le indica
    #                 # que SOLO tenga en cuenta esto cuando el robot este cerca de la siguiente
    #                 # posicion.
    #                 #elif light_position_transform == transform and us_cell_center:
    #                 #    position_reached = False
    #                 #    cell  = next_cell
    #                 #    pos   = next_pos
    #                 #    if cell == self.rMap.goal:
    #                 #        print("Goal reached!: ", cell, self.rMap.goal)
    #                 #        break
    #                 #    state = "START_CELL_ADVENTURE"    
    #                 #    print("FORWARD -> START_CELL_ADVENTURE")
    #                 #    self.setSpeed(0,0)
    #     else:
    #         changes            = False
    #         sense              = 1
    #         stops              = []
    #         after_recogn = "RECALCULATE_MAP"

    #         while True:
    #             # Se obtiene la odometria del robot
    #             x, y, th, _ = self.readOdometry()
    #             # print(x, y, th)
    #             # Se obtiene la transformacion correspondiente:
    #             # 1. La posicion es local por defecto, hay que transformarla en global.
    #             # 2. La orientacion del robot es local por defecto:
    #             #   - Primero se rota el eje X [1,0] segun la orientacion del robot th en local.
    #             #   - Despues se pasa a global.
    #             gpos        = self.ltow * Vector2(x, y, 1)
    #             gfor        = self.ltow * Vector2.right.rotate(th, format="RAD")
    #             # Estados del camino
    #             # A. Estado de inicializacion. Obtiene los parametros para la siguiente aventura
    #             if state == "START_CELL_ADVENTURE":
    #                 # Obtenemos los datos de la siguiente celda
    #                 _, next_cell, next_pos  = self.rMap.travel()
    #                 dir = (next_pos - pos).normalize()
    #                 # Obtenemos las transformaciones representativas del destino
    #                 rotation_transform = Transform(Vector2.zero, forward=dir)
    #                 fixed_position_transform = Transform(position=next_pos, forward=dir)
    #                 # - Si la rotacion ya coincide, pasamos a reconocimiento
    #                 if rotation_transform == Transform(position=pos, forward=gfor):
    #                     state = "RECOGN"
    #                     print("START_CELL_ADVENTURE -> RECOGN")
    #                 # - Si no, rotamos el robot
    #                 else:
    #                     state = "ROTATION"
    #                     print("START_CELL_ADVENTURE -> ROTATION")
    #                     self.setSpeed(0, gfor.angle_sense(dir) * w)

    #             # B. Estado de rotacion
    #             elif state == "ROTATION":
    #                 transform = Transform(Vector2.zero, forward=gfor)
    #                 if rotation_transform == transform:
    #                     state = "BEGIN_RECOGN"
    #                     print("ROTATION -> BEGIN_RECOGN")
    #                     self.setSpeed(0, 0)

    #             # C. Estado de inicializacion del reconocimiento del entorno
    #             elif state == "BEGIN_RECOGN":
    #                 sense = 1
    #                 stops = [
    #                     Transform(Vector2.zero, th+45),
    #                     Transform(Vector2.zero, th-45),
    #                     Transform(Vector2.zero, th)
    #                 ]
    #                 self.setSpeed(0, sense*w)
    #                 state = "RECOGN"
    #                 print("BEGIN_RECOGN -> RECOGN")

    #             # D. Estado de reconocimiento
    #             elif state == "RECOGN":
    #                 # Antes de avanzar, comprobamos si hay obstáculo
    #                 shift         = gfor.normalize()
    #                 dx, dy        = int(round(shift.y)), int(round(shift.x))
    #                 conn          = [2*cell[0]+1, 2*cell[1]+1]
    #                 neighbor_conn = [conn[0] + dx, conn[1] + dy]
    #                 neighbor_left = [neighbor_conn[0]-(1-dx), neighbor_conn[1]-(1-dy)]
    #                 neighbor_rght = [neighbor_conn[0]+(1-dx), neighbor_conn[1]+(1-dy)]  
    #                 wall          = [neighbor_conn, neighbor_rght, neighbor_left]
    #                 # Si detecto obstaculo
    #                 if self.us_ev3_obstacle():
    #                     if self.rMap.connectionMatrix[neighbor_conn[0]][neighbor_conn[1]]:
    #                         changes = True
    #                         self.rMap.connectionMatrix[neighbor_conn[0]][neighbor_conn[1]] = 0
    #                         self.rMap.connectionMatrix[neighbor_left[0]][neighbor_left[1]] = 0
    #                         self.rMap.connectionMatrix[neighbor_rght[0]][neighbor_rght[1]] = 0
    #                         dynamic_walls.append(wall)
    #                 # Si no detecto obstaculo pero lo habia antes:
    #                 elif not self.rMap.connectionMatrix[neighbor_conn[0]][neighbor_conn[1]]:
    #                     changes = True
    #                     if wall in dynamic_walls:
    #                         dynamic_walls.remove(wall)
    #                     self.rMap.connectionMatrix[neighbor_conn[0]][neighbor_conn[1]]     = 1
    #                     if self.rMap.connectionSource[neighbor_left[0]][neighbor_left[1]]:
    #                         self.rMap.connectionSource[neighbor_left[0]][neighbor_left[1]] = 1
    #                     if self.rMap.connectionSource[neighbor_rght[0]][neighbor_rght[1]]:
    #                         self.rMap.connectionSource[neighbor_rght[0]][neighbor_rght[1]] = 1
    #                 # Rota hasta la siguiente posicion de barrido
    #                 if stops[0] == Transform(Vector2.zero, forward=gfor):
    #                     stops.pop(0)
    #                     if not stops:
    #                         if changes:
    #                             state = "RECALCULATE_MAP"
    #                             print("RECOGN -> RECALCULATE_MAP")
    #                             self.setSpeed(0, 0)
    #                         else:
    #                             state = "FORWARD"
    #                             print("RECOGN -> FORWARD")
    #                             self.setSpeed(v, 0)
    #                     else:
    #                         sense *= -1
    #                         self.setSpeed(0, sense*w)

    #             # E. Recalcular mapa:
    #             elif state == "RECALCULATE_MAP":
    #                 # C. DESCOMENTAR LA DE ABAJO
    #                 #if after_recogn == "RECALCULATE_MAP":
    #                     self.rMap.replanPath_8N()
    #                     if not self.rMap.path:
    #                         if not dynamic_walls and len(dynamic_walls) == 1:
    #                             dynamic_walls = []
    #                             # C. DESCOMENTAR LA DE ABAJO
    #                             #after_recogn = "BEGIN_RECOGN"
    #                             # C. COMENTAR LA DE ABAJO
    #                             state = "BEGIN_RECOGN"
    #                             print("RECALCULATE_MAP -> BEGIN_RECOGN")
    #                         else:
    #                             # C. DESCOMENTAR LA DE ABAJO
    #                             #after_recogn = "BACKTRACKING"
    #                             # C. COMENTAR LA DE ABAJO
    #                             state = "BACKTRACKING"
    #                             print("RECALCULATE_MAP -> BACKTRACKING")
    #                     else:
    #                         #after_recogn = "START_CELL_ADVENTURE"
    #                         state = "START_CELL_ADVENTURE"
    #                         print("RECALCULATE_MAP -> START_CELL_ADVENTURE")
    #                     # C. DESCOMENTAR LA DE ABAJO
    #                     #self.setSpeed(-v/2, 0)
    #                 # C. DESCOMENTAR ELSE
    #                 #else:
    #                 #    us_values[us_index] = self.us_ev3.value
    #                 #    us_index = (us_index + 1) % len(us_values)
    #                 #    if Decimal(np.median(us_values)) % Decimal(self.rMap.halfCell) <= 12:
    #                 #        self.setSpeed(0,0)
    #                 #        state = after_recogn
    #                 #        after_recogn = "RECALCULATE_MAP"

    #             # F. Estado de backtracking. Vuelve a un camino anterior
    #             elif state == "BACKTRACKING":
    #                 neighbor_conn, neighbor_rght, neighbor_left = dynamic_walls.pop(0)
    #                 self.rMap.connectionMatrix[neighbor_conn[0]][neighbor_conn[1]]     = 1
    #                 if self.rMap.connectionSource[neighbor_left[0]][neighbor_left[1]] and not any(neighbor_left in c for c in dynamic_walls):
    #                     self.rMap.connectionMatrix[neighbor_left[0]][neighbor_left[1]] = 1
    #                 if self.rMap.connectionSource[neighbor_rght[0]][neighbor_rght[1]] and not any(neighbor_rght in c for c in dynamic_walls):
    #                     self.rMap.connectionMatrix[neighbor_rght[0]][neighbor_rght[1]] = 1
    #                 self.rMap.replanPath_8N(cell)
    #                 if not self.rMap.path:
    #                     if not dynamic_walls:
    #                         state = "BEGIN_RECOGN"
    #                         print("BACKTRACKING -> BEGIN_RECOGN")
    #                     else:
    #                         print("BACKTRACKING -> BACKTRACKING")
    #                 else:
    #                     state = "START_CELL_ADVENTURE"
    #                     print("BACKTRACKING -> START_CELL_ADVENTURE")

    #             # G. Estado de avance hacia la siguiente celda
    #             elif state == "FORWARD":
    #                 #gdir = (next_pos - gpos).normalize()
    #                 # Si el vector orientacion del robot no coincide con el vector direccion de 
    #                 # la posicion actual a la siguiente, corrige trayectoria
    #                 if not rotation_transform == Transform(Vector2.zero, forward=gfor):
    #                     #print("MAL")
    #                     self.setSpeed(v, gfor.angle_sense(rotation_transform.forward)*0.25)
    #                 # Si el vector de la posicion del robot a la siguiente posicion no coincide
    #                 # con el vector direccion de la posicion actual a la siguiente, tambie corrige
    #                 #if not rotation_transform == Transform(Vector2.zero, forward=gdir):
    #                 #    self.setSpeed(v, gdir.angle_sense(rotation_transform.forward)*w)
    #                     #print("BIEN")
    #                 # Si ambos coinciden, no necesita aplicar correccion
    #                 else:
    #                     self.setSpeed(v, 0)

    #                 # El ultrasonido esta a unos 15 cm maximos debido a su posicion en el
    #                 # robot, entonces, si haciendo modulo de la mitad de la celda da un
    #                 # valor menor que estos 15 cm maximos, esta en el centro de la celda (+-)
    #                 # D. COMENTAR LA DE ABAJO
    #                 us_cell_center = Decimal(self.us_ev3.value) % Decimal(self.rMap.halfCell) <= 13.5
    #                 # D. DESCOMENTAR LAS 3 DE ABAJO
    #                 us_values[us_index] = self.us_ev3.value
    #                 us_index = (us_index + 1) % len(us_values)
    #                 us_cell_center = True
    #                 for us_value in us_values:
    #                     us_cell_center &= Decimal(us_value) % Decimal(self.rMap.halfCell) <= 15.5
    #                 #us_cell_center = Decimal(np.mean(us_values)) % Decimal(self.rMap.halfCell) <= 16
    #                 # Si la posicion YA ha sido alcanzada o es alcanzada en odometria
    #                 transform = Transform(gpos)
    #                 if position_reached or fixed_position_transform == transform:
    #                     # Si el ultrasonido NO INDICA que sea el centro sigue avanzando
    #                     if not us_cell_center:
    #                         position_reached = True
    #                     else:
    #                         position_reached = False
    #                         cell  = next_cell
    #                         pos   = next_pos
    #                         if cell == self.rMap.goal:
    #                             print("Goal reached!: ", cell, self.rMap.goal)
    #                             break
    #                         state = "START_CELL_ADVENTURE"    
    #                         print("FORWARD -> START_CELL_ADVENTURE")
    #                         self.setSpeed(0,0)
    #                 # Si la posicion NO ha sido alcanzada y el ultrasonido TAMPOCO LO INDICA
    #                 # PERO, si se ha movido escasos cm de la posicion actual podria dar por
    #                 # valido esta condicion si se encuentra un muro, entonces se le indica
    #                 # que SOLO tenga en cuenta esto cuando el robot este cerca de la siguiente
    #                 # posicion.
    #                 #elif light_position_transform == transform and us_cell_center:
    #                 #    position_reached = False
    #                 #    cell  = next_cell
    #                 #    pos   = next_pos
    #                 #    if cell == self.rMap.goal:
    #                 #        print("Goal reached!: ", cell, self.rMap.goal)
    #                 #        break
    #                 #    state = "START_CELL_ADVENTURE"    
    #                 #    print("FORWARD -> START_CELL_ADVENTURE")
    #                 #    self.setSpeed(0,0)
