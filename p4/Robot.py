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
from geometry import Vector2, Matrix2, Transform
from ReMapLib import Map

# Threading package could be used too
from multiprocessing import Process, Value, Array, Lock

class Robot:
    def __init__(self, local_reference=[0.0, 0.0, 0.0], global_reference=[0.0, 0.0, 0.0], resolution=(320, 240), framerate=32, rMap:Map=None):
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
        # Configure gyroscope
        self.PORT_GYROSCOPE = self.BP.PORT_4
        self.BP.set_sensor_type(self.PORT_GYROSCOPE, self.BP.SENSOR_TYPE.EV3_GYRO_ABS_DPS)
        # Configure motors
        self.PORT_LEFT_MOTOR  = self.BP.PORT_D
        self.PORT_RIGHT_MOTOR = self.BP.PORT_A
        self.PORT_BASKET_MOTOR = self.BP.PORT_B
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
        self.us_nxt = Value('d', 0)            # Latest distances ultrasonic NXT sensor stored
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
        
        # MAPS & PATH TRAYECTORY
        self.rMap = rMap
        self.gx   = global_reference[0]
        self.gy   = global_reference[1]
        self.gth  = global_reference[2]
        self.ltow = Matrix2.transform(Vector2(self.gx, self.gy, 0), self.gth)
        print(self.ltow)
    def setSpeed(self, v: float, w: float, wb: float):
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

    def readGyroscope(self):
        """ Returns current value of gyroscope estimation values: 
                [ absolute angle turned from startup,  rotational speed ]
        """
        return self.BP.get_sensor(self.PORT_GYROSCOPE)

    def readUltrasonicSensors(self):
        """ Returns current value of ultrasonic sensors estimation values"""
        return self.us_ev3 #, self.us_nxt
    
    def updateUltrasonicSensors(self):
        """ This function updates the ultrasonic sensors values """
        # update ultrasonic sensors value

        self.us_ev3 = self.BP.get_sensor(self.PORT_ULTRASONIC_EV3) 
        #self.us_nxt = self.BP.get_sensor(self.PORT_ULTRASONIC_NXT)  

    def startOdometry(self):
        """ This starts a new process/thread that will be updating the odometry periodically """

        self.finished.value = False
        self.p = Process(target=self.updateOdometry, args=())
        self.p.start()

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
                th, _ = self.readGyroscope()

                th_normalized = np.deg2rad(-th) % (2 * np.pi)
                # Luego, ajustas el ángulo al intervalo [-π, π):
                if th_normalized > np.pi:
                    th_normalized -= 2 * np.pi

                #print(np.rad2deg(th_normalized), w)
                # Calculate real delta th from gyroscope
                #delta_th = w * self.P
                delta_th = th_normalized - self.th.value
                th = th_normalized
                bh = self.bh.value  + basket_encoder - self.sC.value
                # Calculate delta xWR. Calculate delta s (depends on w) (slide 14)
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
                # update odometry
                self.x.value += delta_x
                self.y.value += delta_y
                self.th.value = th
                # update motors values
                self.bh.value = bh
                self.sD.value = right_encoder
                self.sI.value = left_encoder
                self.sC.value = basket_encoder
                # update ultrasonic sensors value
                # self.updateUltrasonicSensors()
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
        targetRotationReached = False   # True if the robot is aligned with the object on the x-axis.

        # Initializations
        cam, rawCapture = self.initCamera()
        detector = self.initMyBlobDetector()

        # Transform constraints
        rotation_transform      = Transform(Vector2.zero, CUSTOM_POSITION_ERROR=15)
        outbound_transform_xmin = Transform(Vector2.zero, CUSTOM_POSITION_ERROR=self.xmin_to_rotate) 
        outbound_transform_xmax = Transform(Vector2.zero, CUSTOM_POSITION_ERROR=self.cam_center.x)  
        outbound_transform_y    = Transform(Vector2(x=0, y=self.cam_center.y//4), CUSTOM_POSITION_ERROR=10)
        position_transform      = Transform(Vector2(x=0, y=self.ymin_to_stop), CUSTOM_POSITION_ERROR=10)
        
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
                    if not position_transform == blob_position:
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


    def detectObstacle(self, orig, dest):
        """ Detecta si hay un obstáculo en la trayectoria entre dos celdas del mapa y devuelve sus coordenadas """
        # TODO: Para obstaculo en diagonal (en 8-vecindad) habrá que rotar hacia la esquina y luego
        # girar el robot hacia los lados y comprobar si hay obstáculo a una distancia con margen un
        # poco más amplio
        error = 0.5
        sensor_pared = 18
        self.us_ev3 = self.readUltrasonicSensors()
        conn, coords = self.rMap.areConnected(orig, dest)
        return self.us_ev3 > sensor_pared - error and self.us_ev3 < sensor_pared + error and conn, coords            

    def playTrayectory(self):
        # Estado inicial
        state = "RECOGN"
        # Posicion inicial
        _, cell, previous_pos = self.rMap.travel()
        # Si la celda actual es la meta, hemos terminado
        if cell == self.rMap.goal:
            print("Goal reached!: ", cell, self.rMap.goal)
            return
        # Velocidades
        v = 10
        w = 0.8
        # Transform constraints
        rotation_transform = Transform(Vector2.zero, CUSTOM_POSITION_ERROR=2)
        position_transform = Transform(Vector2.zero, CUSTOM_POSITION_ERROR=2)
        sense              = 1
        stops              = []

        dinamic_walls = []
        while True:
            # Se obtiene la odometria del robot
            x, y, th, _ = self.readOdometry()
            # Se obtiene la transformacion correspondiente:
            # 1. La posicion es local por defecto, hay que transformarla en global.
            # 2. La orientacion del robot es local por defecto:
            #   - Primero se rota el eje X [1,0] segun la orientacion del robot th en local.
            #   - Despues se pasa a global.
            gpos        = self.ltow * Vector2(x, y, 1)
            gfor        = self.ltow * Vector2.right.rotate(th, format="RAD")
            print("------------------")
            print(state)
            print("xy1: " + str(Vector2(x, y, 1)))
            print("gpos: " + str(gpos))
            print("gfor: " + str(gfor))
            print([x,y,th])
            # Estado de reconocimiento del entorno
            # Voy a echar una mano
            if state == "RECOGN":
                # sense = 1
                # stops = [Transform(gpos, th+90), Transform(gpos, th-90), Transform(gpos, th)]
                # self.setSpeed(0, w, 0)
                # state = "RECOGN_ROTATE"
            # elif state == "FULL_RECOGN":
                # sense = 1
                # stops = [Transform(gpos, th-5)]
                # self.setSpeed(0, w, 0)
                # state = "BEGIN_RECOGN"
            # elif state == "BEGIN_RECOGN":
                # Se usa el sensor y se actualiza el mapa SI ES NECESARIO
                # Tenemos gfor => 
                # La orientacion global nos dice hacia que celda mira
                # shift = gfor.normalize()
                # Obtenemos la direccion de la conexion segun donde mira el robot
                # Invertimos ya que la matriz va al reves
                # 
                # Por ejemplo:           En el array es
                #    ^ y_global = [0,1]  [cell_x-1][cell_y-1] [cell_x-1][cell_y] [cell_x-1][cell_y+1]
                #    R                   [cell_x  ][cell_y-1] [cell_x  ][cell_y] [cell_x  ][cell_y+1]
                #                        [cell_x+1][cell_y-1] [cell_x  ][cell_y] [cell_x+1][cell_y+1]
                #
                # Ademas, dx corresponde a 'y' y dy a 'x' ya que el mapa es
                #   ^ oy
                #   . > ox  y en la matrix la dimensiones son [x][y] no [y][x]
                # dx, dy = -int(gfor.y), int(gfor.x)
                # 
                # Uso el sensor y guardo su valor
                # Si detecto obstaculo
                #  self.rMap.conectionMatrix(cell + dx, cell + dy) = 0
                # 
                # if not stops:
                #   state == "RECALCULATE_MAP"
                # elif stops[0] == transform:
                #   stops.pop(0)
                #   sense *= -1
                #   self.setSpeed(0, sense*w, 0)
            # elif state == "RECALCULATE_MAP":
                # Se recalcula el mapa como sea 
                # ...
                # if not self.rMap.path:
                    # state = "FULL_RECOGN"
                # else:
                    # state = "TRAVEL_PATH"
            # elif state == "TRAVEL_PATH":
                if self.rMap.path:
                    # Obtenemos el siguiente destino
                    _, cell, next_pos = self.rMap.travel()
                    # Obtenemos las transformaciones representativas del destino
                    destination_dir    = (next_pos - previous_pos).normalize()
                    print("prev: " + str(previous_pos))
                    print("next: " + str(next_pos))
                    print("dest: " + str(destination_dir))
                    rotation_transform = Transform(Vector2.zero, forward=destination_dir, CUSTOM_ROTATION_ERROR=0.5)
                    position_transform = Transform(next_pos, CUSTOM_POSITION_ERROR=0.05)
                    # Si la rotacion ya coincide, nos ahorramos una iteracion
                    transform = Transform(Vector2.zero, forward=gfor)
                    print("transf_for: " + str(transform.forward))
                    print("rot_transf_for: " + str(rotation_transform.forward))
                    if not transform == rotation_transform:
                        state = "ROTATE"
                        self.setSpeed(0, transform.forward.cross(destination_dir) * w, 0)
                    else:
                        # # Antes de avanzar, comprobamos si hay obstáculo
                        # obs, obs_coords = self.detectObstacle(previous_pos, next_pos)
                        # if obs:
                        #     # Si hay obstáculo, se marca la conexión como no válida
                        #     self.rMap.deleteConnection(obs_coords)
                        #     # Guardamos la pared en la lista de paredes dinámicas
                        #     dinamic_walls.append(obs_coords)
                        #     if abs(rotation_transform.forward.x) == 1:
                        #         # Si el obstáculo está en x, se marca la conexión en y
                        #         self.rMap.deleteConnection([obs_coords[0], obs_coords[1] + 1])
                        #         self.rMap.deleteConnection([obs_coords[0], obs_coords[1] - 1])
                        #     elif abs(rotation_transform.forward.y) == 1:
                        #         # Si el obstáculo está en y, se marca la conexión en x
                        #         self.rMap.deleteConnection([obs_coords[0] + 1, obs_coords[1]])
                        #         self.rMap.deleteConnection([obs_coords[0] - 1, obs_coords[1]])
                        #     else:
                        #         # TODO: Si el obstáculo está en diagonal
                        #         # Depende de si el obstáculo está en la adyacente o en la siguiente
                        #         a = 0
                        #     # Y  recalculamos los costes y el camino
                        #     self.rMap.replanPath(cell, previous_pos)
                        #     continue
                        # else:
                        #     state = "FORWARD"
                        #     self.setSpeed(v, 0, 0)

                        # Avanzar sin mirar obstáculos
                        state = "FORWARD"
                        self.setSpeed(v, 0, 0)
                    previous_pos = next_pos
                else:
                    self.setSpeed(0, -w, 0)
            # Estado de rotacion en la celda
            elif state == "ROTATE":
                transform = Transform(Vector2.zero, forward=gfor)
                print(transform.rotation)
                print(rotation_transform.rotation)
                if rotation_transform == transform:
                    state = "FORWARD"
                    self.setSpeed(v, 0, 0)
            elif state == "FORWARD":
                transform = Transform(gpos)
                print(transform.position)
                print(position_transform.position)
                if position_transform == transform:
                    # Si la celda actual es la meta, hemos terminado
                    if cell == self.rMap.goal:
                        print("Goal reached!: ", cell, self.rMap.goal)
                        break
                    state = "RECOGN"
                    self.setSpeed(0, 0, 0)
