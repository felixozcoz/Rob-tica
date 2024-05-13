#!/usr/bin/python
# -*- coding: UTF-8 -*-
from __future__ import print_function # use python 3 syntax but make it compatible with python 2
from __future__ import division

import brickpi3     # Import the BrickPi3 drivers
import csv          # Write csv
import cv2 as cv    # Computer vision operations
import datetime     # Timestamping
import numpy as np  # Mathematical operations
import picamera     # Camera usage
import sys          # 
import time         # Sleep function
from decimal import Decimal
                    # For module operations with decimals
from geometry import Vector2, Matrix2, Transform
                    # For geometrical operations
from matplotlib import pyplot as plt
                    # To plot a graph
from multiprocessing import Process, Value, Array, Lock
                    # Threading package could be used too
from picamera.array import PiRGBArray
                    # To manage raw photos from camera
from ReMapLib import Map
                    # To load and play a map
from scipy.interpolate import PchipInterpolator
                    # To interpolate a set of points and obtain a trajectory

# Robot class
class Robot:
    def __init__(self, local_ref=[0.0, 0.0, 0.0], resolution=(320, 240), framerate=32):
        """
        Initialize basic robot params. \

        Initialize Motors and Sensors according to the set up in your robot
        """
        # Robot construction parameters
        self.R             = 2.8               # Wheels' radius (cm)
        self.L             = 15.25             # Wheels' axis length (cm)
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
        self.us_ev3 = Value('d', 0.0)            # Latest distances ultrasonic EV3 sensor stored

        self.BP.set_sensor_type(self.PORT_ULTRASONIC_NXT, self.BP.SENSOR_TYPE.NXT_ULTRASONIC)
        self.us_nxt = Value('d', 0.0)

        # Configure color sensor
        self.PORT_COLOR = self.BP.PORT_3
        # self.BP.set_sensor_type(self.PORT_COLOR, self.BP.SENSOR_TYPE.NXT_LIGHT_ON)
        self.light = 0
        for _ in range(3):
            # self.light = self.BP.get_sensor(self.PORT_COLOR)
            self.light = 0.0
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
        # Odometria
        self.lock_odometry = Lock()            # Mutex
        self.P             = 0.01              # Update period (in seconds)
        self.finished      = Value('b', 1)     # Boolean to show if odometry updates are finished
        ##################################################
        # Localizacion
        self.x  = Value('d', local_ref[0])
                                               # Robot X coordinate.
        self.y  = Value('d', local_ref[1])
                                               # Robot Y coordinate.
        self.th = Value('d', local_ref[2])
                                               # Robot orientation.
        self.bh = Value('d', 0)                # Robot basket angle [0 to ~90º].
        self.sD = Value('i', 0)                # Latest stored RIGHT encoder value.
        self.sI = Value('i', 0)                # Latest stored LEFT encoder value.
        self.sC = Value('i', 0)                # Latest stored BASKET encoder value.
        self.v  = Value('d', 0.0)              # Robot linear theorical speed (or tangential if rotating).
        self.w  = Value('d', 0.0)              # Robot angular theorical speed.
        self.wi = Value('d', 0.0)              # Robot left wheel theorical speed.
        self.wd = Value('d', 0.0)              # Robot right wheel theorical speed.
        self.wb = Value('d', 0.0)              # Robot basket theorical angular speed.
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

    def getLight(self):
        """
        Get the light value from the color sensor.
        """
        return self.BP.get_sensor(self.PORT_COLOR)

    #-- Odometria --------------------------
    def startOdometry(self):
        """ This starts a new process/thread that will be updating the odometry periodically """
        self.finished.value = False
        self.p = Process(target=self.updateOdometry, args=())
        self.p.start()
        # for _ in range(3):
        #     self.us_nxt.value = self.BP.get_sensor(self.PORT_ULTRASONIC_NXT)
        #     time.sleep(self.P)
        for _ in range(3):
            self.us_ev3.value = self.BP.get_sensor(self.PORT_ULTRASONIC_EV3)
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

                #Theta with Gyros
                try:
                    th, _ = self.BP.get_sensor(self.PORT_GYROSCOPE)
                except Exception:
                    print("Gyroscope error")
                    continue
                # Update odometry uses values that require mutex, they are declared as value, so lock
                # is implicitly done for atomic operations (BUT =+ is NOT atomic)
                # Calculate the arc of circumfrence traveled by each wheel
                left_encoder, right_encoder, basket_encoder, _, _, _, v, w = self.readSpeed()

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
                self.us_ev3.value   = self.BP.get_sensor(self.PORT_ULTRASONIC_EV3)
                try:
                    right_sensor = self.BP.get_sensor(self.PORT_ULTRASONIC_NXT)
                except Exception:
                    print("Lateral sensor error")
                else:
                    self.us_nxt.value   = right_sensor
                    
                self.lock_odometry.release()
                # Aquí sale bien (print correcto) pero en playTrajectory da 0
                # print(self.us_ev3.value)
                # Save LOG
                LOG_WRITER.writerow([datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"), round(self.x.value, 4), round(self.y.value, 4), round(self.th.value, 4)])

                ######## UPDATE UNTIL HERE with your code ########
                tEnd = time.clock()
                time.sleep(self.P - (tEnd-tIni))
            LOG.close()

        #print("Stopping odometry ... X= %d" %(self.x.value))
        sys.stdout.write("Stopping odometry ... X=  %.2f, \
                Y=  %.2f, th=  %.2f \n" %(self.x.value, self.y.value, self.th.value))
        self.BP.reset_all()

    def stopOdometry(self):
        """ Stop the odometry thread """
        self.finished.value = True


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


    #-- Generacion de trayectorias ---------
    def playTrajectory(self, trajectory_points, segments, showPlot=False):
        # . Separar coordenadas de la trayectoria
        x = [point[0] for point in trajectory_points]
        y = [point[1] for point in trajectory_points]
        # . (DEPRECATED) Funcion interpolada respecto de los
        #  puntos dados mediante interpolacion polinomica
        # deg          = len(x)-1
        # coefficients = np.polyfit(x,y,deg)
        # trajectory   = np.poly1d(coefficients)
        # . Funcion interpolada respecto de los puntos dados
        #  mediante PCHIP (quita los minimos y maximos que 
        #  agrega la polinomica)
        trajectory = PchipInterpolator(x, y)
        x_values = np.linspace(min(x), max(x), segments)
        y_values = trajectory(x_values)
        if showPlot:
            plt.figure(figsize=(8,6))
            plt.plot(x_values, y_values, label="Polinomio interpolado", color="blue")
            plt.scatter(x, y, label="Puntos conocidos", color="red")
            plt.xlabel("x")
            plt.ylabel("y")
            plt.title("Interpolacion polinomica")
            plt.legend()
            plt.axis("equal")
            plt.grid(True)
            plt.show()

        # Recorrer trayectoria
        segment  = 1
        state    = "START_SEGMENT_ADVENTURE"
        position = Vector2(x_values[0], y_values[0], 1)
        position_transform = Transform(Vector2.zero)
        #last_transform = Transform(Vector2(x_values[-1], y_values[-1]), 0)
        rotation_reached = False
        us_ev3_values = [self.us_ev3.value, self.us_ev3.value, self.us_ev3.value]
        #us_nxt_values = [self.us_nxt.value, self.us_nxt.value, self.us_nxt.value]
        # Velocidades
        v = 10

        while True:
            # Leer odometria
            x, y, th, _ = self.readOdometry()
            forward = Vector2.right.rotate(th)
            # Estados
            if state == "START_SEGMENT_ADVENTURE":
                next_position      = Vector2(x_values[segment], y_values[segment])
                direction          = next_position - position
                rotation_transform = Transform(Vector2.zero, forward=direction)
                rotation_reached   = False
                position_transform = Transform(next_position, 0, CUSTOM_POSITION_ERROR=0.5)
                sense  = forward.sense(direction)
                w      = forward.angle(direction, "RAD")
                self.setSpeed(v, sense * w)
                state = "GO"
                print("START_SEGMENT_ADVENTURE -> GO")
            elif state == "GO":
                if not rotation_reached:
                    if rotation_transform == Transform(Vector2.zero, forward=forward):
                        rotation_reached = True
                        self.setSpeed(v, 0)
                else:
                    if not rotation_transform == Transform(Vector2.zero, forward=forward):
                        self.setSpeed(v, sense*w)

                if position_transform == Transform(Vector2(x,y), 0):
                    segment += 1
                    if segment >= segments:
                        # Orientar el robot a th 0
                        rotation_transform = Transform(Vector2.zero, rotation=th)
                        while not rotation_transform == Transform(Vector2.zero, rotation=0.0):
                            us_ev3_values = us_ev3_values[1:] + [self.us_ev3.value]
                            x, y, th, _ = self.readOdometry()
                            rotation_transform = Transform(Vector2.zero, rotation=th)
                            self.setSpeed(0, -np.sign(th) * 0.5)
                        self.setSpeed(0, 0)
                        break
                        
                    else:
                        position = next_position
                        state    = "START_SEGMENT_ADVENTURE"
                        print(segment, "GO -> START_SEGMENT_ADVENTURE")

                
    def centerRobot(self, side):
        # Centrar el robot en x en la celda con la distancia al muro de delante
        us_ev3_values = [self.us_ev3.value, self.us_ev3.value, self.us_ev3.value]
        us_nxt_values = [self.us_nxt.value, self.us_nxt.value, self.us_nxt.value]
        while True:
            distance = np.mean(us_ev3_values) % self.rmap.sizeCell
            us_ev3_values = us_ev3_values[1:] + [self.us_ev3.value]
            us_nxt_values = us_nxt_values[1:] + [self.us_nxt.value]
            # print(distance)
            if distance < 13.5:
                self.setSpeed(-3,0)
            elif distance > 14.5:
                self.setSpeed(3,0)
            else:
                self.setSpeed(0,0)
                break

        # Rotar el robot hacia el muro lateral (izda o dcha)
        _, _, th, _ = self.readOdometry()
        odom_rotation_transform = Transform(Vector2.zero, rotation=th)
        rotation_transform = Transform(Vector2.zero, rotation=side*90)
        while not rotation_transform == odom_rotation_transform:
            us_ev3_values = us_ev3_values[1:] + [self.us_ev3.value]
            self.setSpeed(0, side*0.5)
            _, _, th, _ = self.readOdometry()
            odom_rotation_transform = Transform(Vector2.zero, rotation=th)

        # Centrar el robot en y en la celda con la distancia al muro lateral
        while True:
            distance = np.mean(us_ev3_values) % self.rmap.sizeCell
            us_ev3_values = us_ev3_values[1:] + [self.us_ev3.value]
            us_nxt_values = us_nxt_values[1:] + [self.us_nxt.value]
            # print(distance)
            if distance < 14.5:
                self.setSpeed(-3,0)
            elif distance > 15.5:
                self.setSpeed(3,0)
            else:
                self.setSpeed(0,0)
                break
        
        # Rotar el robot hacia su orientacion inicial
        _, _, th, _ = self.readOdometry()
        odom_rotation_transform = Transform(Vector2.zero, rotation=th)
        rotation_transform = Transform(Vector2.zero, rotation=0.0)
        while not rotation_transform == odom_rotation_transform:
            us_ev3_values = us_ev3_values[1:] + [self.us_ev3.value]
            self.setSpeed(0, -side*0.5)
            _, _, th, _ = self.readOdometry()
            odom_rotation_transform = Transform(Vector2.zero, rotation=th)

        # Actualizar odometria
        pos = [self.rmap.start[1] * self.rmap.sizeCell + self.rmap.halfCell, self.rmap.start[0] * self.rmap.sizeCell + self.rmap.halfCell]
        lpos = self.wtol * Vector2(pos[0], pos[1], 1)
        self.lock_odometry.acquire()
        self.x.value = lpos.x
        self.y.value = lpos.y
        self.lock_odometry.release()
        x, y, th, _ = self.readOdometry()
        print("Robot centered at", x, y, th)

    #-- Seguimiento de objetos -------------
    def initCamera(self, resolution):
        '''
            Initialize the camera

            Returns:
                cam = camera object
                rawCapture = object to manage the camera
        '''
        # Init camera
        # self.cam = picamera.PiCamera()
        # self.cam.resolution = self.resolution # default (640, 480)
        # self.cam.framerate  = self.framerate   # default 32
        # self.rawCapture = PiRGBArray(self.cam, size=self.resolution)
        cam = picamera.PiCamera()
        cam.resolution = resolution # default (640, 480)
        cam.framerate = self.framerate   # default 32
        rawCapture = PiRGBArray(cam, size=resolution)

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
        params = cv.SimpleBlobDetector_Params()

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
        ver = (cv.__version__).split('.')
        if int(ver[0]) < 3 :
            detector = cv.SimpleBlobDetector(params)
        else :
            detector = cv.SimpleBlobDetector_create(params)

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
        cam, rawCapture = self.initCamera((320, 240))
        detector = self.initMyBlobDetector()

        # Transform constraints
        rotation_transform          = Transform(Vector2.zero, CUSTOM_POSITION_ERROR=15)
        outbound_transform_xmin     = Transform(Vector2.zero, CUSTOM_POSITION_ERROR=self.xmin_to_rotate) 
        outbound_transform_xmax     = Transform(Vector2.zero, CUSTOM_POSITION_ERROR=self.cam_center.x)  
        outbound_transform_y        = Transform(Vector2(x=0, y=self.cam_center.y//4), CUSTOM_POSITION_ERROR=10)
        fixed_position_transform    = Transform(Vector2(x=0, y=self.ymin_to_stop), CUSTOM_POSITION_ERROR=10)
        
        # Object positional information
        side = 1 # Last side the ball was seen (-1 = left, 1 = right).
        backwards_refpos = None
        nextToMe = False
        
        # Main loop
        while True:
            # for img in self.cam.capture_continuous(self.rawCapture, format="bgr", use_video_port=True):
            for img in cam.capture_continuous(rawCapture, format="bgr", use_video_port=True):

                # Get robot odometry
                x, y, _, bh = self.readOdometry()
                # Get a new frame
                # https://stackoverflow.com/questions/32522989/opencv-better-detection-of-red-color
                # Negative image + Blue mask
                frame = cv.bitwise_not(img.array)  # invertir imagen a negativo
                frame = cv.cvtColor(frame, cv.COLOR_BGR2HSV) 
                mask  = cv.inRange(frame, colorRangeMin, colorRangeMax) 

                # Detect all blobs
                keypoints = detector.detect(mask)

                # Search for the most promising blob...
                best_blob = self.getBestBlob(keypoints)
                
                # Show camera frame if asked
                if showFrame:
                    image = cv.bitwise_and(frame, frame, mask=mask)
                    image = cv.drawKeypoints(image, keypoints, np.array([]), (255,255,255), cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
                    cv.imshow('Captura', image)
                    if cv.waitKey(1) & 0xff == 27:
                        cam.close()
                rawCapture.truncate(0)  # remove img.array content
                
                # Decide v and w for the robot to get closer to target position
                v  = 0
                w  = 0
                wb = 0
                
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
                            cam.close()
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
                    w = (side * self.fw_max)
                    # If the basket has been lowered, it will be raised again.
                    wb = int(bh > 5) * -1
                    targetRotationReached = False
    
                # Robot speed
                self.setSpeed(v, w, wb)

    def drawMatches(img1, kp1, img2, kp2, matches, color=None, thickness = 2, mask=None): 
        """
            Similar to drawMatches in newer versions of open CV
            Draws lines between matching keypoints (kp1, kp2) of the two input images
            color and thickness: line plot properties
            matches: n x Match_objects
            mask: n x bool. List of booleans to indicate which matches should be displayed 
        """
        # We're drawing them side by side.  Get dimensions accordingly.
        # Handle both color and grayscale images.
        if len(img1.shape) == 3:
            new_shape = (max(img1.shape[0], img2.shape[0]), img1.shape[1]+img2.shape[1], img1.shape[2])
        elif len(img1.shape) == 2:
            new_shape = (max(img1.shape[0], img2.shape[0]), img1.shape[1]+img2.shape[1])
        new_img = np.zeros(new_shape, type(img1.flat[0]))  
        # Place images onto the new image.
        new_img[0:img1.shape[0],0:img1.shape[1]] = img1
        new_img[0:img2.shape[0],img1.shape[1]:img1.shape[1]+img2.shape[1]] = img2
        # Draw lines between matches.
        if color:
            c = color
        for i, m in enumerate(matches):
            if mask is None or (mask is not None and mask[i]):            
                # Generate random color for RGB/BGR and grayscale images as needed.
                if not color: 
                    c = np.random.randint(0,256,3) if len(img1.shape) == 3 else np.random.randint(0,256)
                p1 = tuple(np.round(kp1[m.queryIdx].pt).astype(int))
                p2 = tuple(np.round(kp2[m.trainIdx].pt).astype(int) + np.array([img1.shape[1], 0]))
                cv.line(new_img, p1, p2, c, thickness)
        return new_img

    def matchObject(self, img_ref, showMatches=False):
        cam, rawCapture = self.initCamera((1312, 976))
        MIN_MATCH_COUNT = 20       # initially
        MIN_MATCH_OBJECTFOUND = 15 # after robust check, to consider object-found
        # Por eficiencia, que lo haga una sola vez.
        fst_img = cv.cvtColor(img_ref, cv.COLOR_BGR2GRAY)

        while True:
            # self.cam.capture(self.rawCapture, format="bgr")
            cam.capture(rawCapture, format="bgr")
            # frame = cv.flip(rawCapture.array, -1)
            frame = rawCapture.array
            cv.imwrite("frame.jpg", frame)
            # Feature extractor uses grayscale images
            snd_img = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
            # Create a detector with the parameters
            ver = (cv.__version__).split('.')
            if int(ver[0]) < 3: # CURRENT RASPBERRY opencv version is 2.4.9
                # Initiate ORB detector --> you could use any other detector, but this is the best performing one in this version
                binary_features = True
                detector = cv.ORB()
            else: 
                # Initiate BRISK detector --> you could use any other detector, including NON binary features (SIFT, SURF)
                # but this is the best performing one in this version
                binary_features=True
                detector = cv.BRISK_create()
            
            # Find the keypoints and corresponding descriptors
            fst_kp, fst_des = detector.detectAndCompute(fst_img, None)
            snd_kp, snd_des = detector.detectAndCompute(snd_img, None)
            if fst_des is None or snd_des is None:
                print("WARNING -- Empty detection?")
                cam.close()
                return False
            if len(fst_des) < MIN_MATCH_COUNT or len(snd_des) < MIN_MATCH_COUNT:
                print("WARNING -- Not enough features (FST: %d, SND: %d)" % (len(fst_des), len(snd_des)))
                return False
                cam.close()
            print (" FEATURES extracted (FST: %d, SND: %d)" % (len(fst_des), len(snd_des)))

            # Matching
            if binary_features:
                bf = cv.BFMatcher(cv.NORM_HAMMING, crossCheck=True)
                matches = bf.match(fst_des, snd_des)
                good    = sorted(matches, key=lambda x:x.distance)
            else:
                FLANN_INDEX_KDTREE = 0
                index_params  = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
                search_params = dict(checks=50)
                flann         = cv.FlannBasedMatcher(index_params, search_params)
                matches       = flann.knnMatch(fst_des, snd_des, k=2)
                # Store all the good matches as per Lowe's ratio test
                good          = []
                for m,n in matches:
                    if m.distance < 0.7*n.distance:
                        good.append(m)
            print(" Initial matches found: %d" %(len(good)))

            # Find coincidence
            if len(good) > MIN_MATCH_COUNT:
                src_pts = np.float32([ fst_kp[m.queryIdx].pt for m in good]).reshape(-1,1,2)
                dst_pts = np.float32([ snd_kp[m.trainIdx].pt for m in good]).reshape(-1,1,2)
                H_21, mask = cv.findHomography(src_pts, dst_pts, cv.RANSAC, 3.0)
                matches_mask = mask.ravel().tolist()
                num_robuts_matches = np.sum(matches_mask)
                # if num_robuts_matches < MIN_MATCH_OBJECTFOUND:
                #     print(" NOT enough ROBUST matches - %d (required %d)" % (num_robuts_matches, MIN_MATCH_OBJECTFOUND))
                #     return False
                h,w = fst_img.shape
                pts = np.float32([[0,0], [0,h-1], [w-1,h-1], [w-1,0]]).reshape(-1,1,2)
                dst = cv.perspectiveTransform(pts, H_21)
                cv.polylines(frame, [np.int32(dst)], True, color=(255,255,255), thickness=3)
                if (showMatches):
                    if int(ver[0]) < 3:
                        res = self.drawMatches(img_ref, fst_kp, frame, snd_kp, good, color=(0,255,0), mask=matches_mask)
                    else:
                        draw_params = {
                            "matchColor": (0,255,0),
                            "singlePointColor": None,
                            "matchesMask": matches_mask,
                            "flags": 2
                        }
                        res = cv.drawMatches(img_ref, fst_kp, frame, snd_kp, good, None, **draw_params)
                    if res.shape[1] > 1920 or res.shape[0] > 1080:
                        res = cv.resize(res, (int(res.shape[1]/2.5), int(res.shape[0]/2.5)), interpolation=cv.INTER_LINEAR)
                    cv.imshow("INLIERS", res)
                    cv.waitKey(0)
                if num_robuts_matches < MIN_MATCH_OBJECTFOUND:
                    print(" NOT enough ROBUST matches - %d (required %d)" % (num_robuts_matches, MIN_MATCH_OBJECTFOUND))
                    cam.close()
                    return False
        
                print(" ROBUST matches found - %d (out of %d) --> OBJECT FOUND!" % (np.sum(matches_mask), len(good)))
                cam.close()
                return True
            else:
                print(" Not enough initial matches are found - %d (required %d)" % (len(good), MIN_MATCH_COUNT))
                cam.close()
                return False


    #-- Navegacion -------------------------
    def loadMap(self, rmap, rmap_ref):
        '''
            Cargar un mapa en el robot
        '''
        self.rmap = rmap
        self.gx   = rmap_ref[0]
        self.gy   = rmap_ref[1]
        self.gth  = rmap_ref[2]
        self.ltow = Matrix2.transform(Vector2(self.gx, self.gy, 0), self.gth)
        self.wtol = self.ltow.invert()

    def playMap(self):
        '''
            Navegacion a traves de un mapa cargado
        '''
        # Estado inicial
        state = "START_CELL_ADVENTURE"
        # Posicion inicial
        _, cell, pos = self.rmap.travel()
        # Si la celda actual es la meta, hemos terminado
        if cell == self.rmap.goal:
            print("Goal Reached!", cell, self.rmap.goal)
            return
        # Variables
        dynamic_walls = []
        # Velocidades
        v = 10
        w = 0.75
        # Valores de los ultrasonidos
        us_ev3_values = [self.us_ev3.value, self.us_ev3.value, self.us_ev3.value]
        us_nxt_values = [self.us_nxt.value, self.us_nxt.value, self.us_nxt.value]
        
        if self.rmap.neighborhood == 4:
            # Recorrido del camino encontrado
            while True:
                # Se obtiene la odometria del robot
                x, y, th, _ = self.readOdometry()
                # Se obtiene la transformacion correspondiente
                # 1. La posicion es local por defecto, hay que transformarla en global.
                # 2. La orientacion del robot es local por defecto:
                #   - Primero se rota el eje X [1,0] segun la orientacion del robot th en local.
                #   - Despues se pasa a global.
                gpos = self.ltow * Vector2(x,y,1)
                gfor = self.ltow * Vector2.right.rotate(th)
                transform = Transform(gpos, forward=gfor)

                # A. Estado de inicializacion. Obtiene los parametros para la siguiente aventura
                if state == "START_CELL_ADVENTURE":
                    # Extraer siguiente posición
                    _, next_cell, next_pos = self.rmap.travel()
                    dir = (next_pos - pos).normalize()
                    # Obtenemos las transformaciones representativas del destino
                    rotation_transform      = Transform(pos, forward=dir)
                    #entering_cell_transform = Transform(next_pos - self.rmap.halfCell*dir, forward=dir)
                    reaching_cell_transform = Transform(next_pos, forward=dir)
                    # Si la rotacion ya coincide, pasamos a reconocimiento
                    if rotation_transform == transform:
                        state = "RECOGN"
                        print("START_CELL_ADVENTURE -> RECOGN")
                    # Si no, primero rotamos el robot
                    else:
                        state = "ROTATION"
                        print("START_CELL_ADVENTURE -> ROTATION")
                        self.setSpeed(0, gfor.sense(dir) * w)
                # B. Estado de rotacion
                elif state == "ROTATION":
                    if rotation_transform == transform:
                        state = "RECOGN"
                        print("ROTATION -> RECOGN")
                        print("Odometry:", x, y, th)
                        self.setSpeed(0, 0)

                # C. Estado de reconomiento del entorno
                elif state == "RECOGN":
                    # Antes de avanzar, comprobamos si hay obstucalo
                    shift         = gfor.normalize()
                    dx, dy        = int(round(shift.y)), int(round(shift.x))
                    conn          = [2*cell[0]+1, 2*cell[1]+1]
                    neighbor_conn = [conn[0]+dx, conn[1]+dy]
                    neighbor_left = [neighbor_conn[0]-dy, neighbor_conn[1]-dx]
                    neighbor_rght = [neighbor_conn[0]+dy, neighbor_conn[1]+dx]
                    wall          = [neighbor_conn, neighbor_rght, neighbor_left]
                    # Si detecto obstucalo
                    if 0.5 < self.us_ev3.value < (self.rmap.halfCell+5):
                        if self.rmap.connectionMatrix[neighbor_conn[0]][neighbor_conn[1]]:
                            self.rmap.connectionMatrix[neighbor_conn[0]][neighbor_conn[1]] = 0
                            self.rmap.connectionMatrix[neighbor_left[0]][neighbor_conn[1]] = 0
                            self.rmap.connectionMatrix[neighbor_rght[0]][neighbor_rght[1]] = 0
                            self.rmap.replanPath_4N(cell)
                            if not self.rmap.path:
                                if dynamic_walls:
                                    dynamic_walls.append(wall)
                                    state = "BACKTRACKING"
                                    print("RECOGN -> BACKTRACKING")
                                else:
                                    print("RECOGN -> RECOGN")
                            else:
                                dynamic_walls.append(wall)
                                state = "START_CELL_ADVENTURE"
                                print("RECOGN -> START_CELL_ADVENTURE")
                    # Si no detecto obstucalo pero lo habia antes
                    elif not self.rmap.connectionMatrix[neighbor_conn[0]][neighbor_conn[1]]:
                        if wall in dynamic_walls:
                            dynamic_walls.remove(wall)
                        self.rmap.connectionMatrix[neighbor_conn[0]][neighbor_conn[1]]     = 1
                        if self.rmap.connectionSource[neighbor_left[0]][neighbor_left[1]]:
                            self.rmap.connectionMatrix[neighbor_left[0]][neighbor_left[1]] = 1
                        if self.rmap.connectionSource[neighbor_rght[0]][neighbor_rght[1]]:
                            self.rmap.connectionMatrix[neighbor_rght[0]][neighbor_rght[1]] = 1
                        self.rmap.replanPath_4N(cell)
                        if not self.rmap.path:
                            if dynamic_walls:
                                dynamic_walls.append(wall)
                                state = "BACKTRACKING"
                                print("RECOGN -> BACKTRACKING")
                            else:
                                print("RECOGN -> RECOGN")
                        else:
                            state = "START_CELL_ADVENTURE"
                            print("RECOGN -> START_CELL_ADVENTURE")
                    else:
                        state = "FORWARD"
                        print("RECOGN -> FORWARD")
                        self.setSpeed(v, 0)

                # D. Estado de backtracking. Vuelve a un camino anterior
                elif state == "BACKTRACKING":
                    neighbor_conn, neighbor_rght, neighbor_left = dynamic_walls.pop(0)
                    self.rmap.connectionMatrix[neighbor_conn[0]][neighbor_conn[1]]     = 1
                    if self.rmap.connectionSource[neighbor_left[0]][neighbor_left[1]] and not any(neighbor_left in c for c in dynamic_walls):
                        self.rmap.connectionMatrix[neighbor_left[0]][neighbor_left[1]] = 1
                    if self.rmap.connectionSource[neighbor_rght[0]][neighbor_rght[1]] and not any(neighbor_rght in c for c in dynamic_walls):
                        self.rmap.connectionMatrix[neighbor_rght[0]][neighbor_rght[1]] = 1
                    self.rmap.replanPath_4N(cell)
                    if not self.rmap.path:
                        if not dynamic_walls:
                            state = "RECOGN"
                            print("BACKTRACKING -> RECOGN")
                        else:
                            print("BACKTRACKING -> BACKTRACKING")
                    else:
                        state = "START_CELL_ADVENTURE"
                        print("BACKTRACKING -> START_CELL_ADVENTURE")

                # E. Estado de avance hacia la siguiente celda
                elif state == "FORWARD":
                    # Si el vector orientacion del robot no coincide con el vector direccion de 
                    # la posicion actual a la siguiente, corrige trayectoria
                    if not rotation_transform == Transform(Vector2.zero, forward=gfor):
                        self.setSpeed(v, gfor.sense(rotation_transform.forward)*0.15)
                    else:
                        self.setSpeed(v, 0)
                    # Obtenemos los datos del ultrasonido
                    us_position_reached = Decimal(np.mean(us_ev3_values)) % Decimal(self.rmap.sizeCell) <= 14
                    us_ev3_values = us_ev3_values[1:] + [self.us_ev3.value]
                    us_nxt_values = us_nxt_values[1:] + [self.us_nxt.value]
                    # Si la posicion coincide, he llegado a la celda
                    if reaching_cell_transform == transform:
                        # Si el ultrasonido no indica que sea el centro, sigue avanzando
                        if us_position_reached:
                            self.setSpeed(0, 0)
                            # Si ha llegado al centro y era la ultima celda, termina
                            print("next_cell", next_cell, " goal", self.rmap.goal)
                            if next_cell == self.rmap.goal:
                                print("Goal Reached!: ", next_cell, self.rmap.goal)
                                break
                            # Si no, avanzamos a la siguiente celda
                            else:
                                # Se actualiza a la siguiente celda
                                cell  = next_cell
                                pos   = next_pos
                                # Se actualiza la odometria
                                lpos = self.wtol * pos
                                self.lock_odometry.acquire()
                                self.x.value = lpos.x
                                self.y.value = lpos.y
                                self.lock_odometry.release()
                                # Siguiente estado
                                state = "START_CELL_ADVENTURE"
                                print("FORWARD -> START_CELL_ADVENTURE")
                                self.setSpeed(0, 0)

        elif self.rmap.neighborhood == 8:
            # Variables extra para la 8 vecindad
            changes            = False
            sense              = 1
            stops              = []
            # Recorrido del camino encontrado
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
                transform = Transform(gpos, forward=gfor)
                # Estados del camino
                # A. Estado de inicializacion. Obtiene los parametros para la siguiente aventura
                if state == "START_CELL_ADVENTURE":
                    # Obtenemos los datos de la siguiente celda
                    _, next_cell, next_pos  = self.rmap.travel()
                    dir = (next_pos - pos).normalize()
                    # Obtenemos las transformaciones representativas del destino
                    rotation_transform      = Transform(pos, forward=dir)
                    entering_cell_transform = Transform(next_pos, forward=dir)
                    reaching_cell_transform = Transform(next_pos - self.rmap.halfCell*dir, forward=dir)
                    # - Si la rotacion ya coincide, pasamos a reconocimiento
                    if rotation_transform == Transform(position=pos, forward=gfor):
                        state = "RECOGN"
                        print("START_CELL_ADVENTURE -> RECOGN")
                    # - Si no, rotamos el robot
                    else:
                        state = "ROTATION"
                        print("START_CELL_ADVENTURE -> ROTATION")
                        self.setSpeed(0, gfor.angle_sense(dir) * w)

                # B. Estado de rotacion
                elif state == "ROTATION":
                    transform = Transform(Vector2.zero, forward=gfor)
                    if rotation_transform == transform:
                        state = "BEGIN_RECOGN"
                        print("ROTATION -> BEGIN_RECOGN")
                        self.setSpeed(0, 0)

                # C. Estado de inicializacion del reconocimiento del entorno
                elif state == "BEGIN_RECOGN":
                    sense = 1
                    stops = [
                        Transform(gpos, th+45),
                        Transform(gpos, th-45),
                        Transform(gpos, th)
                    ]
                    state = "RECOGN"
                    print("BEGIN_RECOGN -> RECOGN")
                    self.setSpeed(0, sense*w)

                # D. Estado de reconocimiento
                elif state == "RECOGN":
                    # Antes de avanzar, comprobamos si hay obstáculo
                    shift         = gfor.normalize()
                    dx, dy        = int(round(shift.y)), int(round(shift.x))
                    conn          = [2*cell[0]+1, 2*cell[1]+1]
                    neighbor_conn = [conn[0] + dx, conn[1] + dy]
                    neighbor_left = [neighbor_conn[0]-(1-dx), neighbor_conn[1]-(1-dy)]
                    neighbor_rght = [neighbor_conn[0]+(1-dx), neighbor_conn[1]+(1-dy)]  
                    wall          = [neighbor_conn, neighbor_rght, neighbor_left]
                    # Si detecto obstaculo
                    if 0.5 < self.us_ev3.value < (self.rmap.halfCell+5):
                        if self.rmap.connectionMatrix[neighbor_conn[0]][neighbor_conn[1]]:
                            changes = True
                            self.rmap.connectionMatrix[neighbor_conn[0]][neighbor_conn[1]] = 0
                            self.rmap.connectionMatrix[neighbor_left[0]][neighbor_left[1]] = 0
                            self.rmap.connectionMatrix[neighbor_rght[0]][neighbor_rght[1]] = 0
                            dynamic_walls.append(wall)
                    # Si no detecto obstaculo pero lo habia antes:
                    elif not self.rmap.connectionMatrix[neighbor_conn[0]][neighbor_conn[1]]:
                        changes = True
                        if wall in dynamic_walls:
                            dynamic_walls.remove(wall)
                        self.rmap.connectionMatrix[neighbor_conn[0]][neighbor_conn[1]]     = 1
                        if self.rmap.connectionSource[neighbor_left[0]][neighbor_left[1]]:
                            self.rmap.connectionSource[neighbor_left[0]][neighbor_left[1]] = 1
                        if self.rmap.connectionSource[neighbor_rght[0]][neighbor_rght[1]]:
                            self.rmap.connectionSource[neighbor_rght[0]][neighbor_rght[1]] = 1
                    # Rota hasta la siguiente posicion de barrido
                    if stops[0] == transform:
                        stops.pop(0)
                        if not stops:
                            if changes:
                                state = "RECALCULATE_MAP"
                                print("RECOGN -> RECALCULATE_MAP")
                                self.setSpeed(0, 0)
                            else:
                                state = "FORWARD"
                                print("RECOGN -> FORWARD")
                                self.setSpeed(v, 0)
                        else:
                            sense *= -1
                            self.setSpeed(0, sense*w)

                # E. Recalcular mapa:
                elif state == "RECALCULATE_MAP":
                    self.rmap.replanPath_8N()
                    if not self.rmap.path:
                        if not dynamic_walls and len(dynamic_walls) == 1:
                            dynamic_walls = []
                            state = "BEGIN_RECOGN"
                            print("RECALCULATE_MAP -> BEGIN_RECOGN")
                        else:
                            state = "BACKTRACKING"
                            print("RECALCULATE_MAP -> BACKTRACKING")
                    else:
                        state = "START_CELL_ADVENTURE"
                        print("RECALCULATE_MAP -> START_CELL_ADVENTURE")

                # F. Estado de backtracking. Vuelve a un camino anterior
                elif state == "BACKTRACKING":
                    neighbor_conn, neighbor_rght, neighbor_left = dynamic_walls.pop(0)
                    self.rmap.connectionMatrix[neighbor_conn[0]][neighbor_conn[1]]     = 1
                    if self.rmap.connectionSource[neighbor_left[0]][neighbor_left[1]] and not any(neighbor_left in c for c in dynamic_walls):
                        self.rmap.connectionMatrix[neighbor_left[0]][neighbor_left[1]] = 1
                    if self.rmap.connectionSource[neighbor_rght[0]][neighbor_rght[1]] and not any(neighbor_rght in c for c in dynamic_walls):
                        self.rmap.connectionMatrix[neighbor_rght[0]][neighbor_rght[1]] = 1
                    self.rmap.replanPath_8N(cell)
                    if not self.rmap.path:
                        if not dynamic_walls:
                            state = "BEGIN_RECOGN"
                            print("BACKTRACKING -> BEGIN_RECOGN")
                        else:
                            print("BACKTRACKING -> BACKTRACKING")
                    else:
                        state = "START_CELL_ADVENTURE"
                        print("BACKTRACKING -> START_CELL_ADVENTURE")

                # G. Estado de avance hacia la siguiente celda
                elif state == "FORWARD":
                    # Si el vector orientacion del robot no coincide con el vector direccion de 
                    # la posicion actual a la siguiente, corrige trayectoria
                    if not rotation_transform == Transform(Vector2.zero, forward=gfor):
                        self.setSpeed(v, gfor.sense(rotation_transform.forward)*0.15)
                    else:
                        self.setSpeed(v, 0)
                    # Obtenemos los datos del ultrasonido
                    us_position_reached = Decimal(np.mean(us_ev3_values)) % Decimal(self.rmap.sizeCell) <= 14
                    us_ev3_values = us_ev3_values[1:] + [self.us_ev3.value]
                    us_nxt_values = us_nxt_values[1:] + [self.us_nxt.value]
                    # Si la posicion coincide, he llegado a la celda
                    if reaching_cell_transform == transform:
                        # Si el ultrasonido no indica que sea el centro, sigue avanzando
                        if us_position_reached:
                            self.setSpeed(0, 0)
                            # Si ha llegado al centro y era la ultima celda, termina
                            if next_cell == self.rmap.goal:
                                print("Goal Reached!: ", next_cell, self.rmap.goal)
                                break
                            # Si no, avanzamos a la siguiente celda
                            else:
                                # Se actualiza a la siguiente celda
                                cell  = next_cell
                                pos   = next_pos
                                # Se actualiza la odometria
                                lpos = self.wtol * pos
                                self.lock_odometry.acquire()
                                self.x.value = lpos.x
                                self.y.value = lpos.y
                                self.lock_odometry.release()
                                # Siguiente estado
                                state = "START_CELL_ADVENTURE"
                                print("FORWARD -> START_CELL_ADVENTURE")
                                self.setSpeed(0, 0)

