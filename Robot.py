#!/usr/bin/python
# -*- coding: UTF-8 -*-
from __future__ import print_function # use python 3 syntax but make it compatible with python 2
from __future__ import division       #                           ''

import brickpi3 # import the BrickPi3 drivers
import time     # import the time library for the sleep function
import sys
import numpy as np

# tambien se podria utilizar el paquete de threading
from multiprocessing import Process, Value, Array, Lock

class Robot:
    def __init__(self, init_position=[0.0, 0.0, 0.0]):
        """
        Initialize basic robot params. \

        Initialize Motors and Sensors according to the set up in your robot
        """

######## UNCOMMENT and FILL UP all you think is necessary (following the suggested scheme) ########

        # Robot construction parameters
        self.R = 0.03 # inventado, mirar medida correcta (uds en metros)
        self.L = 0.152 # longitud del eje de las ruedas (uds en metros)
        # self. ...

        ##################################################
        # Motors and sensors setup

        # Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.
        self.BP = brickpi3.BrickPi3()

        # Configure sensors, for example a touch sensor.
        #self.BP.set_sensor_type(self.BP.PORT_1, self.BP.SENSOR_TYPE.TOUCH)

        # reset encoder B and C (or all the motors you are using)
        self.BP.offset_motor_encoder(self.BP.PORT_B, self.BP.get_motor_encoder(self.BP.PORT_B))
        self.BP.offset_motor_encoder(self.BP.PORT_C, self.BP.get_motor_encoder(self.BP.PORT_C))

        ##################################################
        # odometry shared memory values
        self.x = Value('d',0.0)
        self.y = Value('d',0.0)
        self.sD = Value('i',0) # last encoder value stored motor port A (right motor)
        self.sI= Value('i',0) # last encoder value stored motor port D (left motor)
        self.sC = Value('i',0) # last encoder value stored motor cesta
        self.v = Value('d',0.0) # linear speed 
        self.w = Value('d',0.0) # angular speed
        self.th = Value('d',0.0) # last thita gyro value
        self.finished = Value('b',1) # boolean to show if odometry updates are finished

        # if we want to block several instructions to be run together, we may want to use an explicit Lock
        self.lock_odometry = Lock()

        # odometry update period
        self.P = 0.1 # in seconds


    def setSpeed(self, v, w):

        # calculate motors angular speeds 
        A = np.array([[1/self.R,   self.L/(2*self.R)],
                      [1/self.R, - self.L/(2*self.R)]])
        vc  = np.array([[v],[w]])
        w_motors = np.dot(A, vc) # angular speed: [wd, wi]

        # set speed power
        speedPower = 100
        self.BP.set_motor_power(self.BP.PORT_A + self.BP.PORT_D, speedPower)

        # set each motor speed
        self.BP.set_motor_dps(self.BP.PORT_D, w_motors[1]) # left
        self.BP.set_motor_dps(self.BP.PORT_A, w_motors[0]) # right

        # store speed data (really necessary?)
        #self.lock_odometry.acquire()
        #self.v.value = v
        #self.w.value = w
        #self.lock_odometry.release()


    def readSpeed(self):
        return self.vc.value # mutex unnecesary
        """ To be filled"""


    def readOdometry(self):
        """ Returns current value of odometry estimation """
        return self.x.value, self.y.value, self.th.value

    def startOdometry(self):
        """ This starts a new process/thread that will be updating the odometry periodically """
        self.finished.value = False
        self.p = Process(target=self.updateOdometry, args=()) #additional_params?))
        self.p.start()
        print("PID: ", self.p.pid)

    # You may want to pass additional shared variables besides the odometry values and stop flag
    def updateOdometry(self): #, additional_params?):
        """ To be filled ...  """

        while not self.finished.value:
            # current processor time in a floating point value, in seconds
            tIni = time.clock()

            # compute updates

            # update odometry uses values that require mutex
            # (they are declared as value, so lock is implicitly done for atomic operations, BUT =+ is NOT atomic)

            try:
                # Each of the following BP.get_motor_encoder functions returns the encoder value
                # (what we want to store).
                sys.stdout.write("Reading encoder values .... \n")
                [encoder1, encoder2] = [self.BP.get_motor_encoder(self.BP.PORT_D),
                                        self.BP.get_motor_encoder(self.BP.PORT_A)]
            except IOError as error:
                #print(error)
                sys.stdout.write(error)
            

            # Operations like += which involve a read and write are not atomic.
            with self.x.get_lock():
                self.x.value+=1

            # calculate the arc of circumference traveled by each wheel
            si = (np.deg2rad(encoder1) - self.sI.value) * self.R # faltaría tratar caso en que se resetee el encoder
            sd = (np.deg2rad(encoder2) - self.sD.value) * self.R # En grados o radianes?
            diff_s = (sd - si) / 2 
            # extract gyroscope data 
            [th_gyro, w_gyro] = self.BP.get_sensor(self.BP.PORT_1)
            th_gyro = np.deg2rad(th_gyro) # convert to radians
            th_gyro = np.arctan2(np.sin(th_gyro), np.cos(th_gyro))
            diff_th = th_gyro - self.th.value
            # calculate the increment of x and y
            diff_x = diff_s * np.cos(th_gyro + (diff_th/2))
            diff_y = diff_s * np.sin(th_gyro + (diff_th/2))

            # to "lock" a whole set of operations, we can use a "mutex"
            self.lock_odometry.acquire()
            self.x.value += diff_x  
            self.y.value += diff_y
            self.th.value = th_gyro
            self.sI.value = np.deg2rad(encoder1)
            self.sD.value = np.deg2rad(encoder2)
            self.lock_odometry.release()

            #sys.stdout.write("Encoder (%s) increased (in degrees) B: %6d  C: %6d " %
            #        (type(encoder1), encoder1, encoder2))


            # save LOG
            # Need to decide when to store a log with the updated odometry ...

            ######## UPDATE UNTIL HERE with your code ########


            tEnd = time.clock()
            time.sleep(self.P - (tEnd-tIni))

        #print("Stopping odometry ... X= %d" %(self.x.value))
        sys.stdout.write("Stopping odometry ... X=  %.2f, \
                Y=  %.2f, th=  %.2f \n" %(self.x.value, self.y.value, self.th.value))

    # Stop the odometry thread.
    def stopOdometry(self):
        self.finished.value = True
        #self.BP.reset_all()

