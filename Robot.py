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
        self.th = Value('d',0.0)
        self.sA = Value('i',0) # last encoder value stored motor port A
        self.sB = Value('i',0) # last encoder value stored motor port B
        self.sC = Value('i',0) # last encoder value stored motor cesta
        self.v = Value('d',0.0) # linear speed 
        self.w = Value('d',0.0) # angular speed
        self.finished = Value('b',1) # boolean to show if odometry updates are finished

        # if we want to block several instructions to be run together, we may want to use an explicit Lock
        self.lock_odometry = Lock()

        # odometry update period
        self.P = 0.1 # in seconds


    def setSpeed(self, v, w):

        # calcular velocidad angular para cada motor
        A = np.array([[1/self.R,   self.L/(2*self.R)],
                      [1/self.R, - self.L/(2*self.R)]])
        vc  = np.array([v],[w])
        w_motors = np.dot(A, vc) # angular speed: [wd, wi]

        # set speed power
        speedPower = 100
        self.BP.set_motor_power(self.BP.PORT_B + self.BP.PORT_C, speedPower)

        # set each motor speed
        self.BP.set_motor_dps(self.BP.PORT_B, w_motors[1]) # left
        self.BP.set_motor_dps(self.BP.PORT_C, w_motors[0]) # right

        # store speed data (really necessary?)
        self.lock_odometry.acquire()
        self.v.value = v
        self.w.value = w
        self.lock_odometry.release()


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
            ######## UPDATE FROM HERE with your code (following the suggested scheme) ########

            # update odometry uses values that require mutex
            # (they are declared as value, so lock is implicitly done for atomic operations, BUT =+ is NOT atomic)

            try:
                # Each of the following BP.get_motor_encoder functions returns the encoder value
                # (what we want to store).
                sys.stdout.write("Reading encoder values .... \n")
                [encoder1, encoder2] = [self.BP.get_motor_encoder(self.BP.PORT_B),
                                        self.BP.get_motor_encoder(self.BP.PORT_C)]
            except IOError as error:
                #print(error)
                sys.stdout.write(error)
            

            # Operations like += which involve a read and write are not atomic.
            with self.x.get_lock():
                self.x.value+=1

            # 
            sa = (np.deg2rad(self.sA.value - encoder1)) * self.R # faltaría tratar caso en que se resetee el encoder
            sb = (np.deg2rad(self.sB.value - encoder2)) * self.R

            # Si el valor de sA es menor que el ultimo valor recogido, hay que hacer el modulo:
            # (self.sA.value - encoder1) % max siendo max = valor antes de poner sA a 0.



            # to "lock" a whole set of operations, we can use a "mutex"
            self.lock_odometry.acquire()
            self.x.value = self.R * ( np.sin(self.th.value) )  #xk
            self.y.value = self.R * ( 1 - np.cos(self.th.value) )  # yk
            self.th.value = self.w.value * self.P  # MCU
    
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

