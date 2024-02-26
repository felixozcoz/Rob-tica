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
        self.R = 2.8 # radio de las ruedas (uds en cm)
        self.L = 15.2 # longitud del eje de las ruedas (uds en cm)
        # self. ...

        ##################################################
        # Motors and sensors setup

        # Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.
        self.BP = brickpi3.BrickPi3()

        # Configure sensors, for example a touch sensor.                                              
        self.BP.set_sensor_type(self.BP.PORT_1, self.BP.SENSOR_TYPE.EV3_GYRO_ABS_DPS)
        #self.BP.set_sensor_type(self.BP.PORT_1, self.BP.SENSOR_TYPE.TOUCH)

        self.PORT_LEFT_MOTOR = self.BP.PORT_C
        self.PORT_RIGHT_MOTOR = self.BP.PORT_B

        # reset encoder B and C (or all the motors you are using)
        self.BP.offset_motor_encoder(self.PORT_RIGHT_MOTOR, self.BP.get_motor_encoder(self.PORT_RIGHT_MOTOR))
        self.BP.offset_motor_encoder(self.PORT_LEFT_MOTOR, self.BP.get_motor_encoder(self.PORT_LEFT_MOTOR))

        ##################################################
        # odometry shared memory values
        self.x = Value('d',0.0)
        self.y = Value('d',0.0)
        self.sD = Value('i',0) # last encoder value stored motor right motor
        self.sI = Value('i',0) # last encoder value stored motor left motor
        self.sC = Value('i',0) # last encoder value stored motor cesta
        self.v = Value('d',0.0) # linear speed 
        self.w = Value('d',0.0) # angular speed
        self.th = Value('d',0.0) 
        self.finished = Value('b',1) # boolean to show if odometry updates are finished

        # Se pueden borrar alegremente cuando ya no se necesiten (son de prueba)
        self.totalLength = Value('d',0.0)

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

        #print("Velocidad: %.5f, Velocidad Angular: %.5f" %(v, w))
        #print("Rueda derecha (Init): %.2f, Rueda izquierda (Init): %.2f" %(w_motors[0], w_motors[1]))

        # set each motor speed
        self.BP.set_motor_dps(self.PORT_RIGHT_MOTOR, np.rad2deg(w_motors[0])) 
        self.BP.set_motor_dps(self.PORT_LEFT_MOTOR, np.rad2deg(w_motors[1]))

        # store speed data (really necessary?)
        #self.lock_odometry.acquire()
        #self.v.value = v
        #self.w.value = w
        #self.lock_odometry.release()


    def readSpeed(self):
        return self.v.value, self.w.value

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

        while not self.finished.value:
            # current processor time in a floating point value, in seconds
            tIni = time.clock()

            # compute updates

            # update odometry uses values that require mutex
            # (they are declared as value, so lock is implicitly done for atomic operations, BUT =+ is NOT atomic)

            try:
                # Each of the following BP.get_motor_encoder functions returns the encoder value
                # (what we want to store).
                #sys.stdout.write("Reading encoder values .... \n")
                [left_encoder, right_encoder] = [self.BP.get_motor_encoder(self.PORT_LEFT_MOTOR),
                                                 self.BP.get_motor_encoder(self.PORT_RIGHT_MOTOR)]
            except IOError as error:
                #print(error)
                sys.stdout.write(error)

            # Calculate the arc of circumfrence traveled by each wheel
            left_offset         = left_encoder - self.sI.value
            left_offset_length  = np.deg2rad(left_offset) * self.R
            right_offset        = right_encoder - self.sD.value
            right_offset_length = np.deg2rad(right_offset) * self.R
            
            # calculate real speed
            wi = np.deg2rad(left_offset) / self.P
            wd = np.deg2rad(right_offset) / self.P
            vw = np.dot(np.array([[self.R/2, self.R/2],[self.R/self.L, -self.R/self.L]]), np.array([[wd],[wi]]))
            #print("wi = %.5f, wd = %.5f" %(wi, wd))
            # calculate real delta th
            delta_th = vw[1] * self.P
            #delta_th = (right_offset_length - left_offset_length) / self.L
            th = self.th.value + delta_th
            delta_x, delta_y = 0, 0

            # calculate delta s  (depends on w) (diapo 14)
            if vw[1] != 0:
                delta_s = (vw[0]/vw[1]) * delta_th
                delta_x = delta_s * np.cos(th + (delta_th * 0.5))
                delta_y = delta_s * np.sin(th + (delta_th * 0.5))
            else:
                delta_s = vw[0] * self.P
                delta_x = delta_s * np.cos(th)
                delta_y = delta_s * np.sin(th)
            
            #print('v: %.2f, w = %.2f' %(vw[0], vw[1]))

            #print('left_offset_length: %.2f, right_offset_length: %.2f' %(left_offset_length, right_offset_length))
            # Extract the gyroscope data (más adelante)

            # To "lock" a whole set of operations, we can use a "mutex"
            #print("\n==================================")
            #print("== Left Wheel ====================")
            #print("> Grades offset: %dº -> %dº (+%dº)" %(self.sI.value, left_encoder, left_offset))
            #print("> Length offset: %.5f" %(left_offset_length))
            #print("== Right Wheel ===================")
            #print("> Grades offset: %dº -> %dº (+%dº)" %(self.sD.value, right_encoder, right_offset))
            #print("> Length offset: %.5f" %(right_offset_length))
            #print("==================================")
            #print("> DeltaS: %.5f" %(delta_s))
            
            
            # calculo de delta xWR
            #print('delta_x: %.2f, delta_y: %.2f' %(delta_x, delta_y))
            
            self.lock_odometry.acquire()
            # update new xWR+
            
            self.x.value += delta_x
            self.y.value += delta_y
            self.th.value = th
            
            self.sI.value = left_encoder
            self.sD.value = right_encoder
            self.totalLength.value += delta_s
            self.lock_odometry.release()

            #print("> Total Length (+/-): %.5f" %(self.totalLength.value))
            #print("== Ángulo =====================")
            #print('x: %.2f, y: %.2f, th: %.2f' %(self.x.value, self.y.value, np.rad2deg(self.th.value)))
            #print('delta_s: %.2f' %(delta_s))
            #print("> Velocidad angular: %.5f" %(w_gyro))
            #print("==================================")


            # save LOG
            # Need to decide when to store a log with the updated odometry ...

            ######## UPDATE UNTIL HERE with your code ########


            tEnd = time.clock()
            #print("tIni: %.2f, tEnd: %.2f, tEnd-tIni: %.2f, tSleep: %.2f" %(tIni, tEnd, tIni-tEnd, self.P-(tEnd-tIni)))
            time.sleep(self.P - (tEnd-tIni))

        #print("Stopping odometry ... X= %d" %(self.x.value))
        sys.stdout.write("Stopping odometry ... X=  %.2f, \
                Y=  %.2f, th=  %.2f \n" %(self.x.value, self.y.value, self.th.value))

    # Stop the odometry thread.
    def stopOdometry(self):
        self.finished.value = True
        self.BP.reset_all()

