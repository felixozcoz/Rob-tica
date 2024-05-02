#!/usr/bin/python
# -*- coding: UTF-8 -*-
import numpy as np
import time
from Robot import Robot
from ReMapLib import Map

from decimal import Decimal


def main():    
    try:




    

        # 1. Inicializar el mapa
        #print("Inicializando mapa ...")
        #print("---------------------------------------------------")
        #neighborhood = 4
#
        ##start, goal = [0,0], [0,1]
        ##rMap = Map("maps/mapa_simple.txt", [0,0], [0,1], neighborhood=neighborhood)
        #
        ##start, goal = [0,0], [2,2]
        ##rMap = Map("maps/mapa0.txt", [0,0], [2,2], neighborhood=neighborhood)
        #
        ##start, goal = [0,0], [2,2]
        ##rMap = Map("maps/mapa1.txt", [0,0], [2,2], neighborhood=neighborhood)
#
        ##start, goal = [0,0], [4,7]
        ##rMap = Map("maps/mapa2.txt", [0,0], [4,7], neighborhood=neighborhood)
#
        #start, goal = [1,0], [3,4]
        #rMap  = Map("maps/mapa2.txt", start, goal, neighborhood=neighborhood)
        ##rMap.drawMapWithRobotLocations()
#
        ## 2. Inicializar el robot
        #global_reference = [rMap.halfCell+start[1]*rMap.sizeCell, rMap.halfCell+start[0]*rMap.sizeCell,90]
        robot = Robot()
        time.sleep(5)
        
        robot.startOdometry()


        points = [[0,0], [40,0], [80,40], [160,-40], [190,0], [200,0]]
        robot.playTrajectory(points, 30)

        # 3. Recorrer mapa
        #print("Recorriendo mapa ... ")
        #if neighborhood == 4:
        #    robot.playNavigation_4N()
        #else:
        #    robot.playNavigation_8N()

        # 4. Wrap up and close stuff ...
        # This currently unconfigure the sensors, disable the motors, 
        # and restore the LED to the control of the BrickPi3 firmware.
        robot.stopOdometry()

    except KeyboardInterrupt: 
    # Except the program gets interrupted by Ctrl+C on the keyboard.
    # THIS IS IMPORTANT if we want that motors STOP when we Ctrl+C ...
        robot.stopOdometry()
        print('*** Ctrl-C detected - Finishing ...')

if __name__ == "__main__":

    # get and parse arguments passed to main
    # Add as many args as you need ...
    # parser = argparse.ArgumentParser()
    # parser.add_argument("-c", "--color", help="color of the ball to track",
    #                     type=float, default=40.0)
    # args = parser.parse_args()
    main()
