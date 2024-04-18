#!/usr/bin/python
# -*- coding: UTF-8 -*-
import numpy as np
import time
from Robot import Robot
from ReMapLib import Map

def main():    
    try:
        # 1. Inicializar el mapa
        print("Inicializando mapa ...")
        #print("---------------------------------------------------")
        #rMap = Map("maps/mapa0.txt", [0,0], [2,2])
        #
        #print("---------------------------------------------------")
        #rMap = Map("maps/mapa1.txt", [0,0], [2,2])
        #
        #print("---------------------------------------------------")
        #rMap = Map("maps/mapa2.txt", [0,0], [4,6])
        print("---------------------------------------------------")
        rMap = Map("maps/mapa_simple.txt", [0,0], [0,1], neighborhood=4)

        #rMap.drawMapWithRobotLocations()
        # 2. Inicializar el robot
        global_reference = [20,20,90]
        robot = Robot(global_reference=global_reference, rMap=rMap) 
        time.sleep(4)
        robot.startOdometry()

        #while(True):
        #    i = 1
        
        # 3. Recorrer mapa
        # print("Recorriendo mapa ... ")
        robot.playTrayectory()
        
        # 3. Wrap up and close stuff ...
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

