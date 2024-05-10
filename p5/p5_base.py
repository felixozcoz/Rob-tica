#!/usr/bin/python
# -*- coding: UTF-8 -*-
import time
import cv2 as cv
import traceback

from Robot import Robot
from ReMapLib import Map
from geometry import Vector2, Transform

def main():    
    try:
        # 1. Inicializar robot
        robot = Robot()
        time.sleep(5)
        
        light = []
        for i in range(3):
           light.append(robot.getLight())
           time.sleep(0.5) 
        print("Light: ", light)
        robot.BP.set_sensor_type(robot.PORT_COLOR, robot.BP.SENSOR_TYPE.NONE)
        light = light[-1]
        rmap = None
        rmap_ref = None
#
#
        ## 2a. Inicializar parametros en base al sensor
        if 2600 < light:  
            print("Mapa B")
            # points          = [[0,0], [35,0], [80,35], [160,-35], [195,0], [200,0]]
            points          = [[0,0], [35,0], [80,35], [160,-35], [200,0]]
            rmap            = Map("mapaB_CARRERA.txt", [2,5], [3,3], neighborhood=4)
            rmap_ref        = rmap.cell2pos([7,5], list) + [-90]
            img_R2D2_or_BB8 = cv.imread("images/BB8_s.png", cv.IMREAD_COLOR)
            exit_cells      = [[6,4], [6,1]] 
            side = 1
        else:
            print("Mapa A")
            # points          = [[0,0], [35,0], [80,-35], [160,35], [195,0], [200,0]]
            points          = [[0,0], [35,0], [80,-35], [160,35], [200,0]]
            points          = [[0,0], [35,0], [75,-30], [85,-30], [155,30], [165,30], [200,0]]
            rmap            = Map("mapaA_CARRERA.txt", [2,1], [3,3], neighborhood=4)
            rmap_ref        = rmap.cell2pos([7,1], list) + [-90]
            img_R2D2_or_BB8 = cv.imread("images/R2-D2_s.png", cv.IMREAD_COLOR)
            exit_cells      = [[6,3], [6,6]]
            side = -1
        ## 2b. Iniciar la odometria
        robot.loadMap(rmap, rmap_ref)
        robot.startOdometry()


        #position_transform = Transform(Vector2(80,0), 0)
        #robot.setSpeed(10,0)
        #while True:
        #    x, y, _, _ = robot.readOdometry()
        #    print(x, y)
        #    if position_transform == Transform(Vector2(x, y), 0):
        #        robot.setSpeed(0, 0)
        #        break
                
        # 3. Ejecutar recorrido
        # . 1a fase. Ejecucion de trayectoria
        robot.playTrajectory(points, 30, False)
        # Centramos robot en su celda
        robot.centerRobot(side)
        # . 2a fase. Navegacion
        robot.playMap()
        # . 3a fase. Obtencion de la salida
        robot.initCamera()
        found = robot.matchObject(img_R2D2_or_BB8)
        print("Found: ", found)
        exit  = exit_cells[int(not found)]
        # . 4a fase. Tracking (mascara con colores negativos)
        #robot.trackObject((80, 70, 50), (100, 255, 255))
        # . 5a fase. Salida
        #x, y, _, _ = robot.readOdometry()
        #rmap.replanPath_8N(rmap.pos2cell(pos.x, pos.y), exit)
        #points = reversed(rmap.path)
        #for point in points:
        #   point = cell2pos(point)
        #   point = list(robot.wtol * point)[:2]
        #robot.playTrayectory(points)

        # 4. Wrap up and close stuff ...
        # This currently unconfigure the sensors, disable the motors, 
        # and restore the LED to the control of the BrickPi3 firmware.
        robot.stopOdometry()

    except Exception as e: #KeyboardInterrupt: 
    # Except the program gets interrupted by Ctrl+C on the keyboard.
    # THIS IS IMPORTANT if we want that motors STOP when we Ctrl+C ...
        robot.stopOdometry()
        print(e)
        traceback.print_exc()
        print('*** Ctrl-C detected - Finishing ...')

if __name__ == "__main__":

    # get and parse arguments passed to main
    # Add as many args as you need ...
    # parser = argparse.ArgumentParser()
    # parser.add_argument("-c", "--color", help="color of the ball to track",
    #                     type=float, default=40.0)
    # args = parser.parse_args()
    main()
