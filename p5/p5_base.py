#!/usr/bin/python
# -*- coding: UTF-8 -*-
import time
import cv2 as cv

from Robot import Robot
from ReMapLib import Map

def main():    
    try:
        # 1. Inicializar robot
        robot = Robot()
        time.sleep(5)

        # 2a. Inicializar parametros en base al sensor 
        if 2000 < robot.light <= 2200:
            points          = [[0,0], [20,0], [40,20], [80,-20], [99,0], [100,0]]
            rmap            = Map("mapaA_CARRERA.txt", [2,1], [3,3], neighborhood=4)
            rmap_ref        = rmap.cell2pos([7,1], list) + [-90]
            img_R2D2_or_BB8 = cv.imread("images/R2-D2_s.png", cv.IMREAD_COLOR)
            exit_cells      = [[6,3], [6,6]]
        elif 2600 < robot.light <= 2800:
            points          = [[0,0], [20,0], [40,-20], [80,20], [99,0], [100,0]]
            rmap            = Map("mapaB_CARRERA.txt", [2,6], [3,3], neighborhood=4)
            rmap_ref        = rmap.cell2pos([7,5], list) + [-90]
            img_R2D2_or_BB8 = cv.imread("images/BB8_s.png", cv.IMREAD_COLOR)
            exit_cells      = [[6,4], [6,1]]
        # 2b. Iniciar la odometria
        robot.loadMap(rmap, rmap_ref)
        robot.startOdometry()

        # 3. Ejecutar recorrido
        # . 1º fase. Ejecucion de trayectoria
        robot.playTrajectory(points, 100)
        # . 2º fase. Navegacion
        robot.playMap()
        # . 3º fase. Obtencion de la salida
        #found = robot.matchObject(img_R2D2_or_BB8)
        #exit  = exit_cells[int(not found)]
        # . 4º fase. Tracking (mascara con colores negativos)
        #robot.trackObject((80, 70, 50), (100, 255, 255))
        # . 5º fase. Salida
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
