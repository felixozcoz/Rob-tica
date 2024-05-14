#!/usr/bin/python
# -*- coding: UTF-8 -*-
import time
import cv2 as cv
import traceback

from Robot import Robot
from ReMapLib import Map
from geometry import Vector2

def main():    
    try:
        # 1. Inicializar robot
        print("Inicializando el robot y sus componentes")
        robot = Robot()

        # 2a. Inicializar parametros en base al sensor
        if robot.light_intensity < 2600: 
            print("Cargando parametros para la salida desde A")
            # points = [[0,0], [35,0], [80,-35], [160,35], [195,0], [200,0]]
            # points = [[0,0], [35,0], [80,-35], [160,35], [200,0]]
            # points = [[0,0], [35,0], [75,-30], [85,-30], [155,30], [165,30], [200,0]]
            # rmap = Map("mapaA_CARRERA.txt", [2,1], [3,3])
            # rmap_ref = rmap.cell2pos([7,1], list) + [-90]
            # robot.loadMap(rmap, rmap_ref)
            # img_R2D2_or_BB8 = cv.imread("images/R2-D2_s.png", cv.IMREAD_COLOR)
            exit_cells = [[6,3], [6,6]]
            end_cells  = [[7,3], [7,6]]
            # sense = -1
        else:
            print("Cargando parametros para la salida desde B")
            # points = [[0,0], [35,0], [80,35], [160,-35], [195, 0], [200,0]]
            # points = [[0,0], [35,0], [80,35], [160,-35], [200,0]]
            points = [[0,0], [35,0], [75,30], [85,30], [155,-30], [165,-30], [200,0]]
            rmap = Map("mapaB_CARRERA.txt", [2,5], [3,3])
            rmap_ref = rmap.cell2pos([7,5], list) + [-90]
            img_R2D2_or_BB8 = cv.imread("images/BB8_s.png", cv.IMREAD_COLOR)
            exit_cells = [[6,3], [6,0]]
            end_cells  = [[7,3], [7,0]]
            sense =  1 
        ## 2b. Iniciar la odometria
        robot.startOdometry()
        ## 2c. Pulsar para comenzar
        #input("Pulsa cualquier letra para comenzar. . .")

        # 3. Ejecutar recorrido
        # . 1a fase. Ejecucion de trayectoria
        print("Recorriendo trayectoria. . .")
        # robot.playTrajectory(points, 30)
        # Centramos robot en su celda
        # robot.centerRobot(sense)
        # . 2a fase. Navegacion
        print("Realizando navegacion. . .")
        # robot.playMap()
        # . 3a fase. Obtencion de la salida
        # print("Searching ...")
        print("Encontrando robot para determinar la salida. . .")
        #found = robot.matchObject(img_R2D2_or_BB8, showMatches=True)
        exit_cell = exit_cells[0] #exit_cells[int(not found)]
        end_cell  = end_cells[0] #end_cells[int(not found)]
        # . 4a fase. Tracking (mascara con colores negativos)
        print("Realizando seguimiento de la pelota para capturarla. . .")
        robot.trackObject((80, 70, 50), (100, 255, 255), sense)
        # . 5a fase. Salida
        print("Saliendo del mapa. . .")
        #x, y, _, _ = robot.readOdometry()
        # [TODO] Eliminar esta prueba ad-hoc
        x, y       = 120, -120.0
        robot.lock_odometry.acquire()
        robot.x.value = x
        robot.y.value = y
        robot.lock_odometry.release()
        gpos = robot.ltow * Vector2(x, y, 1)
        rmap.setPath_8N(rmap.pos2cell(gpos.x, gpos.y), exit_cell)
        points = list(reversed(rmap.path))
        for point in points:
            point = rmap.cell2pos(point)
            point = list(robot.wtol * point)[:2]
        # Se actualizan los puntos ya que:
        # 1. El primer punto es erroneo, podria no empezar desde el centro de la celda, entonces añadimos la posicion del robot
        # 2. El ultimo punto a de ser la casilla final que es la que esta fuera del mapa.
        points = [x,y] + points[1:] + [list(robot.wtol * rmap.cell2pos(end_cell))[:2]]        
        print(points)
        robot.playTrajectory(points, 30, True, True)

        # 4. Wrap up and close stuff ...
        # This currently unconfigure the sensors, disable the motors, 
        # and restore the LED to the control of the BrickPi3 firmware.
        print("Finalizando odometria y saliendo del programa. . .")
        robot.stopOdometry()
        print("\nBYE BYE ;)! ! !\n")

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

