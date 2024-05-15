#!/usr/bin/python
# -*- coding: UTF-8 -*-
import time
import cv2 as cv
import traceback

from Robot import Robot
from ReMapLib import Map
from geometry import Vector2, Transform
from decimal import Decimal
import numpy as np

def main():    
    try:
        # 1. Inicializar robot
        print("Inicializando el robot y sus componentes")
        robot = Robot()

        # 2a. Inicializar parametros en base al sensor
        if robot.light_intensity < 2600: 
            print("Cargando parametros para la salida desde A")
            #points = [[0,0], [35,0], [80,-40], [160,40], [195,0], [200,0]]
            # points = [[0,0], [35,0], [80,-35], [160,35], [200,0]]
            points = [[0,0], [35,0], [75,-30], [85,-30], [155,30], [165,30], [200,0]]
            rmap = Map("mapaA_CARRERA.txt", [2,1], [4,3])
            rmap_ref = rmap.cell2pos([7,1], list) + [-90]
            robot.loadMap(rmap, rmap_ref)
            img_R2D2_or_BB8 = cv.imread("images/R2-D2_s.png", cv.IMREAD_COLOR)
            exit_cells = [[6,3], [6,6]]
            end_cells  = [[7,3], [7,6]]
            direction  = [Vector2.down, Vector2.up]
            sense = -1
        else:
            print("Cargando parametros para la salida desde B")
            points = [[0,0], [35,0], [80,40], [160,-40], [195, 0], [200,0]]
            # points = [[0,0], [35,0], [80,35], [160,-35], [200,0]]
            points = [[0,0], [35,0], [75,30], [85,30], [155,-30], [165,-30], [200,0]]
            rmap = Map("mapaB_CARRERA.txt", [2,5], [4,3])
            rmap_ref = rmap.cell2pos([7,5], list) + [-90]
            robot.loadMap(rmap, rmap_ref)
            img_R2D2_or_BB8 = cv.imread("images/BB8_s.png", cv.IMREAD_COLOR)
            exit_cells = [[6,3], [6,0]]
            end_cells  = [[7,3], [7,0]]
            direction  = [Vector2.up, Vector2.down]
            sense =  1 
        robot.BP.set_sensor_type(robot.PORT_COLOR, robot.BP.SENSOR_TYPE.NONE)
        ## 2b. Iniciar la odometria
        robot.startOdometry()
        ## 2c. Pulsar para comenzar
        #input("Pulsa cualquier letra para comenzar. . .")

        # 3. Ejecutar recorrido
        # . 1a fase. Ejecucion de trayectoria
        print("Recorriendo trayectoria. . .")
        robot.playTrajectory(points, 30, False, False, showPlot=False)
        # Centramos robot en su celda
        robot.centerRobot(sense)
        # . 2a fase. Navegacion
        print("Realizando navegacion. . .")
        robot.playMap()
        # . 3a fase. Obtencion de la salida
        print("Encontrando robot para determinar la salida. . .")
        found = robot.matchObject(img_R2D2_or_BB8, True)
        exit_cell = exit_cells[int(not found)]
        end_cell = end_cells[int(not found)]
        end_rot  = direction[int(not found)]
        # . 4a fase. Tracking (mascara con colores negativos)
        print("Realizando seguimiento de la pelota para capturarla. . .")
        robot.trackObject((80, 70, 50), (100, 255, 255), -sense)
        # . 5a fase. Salida
        print("Saliendo del mapa. . .")
        # us_ev3_values = [robot.us_ev3.value, robot.us_ev3.value, robot.us_ev3.value]
        x, y, th, _ = robot.readOdometry()
        # forward     = Vector2.right.rotate(th)
        # rotation_reached = False
        # position_reached = False
        # rotation_transform = Transform(Vector2.zero, forward=end_rot)
        # robot.setSpeed(0, forward.sense(end_rot))
        # state = "ROTATE"
        # while True:
        #     x, y, th, _ = robot.readOdometry()
        #     forward     = Vector2.right.rotate(th)
        #     if state == "ROTATE":
        #         if not rotation_reached:
        #             if rotation_transform == Transform(Vector2.zero, forward=forward):
        #                 robot.setSpeed(10,0)
        #                 state = "FORWARD"
        #     elif state == "FORWARD":
        #         # Obtenemos los datos del ultrasonido
        #         us_position_reached = np.mean(us_ev3_values) <= 14 #Decimal(np.mean(us_ev3_values)) % Decimal(robot.rmap.sizeCell) <= 14
        #         us_ev3_values = us_ev3_values[1:] + [robot.us_ev3.value]
        #         if us_position_reached:
        #             rotation_transform = Transform(Vector2.zero, forward=Vector2.right)
        #             robot.setSpeed(0,forward.sense(Vector2.right))
        #             state = "ME GIRO"
        #     elif state == "ME GIRO":
        #         if rotation_transform == Transform(Vector2.zero, forward=forward):
        #             aux = robot.wtol * robot.rmap.cell2pos(end_cell)
        #             rotation_transform = Transform(Vector2.zero, forward=aux - Vector2(x,y))
        #             position_transform = Transform(aux)
        #             robot.setSpeed(-10,0)
        #             state = "BACKWARD"
        #     elif state == "BACKWARD":
        #         us_position_reached = Decimal(np.mean(us_ev3_values)) % Decimal(robot.rmap.sizeCell) <= 14
        #         us_ev3_values = us_ev3_values[1:] + [robot.us_ev3.value]
        #         if not rotation_transform == Transform(Vector2.zero, forward=forward):
        #             robot.setSpeed(-10, forward.sense(rotation_transform.forward) * 0.35)
        #         else:
        #             robot.setSpeed(-10,0)
        #         if position_reached or position_transform == Transform(Vector2(x,y)):
        #             position_reached = True
        #             if us_position_reached:
        #                 robot.setSpeed(0,0)
        #                 break
                    
                    
        
        gpos = robot.ltow * Vector2(x, y, 1)
        #gfor = robot.ltow * Vector2.right.rotate(th)
        #robot.rmap.setPath_4N(rmap.pos2cell(gpos.x, gpos.y), exit_cell)
        #robot.rmap.path = [end_cell] + robot.rmap.path
        #robot.rmap.connectionMatrix[2*end_cell[0]+1-1][2*end_cell[1]+1] = 1
        #robot.rmap.goal = end_cell
        #mappos = robot.rmap.cell2pos(robot.rmap.path[-1])
        #dir = mappos - gpos
#
        #rotation_reached   = False
        #rotation_transform = Transform(Vector2.zero, forward=dir)
        #position_transform = Transform(mappos)
        #
        #print(robot.rmap)
        #print(robot.rmap.path[-1], mappos, gpos, dir)
        #robot.setSpeed(0, gfor.sense(dir))
        #while True:
        #    x, y, th, _ = robot.readOdometry()
        #    gpos = robot.ltow * Vector2(x, y, 1)
        #    gfor = robot.ltow * Vector2.right.rotate(th)
        #    if not rotation_reached:
        #        if rotation_transform == Transform(Vector2.zero, forward=gfor):
        #            rotation_reached = True
        #            robot.setSpeed(0,0)
        #            break

            #else:
            #    if not rotation_transform == Transform(Vector2.zero, forward=gfor):
            #        robot.setSpeed(10, gfor.sense(rotation_transform.forward) * 0.35)
            #    else:
            #        robot.setSpeed(10,0)
            #    if position_transform == Transform(gpos):
            #        robot.setSpeed(0,0)
            #        break
        #robot.playMap(recogn=False)
        
        rmap.setPath_8N(rmap.pos2cell(gpos.x, gpos.y), exit_cell)
        # cambio de celdas a posiciones locales (inicio lista = última celda, final lista = primera celda)
        points = rmap.path
        for i in range(len(points)):
            points[i] = rmap.cell2pos(points[i])
            points[i] = list(robot.wtol * points[i])[:2]
        # Se actualizan los puntos ya que:
        # 1. El primer punto es erroneo, podria no empezar desde el centro de la celda, entonces añadimos la posicion del robot
        # 2. El ultimo punto a de ser la casilla final que es la que esta fuera del mapa.
        points = [list(robot.wtol * rmap.cell2pos(end_cell))[:2]] + points[:-1] + [[x,y]]      
        print(points)
        robot.playTrajectory(points, 30, True, True, showPlot=False)

        # 4. Wrap up and close stuff ...
        # This currently unconfigure the sensors, disable the motors, 
        # and restore the LED to the control of the BrickPi3 firmware.
        print("Finalizando odometria y saliendo del programa. . .")
        robot.stopOdometry()
        print("\nBYE BYE ;)! ! !\n")

    except Exception as e: #KeyboardInterrupt: 
    # Except the program gets interrupted by Ctrl+C on the keyboard.
    # THIS IS IMPORTANT if we want that motors STOP when we Ctrl+C ...
        robot.setSpeed(0,0,0)
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

