#!/usr/bin/python
# -*- coding: UTF-8 -*-
import time
from Robot import Robot
from ReMapLib import Map

def main():    
    try:
        # 1. Inicializar robot
        robot = Robot()
        time.sleep(5)

        # 2a. Inicializar parametros en base al sensor 
        while True:
            color     = robot.getColor()
            luminance = 0.2126*color[0] + 0.7512*color[1] + 0.0722*color[2]
            if luminance < 0.2:
                # Generación de trayectoria:
                points = [[0,0], [20,0], [40,20], [80,-20], [99,0], [100,0]]
                # Mapa
                rMap   = Map("mapaA_CARRERA.txt", [2,1], [3,3], neighborhood=4)
                #global_reference = [60,280,-90]
                #global_reference = [rMap.halfCell+start[1]*rMap.sizeCell, rMap.halfCell+start[0]*rMap.sizeCell,90]
                break
            elif luminance > 0.8:
                # Generación de trayectoria:
                points = [[0,0], [20,0], [40,-20], [80,20], [99,0], [100,0]]
                # Mapa
                rMap   = Map("mapaB_CARRERA.txt", [2,6], [3,3], neighborhood=4)
                #global_reference = [220,280,-90]
                #global_reference = [rMap.halfCell+start[1]*rMap.sizeCell, rMap.halfCell+start[0]*rMap.sizeCell,90]
                break

        # 2b. Iniciar la odometria
        robot.startOdometry()

        # 3. Ejecutar recorrido
        # . 1º fase. Ejecucion de trayectoria
        robot.playTrajectory(points, 100)
        # . 2º fase. Navegacion
        robot.playNavigation(rMap)
        # . 3º fase. Obtencion de la salida
        # robot.NoTieneNombreTodavia()
        # . 4º fase. Tracking (mascara con colores negativos)
        robot.trackObject((80, 70, 50), (100, 255, 255))
        # . 5º fase. Salida
        # robot.NoTieneNombreTodavia()

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
