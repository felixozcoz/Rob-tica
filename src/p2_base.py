#!/usr/bin/python
# -*- coding: UTF-8 -*-
import argparse
import math
import numpy as np
import time
from Robot import Robot
import testEvaluacion as te

def dot(v, w):
    return (v[0] * w[0]) + (v[1] * w[1])

def mod(v):
    return math.sqrt(v[0]*v[0] + v[1]*v[1])

def angle(v, w):
    return np.rad2deg(math.acos((dot(v, w)) / (mod(v) * mod(w))))


def get_robot_axis(rot):
    mat = [[math.cos(rot), -math.sin(rot)], [math.sin(rot), math.cos(rot)]]
    res  = np.dot([1, 0], mat)
    return [round(res[0], 2), round(res[1],2)]

def bicycle_test(robot, rad1, rad2, distEjes):
    # Parámetros para primer movimiento (90º izq)
    th1 = np.pi/2  # primero giro para colocacióne en posición 2 (giro a izq)
    w1 = th1 / 3
    
    # Parámetro para rueda
    t = 9            # tiempo de recorrido de media circunferencia ( segundos )
    th = - 2 * np.pi   # ángulo a recorrer en cada fase (4 fases)
    w = th / t    
    v1 = -rad1 * w

    # Parámetro para rueda
    t = 10         # tiempo de recorrido de media circunferencia ( segundos )
    th = - np.pi   # ángulo a recorrer en cada fase (4 fases)
    w2 = th / t
    v2 = - rad2 * w2

    epsilon   = 1          # Margen de error en estimación de la odometría
    f_epsilon = 0.1        # Margen de error
    # Puntos de tangencia para cambio de trayectorio
    (x1, y1), (x2, y2), (x3, y3), (x4, y4) = te.calculate_tangencial_points(rad1, rad2, distEjes) # sup rueda1, sup rueda 2, inf rueda 1, inf rueda 2
    print("Puntos tangenciales: ", (x1, y1), (x2, y2), (x3, y3), (x4, y4))

    dir_sup = [x2 - x1, y2 - y1]
    dir_inf = [x3 - x4, y3 - y4]

    # rueda 1 (pto superior)
        # zona pto tangencia sup rueda 1
    max_sup_r1       = [x1 + (epsilon+2), y1 + (epsilon+2)]
    min_sup_r1       = [x1 - epsilon, y1 - (epsilon+2)]
        # zona pto tangencia inf rueda 1
    max_inf_r1       = [x3+(epsilon+6), y3 + (epsilon+6)]
    min_inf_r1       = [x3-(epsilon+6), y3 - (epsilon+6)]
        # zona pto tangencia sup rueda 2
    max_sup_r2       = [x2 + (epsilon+4), y2 + (epsilon+4)]
    min_sup_r2       = [x2 - (epsilon+4), y2 - (epsilon+4)]
        # zona pto tangencia inf rueda 2
    max_inf_r2       = [x4 + (epsilon+3), y4 + (epsilon+3)]
    min_inf_r2       = [x4 - (epsilon+3), y4 - (epsilon+3)]

    stop = False
    last = False
    # Primero giro (90º izq)
    robot.setSpeed(0, w1)
    while True:
        _, _, th = robot.readOdometry()
        th = np.abs(th)
        if th >= np.pi/2 and th < (np.pi/2 + f_epsilon):
            break
    
    # Resto de movimientos
    robot.setSpeed(v1, w)
    while True:
        #robot.setSpeed(v1, w)
        x, y, th = robot.readOdometry()
        dir_rob = get_robot_axis(-th)
        dir_ang = angle(dir_inf, dir_rob)
        print("DEG: %.2f, ROB(%.2f, %.2f), INF(%.2f, %.2f)" %(dir_ang, dir_rob[0], dir_rob[1], dir_inf[0], dir_inf[1]))
        #print("(X= %.2f, Y= %.2f), (R1SUPMIN.X= %.2f, R1SUPMIN.Y= %.2f), (R1SUPMAX.X= %.2f, R1SUPMAX.Y= %.2f)" %(x, y, min_sup_r1[0], min_sup_r1[1], max_sup_r2[0], max_sup_r2[1]))
        #print("(X= %.2f, Y= %.2f), (R1INFMIN.X= %.2f, R1INFMIN.Y= %.2f), (R1INFMAX.X= %.2f, R1INFMAX.Y= %.2f)" %(x, y, min_inf_r1[0], min_inf_r1[1], max_inf_r2[0], max_inf_r2[1]))
        if (x > min_sup_r1[0] and x < max_sup_r1[0]) and (y > min_sup_r1[1] and y < max_sup_r1[1]): # cambio a v linear hacia sup rueda 2
            #dir_ang = angle(dir_sup, [x2 - x, y2 - y])
            #if dir_ang > -1 and dir_ang < 1:
            robot.setSpeed(v1, 0)
        
        elif (x > min_sup_r2[0] and x < max_sup_r2[0]) and (y > min_sup_r2[1] and y < max_sup_r2[1]): # cambio a w hacia inf rueda 2
            robot.setSpeed(v2, w2)
        
        elif (x > min_inf_r2[0] and x < max_inf_r2[0]) and (y > min_inf_r2[1] and y < max_inf_r2[1]): # cambio a v hacia inf rueda 1
        #elif (dir_ang > -1 and dir_ang < 1):
            time.sleep(0.91 * rad1/20)
            robot.setSpeed(v1, 0)

        elif (x > min_inf_r1[0] and x < max_inf_r1[0]) and (y > min_inf_r1[1] and y < max_inf_r1[1]): # cambio a w hacia sup rueda 1
            time.sleep(0.6 * rad1/20)
            robot.setSpeed(v1, w)
            stop = True
        
        #if stop:
        #    time.sleep(t/4.5)
        #    robot.setSpeed(0,0)
        #    exit(1) 

 

        time.sleep(robot.P)


def test_trace_eight(robot, r):
    # Parámetros para primer movimiento (90º dcha)
    th1 = -np.pi/2  # primero giro para colocacióne en posición 2 (giro a derecha)
    w1 = th1 / 3
    
    # Parámetro resto de movimientos
    t = 10            # tiempo de recorrido de media circunferencia ( segundos )
    th = 2 * np.pi   # ángulo a recorrer en cada fase (4 fases)
    w = th / t    
    v = r * w

    sense     = 1          # Sentido de movimiento
    center    = [2 * r, 0] # Localización de cambio de fase 
    epsilon   = 3          # Margen de error en estimación de la odometría
    f_epsilon = 0.2        # Margen de error
    n_epsilon = 0.02
    min       = [center[0] - epsilon, center[1] - epsilon]
    max       = [center[0] + epsilon, center[1] + epsilon]
    
    
    # Primero giro (90º dcha)
    robot.setSpeed(0, w1)
    while True:
        _, _, th = robot.readOdometry()
        th = np.abs(th)
        if th >= np.pi/2 and th < (np.pi/2 + f_epsilon):
            break
    
    # Resto de movimientos
    robot.setSpeed(v, sense * w)
    #while True:
    #    robot.setSpeed(v, sense * w)
    #    x, y, _ = robot.readOdometry()
    #    rcvec   = [center[0] - x, center[1] - y]
    #    dist    = math.sqrt(rcvec[0]*rcvec[0] + rcvec[1]*rcvec[1])
    #    print("X= %.2f, Y= %.2f, DIST= %.2f" %(x, y, dist))
    #    if dist < n_epsilon:
    #        sense *= -1
    #        time.sleep(0.2)
    while True:
        robot.setSpeed(v, sense * w)
        x, y, _ = robot.readOdometry()
        if (x > min[0] and x < max[0]) and (y > min[1] and y < max[1]):
            sense *= -1
            time.sleep(0.6)
    

def test_velocidad_lineal(robot, d, t):
    robot.setSpeed((d/t),0.0)
    x = 0

    while x < d:
        time.sleep(2*robot.P)
        x,_,_ = robot.readOdometry()
        print("\t\tx TEST: %.2f" %(x))

def main(args):
    try:
        if args.radioD < 0:
            print('d must be a positive value')
            exit(1)

        # Instantiate Odometry. Default value will be 0,0,0
        # robot = Robot(init_position=args.pos_ini)
        robot = Robot()

        print("X value at the beginning from main X= %.2f" %(robot.x.value))

        # 1. launch updateOdometry Process()
        #time.sleep(10)
        robot.startOdometry()

        # 2. perform trajectory

        # PART 1:
        #while True:
        #    robot.setSpeed(1,0)
        # until ...

        # PART 2:
        # robot.setSpeed()
        # until ...

        # ...

        # Test 1: solo velocidad lineal va recto y no modifica ángulo
        #test_velocidad_lineal(robot, 120.0, 6)
        #test_trace_eight(robot, 20)
        bicycle_test(robot, 20, 30, 60)
        # 3. wrap up and close stuff ...
        # This currently unconfigure the sensors, disable the motors,
        # and restore the LED to the control of the BrickPi3 firmware.
        robot.stopOdometry()


    except KeyboardInterrupt:
    # except the program gets interrupted by Ctrl+C on the keyboard.
    # THIS IS IMPORTANT if we want that motors STOP when we Ctrl+C ...
        robot.stopOdometry()

if __name__ == "__main__":

    # get and parse arguments passed to main
    # Add as many args as you need ...
    parser = argparse.ArgumentParser()
    parser.add_argument("-d", "--radioD", help="Radio to perform the 8-trajectory (mm)",
                        type=float, default=40.0)
    args = parser.parse_args()

    main(args)



