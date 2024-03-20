import math
import time
from geometry import Vector2, circunferences_secant_points, Transform

# HSV O RGB
# Distancia con la que detectamos la pelota
# si se mueve bien el robot o va a trompicones
# Algoritmo de detección = Explicar (blob, deteccion de areas, conexas)
# Que coga la pelota y que determine correctamente que la ha cogido


# COSAS A TENER EN CUENTA:
# - El sistema de referencia empieza en el (0,0)
#

########################################################################

def test_lineal_trayectory(robot, dest, v, w):
    """
        Trayectoria lineal  \
        - robot: El robot
        - dest:  Distancia a recorrer (cm).
        - v:     Velocidad lineal del robot (cm/s).   
    """
    STATE  = "IDLE"
    STATES = {
        "IDLE": None,
        "A_TO_B": Transform(dest, 0),
        "ROTATING_IN_B": Transform(dest, 180),
        "B_TO_A": Transform(Vector2.zero, 180),
        "EXIT": Transform(Vector2.zero, 0)
    }

    while True:
        x, y, th  = robot.readOdometry()
        transform = Transform(Vector2(x, y), th)
        # --
        if STATE == "IDLE":
            robot.setSpeed(v, 0.0)
            STATE = "A_TO_B"
        elif STATE == "A_TO_B":
            if transform == STATES[STATE]:
                robot.setSpeed(0.0, w)
                STATE = "ROTATING_IN_B"
        elif STATE == "ROTATING_IN_B":
            if transform == STATES[STATE]:
                robot.setSpeed(v, 0.0)
                STATE = "B_TO_A"
        elif STATE == "B_TO_A":
            if transform == STATE[STATE]:
                robot.setSpeed(0.0, -w)
                STATE = "EXIT"
        elif STATE == "EXIT":
            if transform == STATE[STATE]:
                robot.setSpeed(0.0, 0.0)
                break
        time.sleep(robot.P)

################################################

def test_trace_eight(robot, v, radius):

    w      = v/radius 
    STATE  = "IDLE"
    STATES = {
        "IDLE": None,
        "ROTATING_IN_A": Transform(Vector2.zero, 90),
        "TURNING_TOWARDS_B": Transform(Vector2(2*radius, 0), -90),
        "TURNING_BACK_TO_B": Transform(Vector2(2*radius, 0),  90),
        "TURNING_BACK_TO_A": Transform(Vector2.zero, 90),
        "EXIT": Transform(Vector2.zero, 0)
    }
    while True:
        x, y, th  = robot.readOdometry()
        transform = Transform(Vector2(x,y), th)
        # Path states
        if STATE == "IDLE":
            robot.setSpeed(0, w)
            STATE = "ROTATING_IN_A"
        elif STATE == "ROTATING_IN_A":
            if transform == STATES[STATE]:
                robot.setSpeed(v,  -w)
                STATE = "TURNING_TOWARDS_B"
        elif STATE == "TURNING_TOWARDS_B":
            if transform == STATES[STATE]:
                robot.setSpeed(v,  w)
                STATE = "TURNING_BACK_TO_B"
        elif STATE == "TURNING_BACK_TO_B":
            if transform == STATES[STATE]:
                robot.setSpeed(v, -w)
                STATE = "TURNING_BACK_TO_A"
        elif STATE == "TURNING_BACK_TO_A":
            if transform == STATES[STATE]:
                robot.setSpeed(v,  w)
                STATE = "EXIT"
        elif STATE == "EXIT":
            if transform == STATES[STATE]:
                robot.setSpeed(0,  0)
                break
        time.sleep(robot.P)

def test_bicycle(robot, v, fst_radius, snd_radius, axis_dist):

    w = v/((fst_radius + snd_radius)/2)
    P1, P2, P3, P4 = circunferences_secant_points(fst_radius, snd_radius, axis_dist)
    STATE  = "IDLE"
    STATES = {
        "IDLE": None,
        "ROTATING_IN_A": Transform(Vector2.zero, -90),
        "TURNING_TOWARDS_P1": Transform(P1, forward=P2-P1),
        "P1_TO_P2": Transform(P2, forward=P2-P1),
        "TURNING_TOWARDS_P3": Transform(P3, forward=P4-P3),
        "P3_TO_P4": Transform(P4, forward=P4-P3),
        "TURNING_TOWARDS_A": Transform(Vector2.zero, -90),
        "EXIT": Transform(Vector2.zero, 0)
    }
    # 
    while True:
        x, y, th = robot.readOdometry()
        transform = Transform(Vector2(x,y), th)
        # Pâth states
        if STATE == "IDLE":
            robot.setSpeed(0, -w)
            STATE = "ROTATING_IN_A"
        elif STATE == "ROTATING_IN_A":
            if transform == STATES[STATE]:
                robot.setSpeed(v, w)
                STATE = "TURNING_TOWARDS_P1"
        elif STATE == "TURNING_TOWARDS_P1":
            if transform == STATES[STATE]:
                robot.setSpeed(v, 0)
                STATE = "P1_TO_P2"
        elif STATE == "P1_TO_P2":
            if transform == STATES[STATE]:
                robot.setSpeed(v, w)
                STATE = "TURNING_TOWARDS_P3"
        elif STATE == "TURNING_TOWARDS_P3":
            if transform == STATES[STATE]:
                robot.setSpeed(v, 0)
                STATE = "P3_TO_P4"
        elif STATE == "P3_TO_P4":
            if transform == STATES[STATE]:
                robot.setSpeed(v, w)
                STATE = "TURNING_TOWARDS_A"
        elif STATE == "TURNING_TOWARDS_A":
            if transform == STATES[STATE]:
                robot.setSpeed(0, w)
                STATE = "EXIT"
        elif STATE == "EXIT":
            if transform == STATES[STATE]:
                robot.setSpeed(0, 0)
                break

        time.sleep(robot.P)


