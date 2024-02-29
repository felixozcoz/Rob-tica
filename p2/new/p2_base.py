import math
from geometry import Vector2

# HSV O RGB
# Distancia con la que detectamos la pelota
# si se mueve bien el robot o va a trompicones
# Algoritmo de detección = Explicar (blob, deteccion de areas, conexas)
# Que coga la pelota y que determine correctamente que la ha cogido

class Transform:
    def __init__(self, position, point_error : Vector2, modulus_error):
        self.position = position
        # Area error
        position_shift = point_error.normalize()
        self.inf_error = position - poin
        self.point_inf_error = position - point_shift
        self.point_sup_error = position + point_shift
        # Modulus error
        self.modulus_error      = modulus_error
        self.modulus_last_error = modulus_error

    def check(self, position : Vector2):
        # Center range check
        if (position.x > self.point_inf_error.x and position.x <= self.point_sup_error.x) and (position.y > self.point_inf_error.y and position.y <= self.point_sup_error.y):
            return True
        # Center distance check
        dist = (self.position - position).magnitude()
        if dist < self.point_dist_error:
            return True
        # If last error is 
        if self.modulus_last_error < self.modulus_error:
            return True
        
            





########################################################################

def test_lineal_trayectory(robot, d, v):
    """
        Trayectoria lineal  \
        - robot: El robot
        - d:     Distancia a recorrer (cm).
        - v:     Velocidad lineal del robot (cm/s).   
    """
    robot.setSpeed(v, 0.0)
    while True:
        x,_,_ = robot.readOdometry()
        if x >= d:
            robot.setSpeed(0.0, 0.0)
            break

################################################

def test_trace_eight(robot, r):
    """
        Trayectoria del 8 \
        - robot:    El robot
        - r:        Radio de ambas circunferencias
    """
    # Primero giro (90º dcha)
    rot_range = [math.pi/2, math.pi/2 + 0.2]
    robot.setSpeed(0, -math.pi/2)
    while True:
        _, _, th = robot.readOdometry()
        th = abs(th)
        if th >= rot_range[0] and th < rot_range[1]:
            break

    # Movimientos del 8
    # - Positions check
    point_error = 1
    point_shift = Vector2(point_error, point_error).normalize()
    moved = False
    start = Vector2()
    start_range = [start - point_shift, start + point_shift ]

    center = Vector2(2*r, 0)
    center_range = [center - point_shift, center + point_shift]
    # - Magnitude check
    center_dist_error = 1
    center_last_dist  = center.magnitude()
    # - Robot speed
    w = (2*math.pi)/10 # Velocidad angular del robot.
    v = r * w          # Velocidad lineal del robot.
    sense = 1          # Sentido de movimiento.
    # - Run conditions
    ignore = False

    while True:
        robot.setSpeed(v, sense * w)
        x,y,_ = robot.readOdometry()

            #if not ignore:
            #    sense  *= -1
            #    ignore  = True
            #    moved   = True
        
        ignore = False
        if moved and (x > start_range[0].x and x < start_range[1].x) and (y > start_range[0].y and y < start_range[1].y):
            break


    center    = [2 * r, 0] # Localización de cambio de fase 
    epsilon   = 3          # Margen de error en estimación de la odometría
    n_epsilon = 0.02       # Margen de error para el metodo con distancia vectorial

    # Es un primer metodo que se tenia en mente, coger el punto central y añadirle
    # unos margenes de error porque el robot no iba a pasar perfectamente por ese punto,
    # entonces, se calculan dos puntos minimo y superior alrededor del punto central
    # para añadir mas margen de detección
    # Punto central del circuito
    min       = [center[0] - epsilon, center[1] - epsilon]
    max       = [center[0] + epsilon, center[1] + epsilon]
    ignore    = False # Al detectar que esta dentro de la región, puede volver a detectarla y
                      # volver a cambiar de sentido, por eso, si lo detecta una vez, que las
                      # siguientes veces lo ignore. 
    steps     = 0
    

    
    # Resto de movimientos
    robot.setSpeed(v, sense * w)
    # En un segundo metodo, la idea es prescindir de las areas y utilizar el modulo del vector,
    # si la distancia del centro del robot al punto es menor que tanta distancia x, quiere decir
    # que esta cerca, pero habia que pulir un metodo que funcionara antes.
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
            if not ignore:
                sense *= -1
                if (sense < 0):
                    time.sleep(0.2 * r/20)
                else:
                    time.sleep(0.6 * r/20)
                ignore = True
                steps += 1
        else:
            ignore = False
        if (steps >= 2) and (x > (0 - epsilon) and x < (0 + epsilon)) and (y > (0 - epsilon) and y < (0 + epsilon)):
            robot.setSpeed(0,0)
            break
    