import numpy as np
from geometry import Vector2, Matrix2, Transform
# from pynput import keyboard
# from pynput.keyboard import Key
from p5.ReMapLibold import Map
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline, PchipInterpolator

def round_list(a_list):
    for i,v in enumerate(a_list):
        a_list[i] = round(v,2)
        if a_list[i] == 0:
            a_list[i] = abs(a_list[i])
    return a_list

def simulate_robot(key):
    if key == Key.right:
        lpos[2] = (lpos[2]+90) % 360
    elif key == Key.left:
        lpos[2] = (lpos[2]-90) % 360
    elif key == Key.up:
        if (lpos[2] ==  0):
            lpos[0] += 1
        if (lpos[2] == 180):
            lpos[0] -= 1
        if (lpos[2] == 90):
            lpos[1] += 1
        if  (lpos[2] == 270):
            lpos[1] -= 1
    elif key == Key.down:
        if (lpos[2] ==  0):
            lpos[0] -= 1
        if (lpos[2] == 180):
            lpos[0] += 1
        if (lpos[2] == 90):
            lpos[1] -= 1
        if  (lpos[2] == 270):
            lpos[1] += 1
    elif key == Key.esc:
        exit()
    
    aux = lpos[2]*np.pi/180
    #new_lrig = [lrig[0]*np.cos(aux) - lrig[1]*np.sin(aux), lrig[0]*np.sin(aux) + lrig[1]*np.cos(aux)]
    new_lfor = [lfor[0]*np.cos(aux) - lfor[1]*np.sin(aux), lfor[0]*np.sin(aux) + lfor[1]*np.cos(aux)]
    #print(f"LOCAL: {lpos} - {round_list(new_lfor)} | {round_list(new_lrig)}, GLOBAL: {np.matmul(ltow, lpos[:2] + [1])} - {round_list(list(np.matmul(ltow, new_lfor + [0])[:2]))} | {round_list(list(np.matmul(ltow, new_lrig + [0])[:2]))}")
    print(f"LOCAL: {lpos} - {round_list(new_lfor)}, GLOBAL: {np.matmul(ltow, lpos[:2] + [1])} - {round_list(list(np.matmul(ltow, new_lfor + [0])[:2]))}")

# Test 1
x = [0,20,40,80,99,100]#[0,20,40,60,80,100]
y = [0,0,-20,20,0,0]#[0,0,20,0,-20,0]



deg = len(x)-1
coefficients = np.polyfit(x,y,deg)
polynomial = np.poly1d(coefficients)
x_values = np.linspace(min(x), max(x), 100)
y_values = polynomial(x_values)

hermite_interpolator = PchipInterpolator(x, y)
x_values = np.linspace(min(x), max(x), 100)
y_values = hermite_interpolator(x_values)

angle    = 0
pos      = Vector2(x_values[0], y_values[0])
a_values = [0, 0, 0]
for i in range(1,99):
    next_pos = Vector2(x_values[i], y_values[i])
    next_next_pos = Vector2(x_values[i+1], y_values[i+1])
    next_dir = next_pos - pos
    next_next_dir = next_next_pos - next_pos
    print(next_pos, "--", a_values)
    a_values = a_values[1:] + [next_dir.sense(next_next_dir) * next_dir.angle(next_next_dir, "RAD")]




plt.figure(figsize=(8,6))
plt.plot(x_values, y_values, label="Polinomio interpolado", color="blue")
plt.scatter(x, y, label="Puntos conocidos", color="red")
plt.xlabel("x")
plt.ylabel("y")
plt.title("Interpolacion polinomica")
plt.legend()
plt.axis("equal")
plt.grid(True)
plt.show()

# Test 2
#x_lineal = [1,2]
#y_lineal = [2,4]
#x_curval = [3,4,5,6]
#y_curval = [5,8,10,12]
#spline_lineal = CubicSpline(x_lineal, y_lineal)
#spline_curval = CubicSpline(x_curval, y_curval)
#
#x_values = np.linspace(min(x_lineal), max(x_curval), 100)
#y_values_lineal = spline_lineal(x_values)
#y_values_curval = spline_curval(x_values)
#
#plt.figure(figsize=(8,6))
#plt.plot(x_values, y_values_lineal, label="IL", color="blue")
#plt.plot(x_values, y_values_curval, label="IC", color="green")
#plt.scatter(x_lineal, y_lineal, label="PL", color="red")
#plt.scatter(x_curval, y_curval, label="PC", color="orange")
#plt.xlabel("x")
#plt.ylabel("y")
#plt.title("Interpolacion")
#plt.grid(True)
#plt.legend()
#plt.show()
#points = [[20,0], [40,20], [60,0], [80,-20], [100,0]]
#
#v = 10
#w =  1
#
#STATE = "START_POINT_ADVENTURE"
#x, y, th, bh = self.readOdometry()
#position = Vector2(0,0,1) # Vector2(x, y)
#for point in points:
#
#    transform = Transform(Vector2(x, y), th)
#    forward   = Vector2.right.rotate(th)
#
#    if STATE == "START_POINT_ADVENTURE":
#        next_position = Vector2(point[0], point[1], 1)
#        direction     = next_position - position
#        print(f"{next_position}, {direction}, {forward.sense(direction)*forward.angle(direction)}")
#        STATE = "MOVE_TO_POINT"
#        self.setSpeed(v, forward.sense(direction)*w, 0)
#    elif STATE == "MOVE":
#   
#    position      = next_position

exit(0)

gx = 20
gy = 20
gth = 90
x = 0
y = 0
th = 0

ltow = Matrix2.transform(Vector2(gx, gy, 0), gth)

# Extraemos la matriz de rotación R y el vector de traslación p
print(ltow)

print(ltow.invert())

print(ltow*Vector2(x, y, 1))

print(ltow.invert()*Vector2(gx, gy, 1))

#lpos = [0,0,0]
#gref = [20,20,90]
#th = gref[2] * np.pi/180
#ltow = [
#    [np.cos(th), -np.sin(th), gref[0]],
#    [np.sin(th),  np.cos(th), gref[1]],
#    [         0,           0,       1]
#]
#th = -np.pi/2
#ltow2 = [
#    [np.cos(th), -np.sin(th), 0],
#    [np.sin(th),  np.cos(th), 0],
#    [0, 0, 1]
#]
#ltow = np.matmul(ltow2, ltow)
#
#aux = lpos[2]*np.pi/180
#new_lfor = [1*np.cos(aux) - 0*np.sin(aux), 1*np.sin(aux) + 0*np.cos(aux), 0]
#
#print(np.matmul(ltow, new_lfor))


#print("---------------------------------------------------")
#rMap = Map("maps/mapa0.txt", [0,0], [2,2])
#
#print("---------------------------------------------------")
#rMap = Map("maps/mapa1.txt", [0,0], [2,2])
#
#print("---------------------------------------------------")
#rMap = Map("maps/mapa2.txt", [0,0], [4,6])

#print("---------------------------------------------------")
rMap = Map("maps/mapa3.txt", [4,2], [0,7], neighborhood=4)

#print("---------------------------------------------------")
#rMap = Map("maps/mapa4.txt", [0,0], [0,1], neighborhood=4)
np.set_printoptions(precision=2, suppress=True)
#rMap.drawMapWithRobotLocations()
exit(0)


cell = [0,0]
shift = [1,1]
# A dx le corresponde a 'y' y dy a 'x' ya que el mapa es:
#   ^ oy
#   . > ox  y en la matriz la dimensiones son [x][y] no [y][x]
dx, dy = int(round(shift[0])), int(round(shift[1]))
conn = [2*cell[0]+1, 2*cell[1]+1]
neighbor_conn = [conn[0] + dx, conn[1] + dy]

# Si detecto obstaculo
rMap.connectionMatrix[neighbor_conn[0]][neighbor_conn[1]]       = 0
rMap.connectionMatrix[neighbor_conn[0]-(1-dx)][neighbor_conn[1]-(1-dy)] = 0
rMap.connectionMatrix[neighbor_conn[0]+(1-dx)][neighbor_conn[1]+(1-dy)] = 0

print(rMap)


exit(0)


gref = [20,20,90]
gfor = Vector2.right
grig = Vector2.up

ltow = Matrix2.transform(Vector2(gref[0], gref[1], 0), gref[2])
lref = [0,0,0]
lpos = Vector2(lref[0], lref[1], 1)
lfor = Vector2.right
lrig = Vector2.up

#print("POSICION LOCAL:", lpos)
#print("BASE LOCAL:", lfor, "|", lrig)
#print("POSICION GLOBAL:", ltow * lpos)
#print("BASE GLOBAL (L):", ltow*lfor, "|", ltow*lrig)
#print("BASE GLOBAL (G):", gfor, "|", grig)

# ESTO ES LO QUE VA A SER
position_transform, rotation_transform = None, None
prev_i, prev_cell, prev_pos = rMap.travel()
state  = "RECOGN"
x,y,th = 0,0,0 

def test(key):
    global rMap
    global x, y, th
    global state
    global prev_i, prev_cell, prev_pos
    global position_transform, rotation_transform

    if key == Key.right:
        th = (th+90) % 360
    elif key == Key.left:
        th = (th-90) % 360
    elif key == Key.up:
        if (th ==  0):
            x += 1
        if (th == 180):
            x -= 1
        if (th == 90):
            y += 1
        if  (th == 270):
            y -= 1
    elif key == Key.down:
        if (th ==  0):
            x -= 1
        if (th == 180):
            x += 1
        if (th == 90):
            y -= 1
        if  (th == 270):
            y += 1
    elif key == Key.esc:
        exit()
    
    lpos = Vector2(x, y, 1)
    lfor = Vector2.right.rotate(th)
    gpos = ltow * lpos
    gfor = ltow * lfor
    transform = Transform(gpos, forward=gfor)
    print("---------------------------")
    #print("LOCAL BASE:", lpos, lfor)
    #print("GLOBAL BASE:", gpos, gfor)
    print(state)
    print(transform)
    print(position_transform)
    print(rotation_transform)
    print("---------------------------")
    print()
    if state == "RECOGN":
        if prev_cell == rMap.goal:
            print("Goal reached!")
            print(prev_cell, rMap.goal)
            exit(0)
        # Aqui usais el sensor y recalcular el nuevo path con el nuevo mapa
        # rMap.redo_path_8n()
        if rMap.path:
            next_i, next_cell, next_pos = rMap.travel()
            position_transform = Transform(next_pos, forward=(next_pos - prev_pos).normalize())
            rotation_transform = Transform(prev_pos, forward=(next_pos - prev_pos).normalize())
            if not transform == rotation_transform:
                state = "ROTATE"
            else:
                state = "MOVE"
            prev_i, prev_cell, prev_pos = next_i, next_cell, next_pos
    elif state == "ROTATE":
        if transform == rotation_transform:
            state = "MOVE"
    elif state == "MOVE":
        if transform == position_transform:
            state = "RECOGN"


with keyboard.Listener(on_release=test) as listener:
    listener.join()

exit(0)

# GLOBAL x,y,th
gref = [20,20,90]

ltowA = Matrix3.transform(Vector3(gref[0],gref[1],0), gref[2], "Z-AXIS")
ltowB = Matrix3.transform(Vector3.zero, -180)

th = np.deg2rad(gref[2])
ltowC = [
    [np.cos(th), -np.sin(th), 0, gref[0]],
    [np.sin(th),  np.cos(th), 0, gref[1]],
    [         0,           0, 1,       0],
    [         0,           0, 0,       1]
]
th = -np.pi
ltowD = [
    [1,          0,           0, 0],
    [0, np.cos(th), -np.sin(th), 0],
    [0, np.sin(th),  np.cos(th), 0],
    [0,           0,          0, 1]
]

#print(np.array(ltowA.A))
#print()
#print(np.array(ltowC))
#print()
#print()
#print(np.array(ltowB.A))
#print()
#print(np.array(ltowD))
#print()
#print()
#ltow = np.matmul(ltowC, ltowD)
ltow = ltowA * ltowB

# Robot
x,y,th  = 0,0,0
lpos = Vector3(x, y, 0, 1)
gpos = ltow * lpos
lfor = Vector3.right
lrig = Vector3.up
gfor = ltow * lfor
grig = ltow * lrig

A = Vector3((1+rMap.start[0])*rMap.halfCell, (1+rMap.start[1])*rMap.halfCell, 0, 1)
B = Vector3((1+rMap.path[-2][0])*rMap.halfCell, (1+rMap.path[-2][1])*rMap.halfCell, 0, 1)
dir = B - gpos
angle = vector2(gfor).angle(vector2(dir), "DEG")

print("LPOS:", lpos)
print("GPOS:", gpos)
print("LBAS:", lfor, " | ", lrig)
print("GBAS:", gfor, " | ", grig)
print("INIT:", A)
print("NEXT:", B)
print("NDIR:", dir)

print("ANGLE:", angle)
print(vector2(gfor).rotate(angle))


# ESTO ES LO QUE VA A SER
# state = "RECOGNITION"
# step  = 1
# prev_position = Vector2.zero
# position_transform: Transform = None
# rotation_transform: Transform = None
# cell = [0,0]
# 
# while True:
#     x, y, th, _  = robot.readOdometry()
#     gpos = vector2(ltow * Vector3(x,y,0,1))
#     gfor = vector2(ltow * robot.forward)
#     transform = Transform(vector2(gpos), th)
#     # Analizando entorno de la casilla actual
#     # En el futuro se prevee hacerlo a la vez para calcular el path a priori.
#     # Hacer a priori el path, lo de position_transform y rotation_transform 
#     # debe ser innamovible a no ser que puedan poner obstaculos antes de arrancar..
#     if state == "RECOGNITION":
#         if cell == rMap.goal:
#             break
#         # Aqui usais el sensor y recalcular el mapa encontrando el nuevo path
#         # ...
#         # Obteniendo transformaciones para la siguiente casilla
#         cell, next_position = rMap.getPath(-step)
#         print(step, next_position)
#         position_transform = Transform(next_position, 0)
#         rotation_transform = Transform(next_position, gfor.angle(next_position - prev_position))
#         prev_position = next_position
#         state = "ROTATE"
#     # Rotando sobre la casilla actual
#     elif state == "ROTATE":
#         if transform == rotation_transform:
#             state = "MOVE"
#     # Moviendose a la siguiente casilla
#     elif state == "MOVE":
#         if transform == position_transform:
#             step += 1
#             state = "RECOGNITION"
