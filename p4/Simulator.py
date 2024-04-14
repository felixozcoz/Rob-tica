import numpy as np
from geometry import vector2, Vector3, Matrix3, Transform
from pynput import keyboard
from pynput.keyboard import Key
from ReMapLib import Map

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

print("---------------------------------------------------")
rMap = Map("maps/mapa3.txt", [0,0], [4,7])
np.set_printoptions(precision=2, suppress=True)

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

print(np.array(ltowA.A))
print()
print(np.array(ltowC))
print()
print()
print(np.array(ltowB.A))
print()
print(np.array(ltowD))
print()
print()
ltow = np.matmul(ltowC, ltowD)
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
#x, y, th, _  = robot.readOdometry()
#transform    = Transform(gpos, angle)
#
#STATE = "SCAN"
#STATE = "MOVE"
#STATE = "ROTATE"


#for i, cell in enumerate(reversed(rMap.path[:-1])):
#
#    # En la version final se hace esto:
#    # gpos = ltow * Vector3(x, y, 0, 1)
#    lpos = Vector3(x, y, 0, 1)
#    gpos = ltow * lpos
#
#    pos = Vector3((1+cell[0])*rMap.halfCell, (1+cell[1])*rMap.halfCell, 0, 1)
#    
#    print()
#    print("|", lpos, gpos)
#    print("└", i, step, pos, pos-step)

#with keyboard.Listener(on_release=simulate_robot) as listener:
#    listener.join()
