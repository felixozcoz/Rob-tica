import bisect
import numpy as np
import time
import json
import geometry
from ReMapLib import Map
from pynput import keyboard
from pynput.keyboard import Key

def inside(cell):
    return cell[0] >= 0 and cell[0] < sizeX and cell[1] >= 0 and cell[1] < sizeY

def another_insert(a_list, a_value):
    for i, e in reversed(list(enumerate(a_list))):
        if (a_value[0] < e[0]) or (a_value[0] == e[0] and a_value[1] < e[1]):
            i += 1
            break
    return a_list[0:i] + [a_value] + a_list[i:]

def print_map(a_map):
    for i, row in enumerate(a_map):
        for j, cell in enumerate(row):
            if not cell:
                    print("■", end=" ")
            else:
                if i%2==1 and j%2==1:
                    print(".", end=" ")
                else:
                    print("□", end=" ")
        print()

def print_path(a_map, a_path, start, goal):
    for i, row in enumerate(a_map):
        for j, cell in enumerate(row):
            if not cell:
                    print("■", end=" ")
            else:
                if i%2==1 and j%2==1:
                    cell = [(i-1)//2,(j-1)//2]
                    if cell in a_path:
                        if cell == goal:
                            print("⚑", end=" ")
                        elif cell == start:
                            print("○", end=" ")
                        else:
                            print("●", end=" ")
                    else:
                        print(".", end=" ")
                else:
                    print("□", end=" ")
        print()

def cross(v, w):
    return (v[0] * w[1]) - (v[1] * w[0])

def dot(v, w):
    return v[0]*w[0] + v[1]*w[1]

def magnitude(v):
    return np.sqrt(v[0]*v[0] + v[1]*v[1])

def normalize(v):
    m = magnitude(v)
    return [v[0]/m, v[1]/m]

def angle(v, w, format="DEG"):
    res = np.acos(dot(v,w) / (magnitude(v) * magnitude(w)))
    if (format == "DEG"):
        res *= 180 / np.pi
    return res

def rotate(v, angle, format="DEG"):
    if format == "DEG":
        angle *= np.pi/180
    return [v[0]*np.cos(angle) - v[1]*np.sin(angle), v[0]*np.sin(angle) + v[1]*np.cos(angle)]

def insert(a_list, a_node):
    if not a_list:
        return [a_node]
    else:
        a_value = a_node["coords"]
        for i, e_node in enumerate(a_list):
            e_value = e_node["coords"]
            #print(h[a_value[0]][a_value[1]], h[e_value[0]][e_value[1]])
            if costMatrix[a_value[0]][a_value[1]] < costMatrix[e_value[0]][e_value[1]]:
                break
        return a_list[:i] + [a_node] + a_list[i:]

def propagate_v4(goal):
    cost            = 0
    end_propagation = False
    border          = [goal]
    old_border      = []
    new_border      = []
    now = time.time()
    while not end_propagation:
        # Variables para parar
        end_propagation = True
        # Nueva frontera
        new_border      = []
        for cell in reversed(border):
            conn = [2*cell[0]+1, 2*cell[1]+1]
            costMatrix[cell[0]][cell[1]] = cost
            for dx in [-1,1]:
                for dy in [0,1]:
                    neighbor           = [cell[0], cell[1]]
                    neighbor[dy]      += dx
                    neighbor_conn      = [conn[0], conn[1]]
                    neighbor_conn[dy] += dx 
                #for neighbor in [[cell[0]+shift, cell[1]], [cell[0], cell[1]+shift]]:
                    if connectionMatrix[neighbor_conn[0]][neighbor_conn[1]] and neighbor not in border and neighbor not in old_border and neighbor not in new_border:
                        end_propagation = False
                        bisect.insort(new_border, neighbor)
        # Actualizamos la frontera
        old_border = border
        border = new_border
        cost += 1

    print(f"{round((time.time() - now)*1000000.0, 4)}ns")
    print(costMatrix)

def propagate_v8(goal):
    # Variables para la propagacion por fronteras
    cost            = 0
    end_propagation = False
    border          = [goal]
    old_border      = []
    new_border      = []
    # Propagando el coste
    while not end_propagation:
        # Si no se introduce al menos una nueva celda, deja de ejecutarse
        end_propagation = True
        # Frontera para la siguiente iteracion
        new_border      = []
        for cell in reversed(border):
            conn = [2*cell[0]+1, 2*cell[1]+1]
            costMatrix[cell[0]][cell[1]] = cost
            for dx in [-1,0,1]:
                for dy in [-1,0,1]:
                    neighbor      = [cell[0]+dx, cell[1]+dy]
                    neighbor_conn = [conn[0]+dx, conn[1]+dy]
                    if connectionMatrix[neighbor_conn[0]][neighbor_conn[1]] and neighbor not in border and neighbor not in old_border and neighbor not in new_border:
                        end_propagation = False
                        bisect.insort(new_border, neighbor)
        # Actualizamos la frontera para la siguiente iteracion
        old_border = border
        border = new_border
        cost += 1

    print(costMatrix)

def find_path_v4(start, goal):
    # https://medium.com/@nicholas.w.swift/easy-a-star-pathfinding-7e6689c7f7b2
    path   = []
    border = [{
        "parent": None,
        "coords": start,
    }]
    expand = []

    while border:
        node = border.pop(0)
        cell = node["coords"]
        if (cell == goal):
            while node:
                path.append(node["coords"])
                node = node["parent"]
            break
        #
        conn = [2*cell[0]+1, 2*cell[1]+1]
        expand.append(cell)
        for dx in [-1,1]:
            for dy in [0,1]:
                neighbor           = [cell[0], cell[1]]
                neighbor[dy]      += dx
                neighbor_conn      = [conn[0], conn[1]]
                neighbor_conn[dy] += dx
                if connectionMatrix[neighbor_conn[0]][neighbor_conn[1]] and neighbor not in expand:
                    border = insert(border, {"parent": node, "coords":neighbor})
    return path

def find_path_v8(start, goal):
    # https://medium.com/@nicholas.w.swift/easy-a-star-pathfinding-7e6689c7f7b2
    path   = []
    border = [{
        "parent": None,
        "coords": start,
    }]
    expand = []

    while border:
        node = border.pop(0)
        cell = node["coords"]
        if (cell == goal):
            while node:
                path.append(node["coords"])
                node = node["parent"]
            break
        #
        conn = [2*cell[0]+1, 2*cell[1]+1]
        expand.append(cell)
        for dx in [-1,0,1]:
            for dy in [1,0,1]:
                neighbor      = [cell[0]+dx, cell[1]+dy]
                neighbor_conn = [conn[0]+dx, conn[1]+dy]
                if not (dx == 0 and dy == 0) and connectionMatrix[neighbor_conn[0]][neighbor_conn[1]] and neighbor not in expand:
                    border = insert(border, {"parent": node, "coords":neighbor})

    return path

#lref = [0,0,0]
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
#aux = lref[2]*np.pi/180
#new_lfor = [1*np.cos(aux) - 0*np.sin(aux), 1*np.sin(aux) + 0*np.cos(aux), 0]
#
#print(np.matmul(ltow, new_lfor))
#exit(0)


print("---------------------------------------------------")
map = Map("maps/mapa0.txt", [0,0], [2,2])

print("---------------------------------------------------")
map = Map("maps/mapa1.txt", [0,0], [2,2])

print("---------------------------------------------------")
map = Map("maps/mapa2.txt", [0,0], [4,6])

print("---------------------------------------------------")
map = Map("maps/mapa3.txt", [0,0], [4,7])

exit(0)

### MAIN
for i in range(0,1):
    print("---------------------------------------------------")
    print(f"Mapa {i}")
    mapF = open(f"maps/mapa{i}.txt", "r")
    # 1. Special case for first line. Initialize dimX dimY cellSize
    header = mapF.readline()
    header = header.split()
    if len(header) != 3:
        exit(1)

    sizeX, sizeY, sizeCell = int(header[0]), int(header[1]), int(header[2])//10
    print(sizeX, sizeY, sizeCell)
    print(2*sizeX+1, 2*sizeY+1)

    # 2. Obtener conexiones entre celdas
    connectionMatrix = np.zeros((2*sizeY+1, 2*sizeX+1))
    row = 0
    while row < connectionMatrix.shape[0]:
        # Se obtienen los valores de la fila actual
        connections = mapF.readline().split()
        if len(connections) == 0:
            continue
        if not len(connections) == connectionMatrix.shape[1]:
            break
        # Se guarda el valor obtenido
        connectionMatrix[row] = [int(c) for c in connections]
        row += 1

    print_map(connectionMatrix)
    mapF.close()

    # 3. Obtener costes:
    costMatrix = np.zeros((sizeY, sizeX))

    #gpos = self.localToGlobal * Vector2(self.x.value, self.y.value, 1)
    #grot = Vector2.up.angle(self.localToGlobal * Matrix2.transform(Vector2.zero, self.th.value) * Vector2.up)

    start = [0,0]
    goal  = [sizeY-1,sizeX-1]
    goal  = [sizeY-1,sizeX-1]
    print(f"Start: {start}, Goal: {goal}")

    print("\nVECINDAD 4")
    now = time.time()
    propagate_v4(goal)
    path = find_path_v4(start, goal)
    print(f"{round((time.time() - now)*1000000.0, 4)}ns")
    print(path)
    print_path(connectionMatrix, path, start, goal)

    #print("\nVECINDAD 8")
    #now = time.time()
    #propagate_v8(goal)
    #path = find_path_v8(start, goal)
    #print(f"{round((time.time() - now)*1000000.0, 4)}ns")
    ##print(path)
    #print_path(connectionMatrix, path, start, goal)

    #Robot = {
    #    "gref": [20,20, 0],
    #    "lref": [ 0, 0, 0],
    #    "orientation": [1,0] 
    #}

    lref = [0,0,0]
    gref = [20,20,90]
    th = gref[2] * np.pi/180
    ltow = [
        [np.cos(th), -np.sin(th), gref[0]],
        [np.sin(th),  np.cos(th), gref[1]],
        [         0,           0,       1]
    ]
    th = -np.pi/2
    ltow2 = [
        [np.cos(th), -np.sin(th), 0],
        [np.sin(th),  np.cos(th), 0],
        [0, 0, 1]
    ]
    ltow = np.matmul(ltow2, ltow)

    forward = [1,0,0]
    for i in range(len(path)-1,0,-1):
        cell = path[i]
        A  = [(1+cell[0])*sizeCell//2, (1+cell[1])*sizeCell//2]
        cell = path[i-1]
        B  = [(1+cell[0])*sizeCell//2, (1+cell[1])*sizeCell//2]
        AB = [B[0]-A[0],B[1]-A[1]]
        print("CURRENT STEP")
        print(f"- Current/Next position: {A}/{B}")
        print(f"- Forward local/global orientation: {forward[:2]}/{np.matmul(ltow, forward)[:2]}")

    #for i in range(len(path)-1,0,-1):
    #    cell = path[i]
    #    A = [(1+cell[0])*sizeCell//2, (1+cell[1])*sizeCell//2]
    #    cell = path[i-1]
    #    B = [(1+cell[0])*sizeCell//2, (1+cell[1])*sizeCell//2]
    #    AB = [B[0]-A[0],B[1]-A[1]]
    #    print(A, B, normalize(AB))

    print()

    # Para testear los algoritmos
    #print("\nPROPAGATE V4")
    #for i in range(0,sizeX):
    #    for j in range(0, sizeY):
    #        print(f"== [{i},{j}] =========================")
    #        propagate_v4_0([i,j])
    #        print("==================================")

    #now = time.time()
    #print("\nPROPAGATE V8")
    #for i in range(0,sizeX):
    #    for j in range(0, sizeY):
    #        print(f"== [{i},{j}] =========================")
    #        propagate_v8([i, j])
    #        print("==================================")
    #print(f"{round((time.time() - now)*1000.0, 4)}ms")



def round_list(a_list):
    for i,v in enumerate(a_list):
        a_list[i] = round(v,2)
        if a_list[i] == 0:
            a_list[i] = abs(a_list[i])
    return a_list

lref = [0,0,0]
lfor = [1,0]
lrig = [0,1]

gref = [20,20,90]
th = gref[2] * np.pi/180
ltow = [
    [np.cos(th), -np.sin(th), 0, gref[0]],
    [np.sin(th),  np.cos(th), 0, gref[1]],
    [         0,           0, 1,       0],
    [         0,           0, 0,       1]
]
th = -np.pi/2
ltow2 = [
    [1,          0,           0, 0],
    [0, np.cos(th), -np.sin(th), 0],
    [0, np.sin(th),  np.cos(th), 0],
    [         0,           0, 0, 1]
]
ltow = np.matmul(ltow, ltow2)
forward = [1,0,0,0]
print(round_list(np.matmul(ltow, forward)))

exit(0)
     

def on_key_release(key):
    if key == Key.right:
        lref[2] = (lref[2]+90) % 360
    elif key == Key.left:
        lref[2] = (lref[2]-90) % 360
    elif key == Key.up:
        if (lref[2] ==  0):
            lref[0] += 1
        if (lref[2] == 180):
            lref[0] -= 1
        if (lref[2] == 90):
            lref[1] += 1
        if  (lref[2] == 270):
            lref[1] -= 1
    elif key == Key.down:
        if (lref[2] ==  0):
            lref[0] -= 1
        if (lref[2] == 180):
            lref[0] += 1
        if (lref[2] == 90):
            lref[1] -= 1
        if  (lref[2] == 270):
            lref[1] += 1
    elif key == Key.esc:
        exit()
    
    aux = lref[2]*np.pi/180
    #new_lrig = [lrig[0]*np.cos(aux) - lrig[1]*np.sin(aux), lrig[0]*np.sin(aux) + lrig[1]*np.cos(aux)]
    new_lfor = [lfor[0]*np.cos(aux) - lfor[1]*np.sin(aux), lfor[0]*np.sin(aux) + lfor[1]*np.cos(aux)]
    #print(f"LOCAL: {lref} - {round_list(new_lfor)} | {round_list(new_lrig)}, GLOBAL: {np.matmul(ltow, lref[:2] + [1])} - {round_list(list(np.matmul(ltow, new_lfor + [0])[:2]))} | {round_list(list(np.matmul(ltow, new_lrig + [0])[:2]))}")
    print(f"LOCAL: {lref} - {round_list(new_lfor)}, GLOBAL: {np.matmul(ltow, lref[:2] + [1])} - {round_list(list(np.matmul(ltow, new_lfor + [0])[:2]))}")

with keyboard.Listener(on_release=on_key_release) as listener:
    listener.join()