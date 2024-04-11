import bisect
import numpy as np
import time
import json 
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
                    if connectionMatrix[neighbor_conn[0]][neighbor_conn[1]] and not neighbor in border and not neighbor in old_border and not neighbor in new_border:
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
        print(cost, border)
        end_propagation = True
        # Frontera para la siguiente iteracion
        new_border      = []
        for cell in reversed(border):
            conn = [2*cell[0]+1, 2*cell[1]+1]
            costMatrix[cell[0]][cell[1]] = cost
            for dx in [-1,0,1]:
                for dy in [-1,0,1]:
                    neighbor      = [cell[0]+dx, cell[1]+dy]
                    neighbor_conn = [conn[0]+dx, conn[0]+dy]
                    if connectionMatrix[neighbor_conn[0]][neighbor_conn[1]] and not neighbor in border and not neighbor in old_border and not neighbor in new_border:
                        end_propagation = False
                        bisect.insort(new_border, neighbor)
        # Actualizamos la frontera para la siguiente iteracion
        old_border = border
        border = new_border
        cost += 1

    print(costMatrix)

    
def dot(v, w):
    return v[0]*w[0] + v[1]*w[1]

def magnitude(v):
    return np.sqrt(v[0]*v[0] + v[1]*v[1])

def normalize(v):
    m = magnitude(v)
    return [v[0]/m, v[1]/m]

def angle(v, w, format = 'RAD'):
    res = np.acos((self * v) / (magnitude(v) * magnitude(w)))
    if (format == "DEG"):
        res *= 180 / np.pi
    return res

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

def find_path(start, goal):
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
        expand.append(node)
        for dx in [-1,1]:
            for dy in [0,1]:
                neighbor           = [cell[0], cell[1]]
                neighbor[dy]      += dx
                neighbor_conn      = [conn[0], conn[1]]
                neighbor_conn[dy] += dx
                if connectionMatrix[neighbor_conn[0]][neighbor_conn[1]]:
                    border = insert(border, {"parent": node, "coords":neighbor})

    print(path)
    print_path(connectionMatrix, path, start, goal)
    return path

### MAIN
mapF = open("mapa3.txt", "r")
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
print("Goal:", goal)
propagate_v4(goal)
path  = find_path(start, goal)

Robot = {
    "gref": [20,20, 0],
    "lref": [ 0, 0, 0],
    "orientation": [1,0] 
}

for i in range(len(path)-1,0,-1):
    cell = path[i]
    A = [(1+cell[0])*sizeCell//2, (1+cell[1])*sizeCell//2]
    cell = path[i-1]
    B = [(1+cell[0])*sizeCell//2, (1+cell[1])*sizeCell//2]
    AB = [B[0]-A[0],B[1]-A[1]]
    print(A, B, normalize(AB))

print()

# Para testear los algoritmos
#now = time.time()
#print("\nPROPAGATE V4")
#for i in range(0,sizeX):
#    for j in range(0, sizeY):
#        print(f"== [{i},{j}] =========================")
#        propagate_v4_0([i,j])
#        print("==================================")
#print(f"{round((time.time() - now)*1000.0, 4)}ms")

#now = time.time()
#print("\nPROPAGATE V8")
#for i in range(0,sizeX):
#    for j in range(0, sizeY):
#        print(f"== [{i},{j}] =========================")
#        propagate_v8([i, j])
#        print("==================================")
#print(f"{round((time.time() - now)*1000.0, 4)}ms")
