import numpy as np

mapF = open("mapa0.txt", "r")
# 1. Special case for first line. Initialize dimX dimY cellSize
header = mapF.readline()
header = header.split()
if len(header) != 3:
    exit(1)

sizeX, sizeY, sizeCell = [int(c) for c in header]
sizeCell //= 10
print(sizeX, sizeY, sizeCell)

# 2. Obtener conexiones entre celdas
connectionMatrix = np.zeros((2*sizeX+1, 2*sizeY+1))
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

print(connectionMatrix)
mapF.close()

# 3. Obtener costes:
goalX, goalY = [sizeX-1, sizeY-1]
costMatrix = np.zeros((sizeX, sizeY))

cell = [goalX, goalY]
print("Cell:", cell)

#for dx in [-1,1]:
#    for dy in [-1,1]:
#        print("Conn:", [conn[0]+dx, conn[1]+dy], "Cell:", [cell[0]+dx, cell[1]+dy])


border = [[2*goalX+1, 2*goalY+1]]
while border:
    conn = border.pop(0)
    for dx in [-1,1]:
        if connectionMatrix[conn[0]+dx][conn[1]]:
            print("Cell", [cell[0]+dx, cell[1]], "is free through connection", [conn[0]+dx, conn[1]])
        if connectionMatrix[conn[0]][conn[1]+dx]:
            print("Cell", [cell[0], cell[1]+dx], "is free through connection", [conn[0], conn[1]+dx])
    

#def getNeihbors(p):
#    return {
#        0: [ p[0]-1, p[1]   ],
#        1: [ p[0]-1, p[1]+1 ],
#        2: [ p[0]  , p[1]+1 ],
#        3: [ p[0]+1, p[1]+1 ],
#        4: [ p[0]+1, p[1]   ],
#        5: [ p[0]+1, p[1]-1 ],
#        6: [ p[0]  , p[1]-1 ],
#        7: [ p[0]-1, p[1]-1 ],
#    }

#border = [p]
#while border:
#    print("Current:", border[0])
#    neighbors = getNeihbors(border[0])
#    for i in range(0,7,2):
#        print(f" Neighbor {i}: {neighbors[i]}")
#        if connectionMatrix[neighbors[i][0]][neighbors[i][1]]:
#            print("  Free")
#            
#        else:
#            print("  Blocked")
#    border.pop(0)



#
#for i in range(0,7,2):
#    print(result[i])
#    if connectionMatrix[result[i][0]][result[i][1]]:
#
#        print("Blocked")
#
#    else:
#        print("Available")



