#!/usr/bin/python
# -*- coding: UTF-8 -*-
from __future__ import print_function # use python 3 syntax but make it compatible with python 2
from __future__ import division       #                           ''
from geometry import Vector2
from matplotlib import animation

import bisect
import matplotlib.pyplot as plt
import numpy as np
import re
import time


class Map:
    # Constructor
    def __init__(self, path, start, goal, neighborhood=8):
        """
            Load and initialize map from file.

            path: path to a text file containing map description in the standard format. \
            Example for a 3x3 grid map, with (squared) cells of 400mm side length called mapa0. \
            All free space, i.e., all connections between cells are open, except those on the limits of the map

                mapa0.txt content:
                3 3 400
                0 0 0 0 0 0 0
                0 1 1 1 1 1 0
                0 1 1 1 1 1 0
                0 1 1 1 1 1 0
                0 1 1 1 1 1 0
                0 1 1 1 1 1 0
                0 0 0 0 0 0 0
        """
        # Parametros para visualizar
        self.mapLineStyle="r-"
        self.costValueStyle="g*"
        self.verbose = True
        # set to False to stop displaying plots interactively (and maybe just save the screenshots)
        # self.verbose = False
        self.current_ax = None

        # LECTURA DEL MAPA
        self.name = re.split(r'[\\/]', path)[-1]
        mapF = open(path, "r")

        # Obtener dimensiones del mapa y de las celdas
        header = mapF.readline().split()
        if len(header) != 3:
            print("Error -- El encabezado '" + ' '.join(header) + "' tiene un formato incorrecto...")
            exit(1)
        self.sizeX = int(header[0])
        self.sizeY = int(header[1])
        self.sizeCell = int(header[2])//10
        self.halfCell = self.sizeCell//2

        # Obtener conexiones entre celdas
        self.connectionMatrix = np.zeros((2*self.sizeY+1, 2*self.sizeX+1))
        rows = 0
        while rows < self.connectionMatrix.shape[0]:
            rows += 1
            # Se obtienen los valores de la fila actual
            connections = mapF.readline().split()
            if len(connections) == 0:
                continue
            if not len(connections) == self.connectionMatrix.shape[1]:
                print("Warning -- La linea " + str(rows) + " tiene dimensiones incorrectas")
                continue
            # Se guarda el valor obtenido
            self.connectionMatrix[-rows] = [int(e) for e in connections]
        if not rows == self.connectionMatrix.shape[0]:
            print("Error -- El mapa tiene el formato incorrecto")

        # Obtener la matriz de costes y el mejor camino
        self.costMatrix = np.zeros((self.sizeY, self.sizeX))
        self.path       = []
        self.index      = 1
        self.start      = start
        self.goal       = goal

        if neighborhood == 4:
            self.propagate_4n()
            self.find_path_4n()
        elif neighborhood == 8:
            self.propagate_8n()
            self.find_path_8n()

        if self.verbose:
            print(self)

    # 4-VECINDAD
    # Propagacion de costes
    def propagate_4n(self):
        """
            Propaga los costes a 4 vecindad y obtiene la matriz de costes
        """
        # Variables para la propagacion por fronteras
        # - Coste actual
        cost            = 0
        # - Condicion de terminacion del bucle
        end_propagation = False
        # - Frontera de propagacion del coste actual
        border          = [self.goal]
        # - Frontera de propagacion del coste anterior
        old_border      = []
        # - Frontera de propagacion del coste siguiente
        new_border      = []
        # Para verificar su coste
        #now = time.time()
        # Obtener matriz de costes
        while not end_propagation:
            # Forzamos la condicion de parada. Si no hay al menos una nueva celda
            # introducida en la nueva frontera, quiere decir que ya no quedan
            end_propagation = True
            # Nueva frontera
            new_border      = []
            for cell in reversed(border):
                conn = [2*cell[0]+1, 2*cell[1]+1]
                self.costMatrix[cell[0]][cell[1]] = cost
                for dx in [-1,1]:
                    for dy in [0,1]:
                        # Existe una propiedad graciosa en esta estructura y es que al sumar dx y dy a la celda
                        # se obtiene el vecino, pero al sumarselo a la conexion, da la direccion al vecino
                        neighbor = [cell[0], cell[1]]
                        neighbor[dy] += dx
                        neighbor_conn = [conn[0], conn[1]]
                        neighbor_conn[dy] += dx
                        # Si esta libre la conexion y no pertenece a ninguna frontera, es candidato a ser expandido
                        if self.connectionMatrix[neighbor_conn[0]][neighbor_conn[1]] and (neighbor not in border and neighbor not in old_border and neighbor not in new_border):
                            end_propagation = False
                            bisect.insort(new_border, neighbor)
            # Actualizamos la frontera
            old_border = border
            border = new_border
            cost += 1

        # print(str(round((time.time() - now)*1000000.0, 4)) + "ns")

    def find_path_4n(self):
        """
            Encuentra el camino menos costoso mediante A*. La heuristica es el coste, y en este
            caso la funcion de g(x) es 0.
        """
        # https://medium.com/@nicholas.w.swift/easy-a-star-pathfinding-7e6689c7f7b2
        # En este metodo los nodos cambian, necesitamos guardar el padre para poder
        # hacer backtracking y obtener el camino. 
        border = [{
            "parent": None,
            "coords": self.start
        }]
        # Nodos expandidos
        expand = []
        # Obtener el camino menos costoso
        while border:
            # Nodo actual a expandir
            node = border.pop(0)
            cell = node["coords"]
            # Si el nodo actual es la meta, hemos terminado
            if cell == self.goal:
                while node:
                    self.path.append(node["coords"])
                    node = node["parent"]
                break
            # Si no, obtenemos los vecinos del nodo actual
            conn = [2*cell[0]+1, 2*cell[1]+1]
            expand.append(cell)
            for dx in [-1,1]:
                for dy in [0,1]:
                    # Existe una propiedad graciosa en esta estructura y es que al sumar dx y dy a la celda
                    # se obtiene el vecino, pero al sumarselo a la conexion, da la direccion al vecino
                    neighbor = [cell[0], cell[1]]
                    neighbor[dy] += dx
                    neighbor_conn = [conn[0],conn[1]]
                    neighbor_conn[dy] += dx
                    # Si esta libre la conexion y no pertenece a ninguna frontera, es candidato a ser expandido
                    if self.connectionMatrix[neighbor_conn[0]][neighbor_conn[1]] and neighbor not in expand:
                        border = self.insert(border, {"parent": node, "coords": neighbor})

    def redo_path_4n(self, position):
        self.start = position
        self.path  = []
        self.index = 1
        self.propagate_4n()
        self.find_path_4n()


    # 8-VECINDAD
    # Propagacion de costes
    def propagate_8n(self):
        """
            Propaga los costes a 8 vecindad y obtiene la matriz de costes
        """
        # Variables para la propagacion por fronteras
        # - Coste actual
        cost            = 0
        # - Condicion de terminacion del bucle
        end_propagation = False
        # - Frontera de propagacion del coste actual
        border          = [self.goal]
        # - Frontera de propagacion del coste anterior
        old_border      = []
        # - Frontera de propagacion del coste siguiente
        new_border      = []
        # Para verificar su coste
        #now = time.time()
        # Obtener matriz de costes
        while not end_propagation:
            # Forzamos la condicion de parada. Si no hay al menos una nueva celda
            # introducida en la nueva frontera, quiere decir que ya no quedan
            end_propagation = True
            # Frontera para la siguiente iteracion
            new_border      = []
            for cell in reversed(border):
                conn = [2*cell[0]+1, 2*cell[1]+1]
                self.costMatrix[cell[0]][cell[1]] = cost
                for dx in [-1,0,1]:
                    for dy in [-1,0,1]:
                        # Existe una propiedad graciosa en esta estructura y es que al sumar dx y dy a la celda
                        # se obtiene el vecino, pero al sumarselo a la conexion, da la direccion al vecino
                        neighbor      = [cell[0]+dx, cell[1]+dy]
                        neighbor_conn = [conn[0]+dx, conn[1]+dy]
                        # Si esta libre la conexion y no pertenece a ninguna frontera, es candidato a ser expandido
                        if self.connectionMatrix[neighbor_conn[0]][neighbor_conn[1]] and (neighbor not in border and neighbor not in old_border and neighbor not in new_border):
                            end_propagation = False
                            bisect.insort(new_border, neighbor)
            # Actualizamos la frontera para la siguiente iteracion
            old_border = border
            border = new_border
            cost += 1

    def find_path_8n(self):
        """
            Encuentra el camino menos costoso mediante A*. La heuristica es el coste, y en este
            caso la funcion de g(x) es 0.
        """
        # https://medium.com/@nicholas.w.swift/easy-a-star-pathfinding-7e6689c7f7b2
        # En este metodo los nodos cambian, necesitamos guardar el padre para poder
        # hacer backtracking y obtener el camino. 
        border = [{
            "parent": None,
            "coords": self.start
        }]
        # Nodos expandidos
        expand = []
        # Obtener el camino menos costoso
        while border:
            # Nodo actual a expandir
            node = border.pop(0)
            cell = node["coords"]
            # Si el nodo actual es la meta, hemos terminado
            if cell == self.goal:
                while node:
                    self.path.append(node["coords"])
                    node = node["parent"]
                break
            # Si no, obtenemos los vecinos del nodo actual
            conn = [2*cell[0]+1, 2*cell[1]+1]
            expand.append(cell)
            for dx in [-1,0,1]:
                for dy in [-1,0,1]:
                    # Existe una propiedad graciosa en esta estructura y es que al sumar dx y dy a la celda
                    # se obtiene el vecino, pero al sumarselo a la conexion, da la direccion al vecino
                    neighbor = [cell[0]+dx, cell[1]+dy]
                    neighbor_conn = [conn[0]+dx, conn[1]+dy]
                    # Si esta libre la conexion y no pertenece a ninguna frontera, es candidato a ser expandido
                    if not (dx==0 and dy==0) and self.connectionMatrix[neighbor_conn[0]][neighbor_conn[1]] and neighbor not in expand:
                        border = self.insert(border, {"parent": node, "coords": neighbor})

    def redo_path_8n(self, position):
        self.start = position
        self.path  = []
        self.index = 1
        self.propagate_8n()
        self.find_path_8n()
    

    # DRAW MAP
    def _drawGrid(self):
        """
        aux function to create a grid with map lines
        """
        if not self.current_ax:
            print("Error plotting: do not call this function directly, \
                call drawMap first to create a plot where to draw")
            return False

        plt.rc('grid', linestyle="--", color='gray')
        plt.grid(True)
        plt.tight_layout()

        x_t = range(0, (self.sizeX+1)*self.sizeCell, self.sizeCell)
        y_t = range(0, (self.sizeY+1)*self.sizeCell, self.sizeCell)
        x_labels = [str(n) for n in x_t]
        y_labels = [str(n) for n in y_t]
        plt.xticks(x_t, x_labels)
        plt.yticks(y_t, y_labels)

        # Main rectangle
        X = np.array([0, self.sizeX, self.sizeX, 0,          0]) * self.sizeCell
        Y = np.array([0, 0,          self.sizeY, self.sizeY, 0]) * self.sizeCell
        self.current_ax.plot(X, Y, self.mapLineStyle)

        # "vertical" walls
        for i in range(2, 2*self.sizeX, 2):
            for j in range(1, 2*self.sizeY, 2):
                if not self.connectionMatrix[j,i]:
                    # paint "right" wall from cell (i-1)/2, (j-1)/2
                    cx= np.floor((i-1)/2)
                    cy= np.floor((j-1)/2)
                    X = np.array([cx+1, cx+1]) * self.sizeCell
                    Y = np.array([cy, cy+1]) * self.sizeCell
                    self.current_ax.plot(X, Y, self.mapLineStyle)

        # "horizontal" walls
        for j in range(2, 2*self.sizeY, 2):
            for i in range(1, 2*self.sizeX, 2):
                if not self.connectionMatrix[j,i]:
                    # paint "top" wall from cell (i-1)/2, (j-1)/2
                    cx=np.floor((i-1)/2)
                    cy=np.floor((j-1)/2)
                    X = np.array([cx, cx+1]) * self.sizeCell
                    Y = np.array([cy+1, cy+1]) * self.sizeCell
                    self.current_ax.plot(X, Y, self.mapLineStyle)
        plt.axis('equal')

        return True

    # aux functions to display the current CostMatrix on the map
    def _drawCostMatrix(self):
        """
        aux function to create a grid with map lines
        """
        if not self.current_ax:
            print("Error plotting: do not call this function directly, \
                call drawMap first to create a plot where to draw")
            return False

        # "center" of each cell
        for i in range(0, self.sizeX):
            for j in range(0, self.sizeY):
                    cx= i*self.sizeCell + self.sizeCell/2.
                    cy= j*self.sizeCell + self.sizeCell/2.
                    X = np.array([cx])
                    Y = np.array([cy])
                    cost = self.costMatrix[j,i]
                    self.current_ax.text(X, Y, str(cost))

        plt.axis('equal')

        return True

    # Dibuja robot en location_eje con color (c) y tamano (p/g)
    def _drawRobot(self, loc_x_y_th=[0,0,0], robotPlotStyle='b', small=False):
        """
        UPDATES existing plot to include current robot position
        It expects an existing open figure (probably with the map already on it)

        loc_x_y_th is the position x,y and orientation in mm and radians of the main axis of the robot

        """
        if not self.current_ax:
            print("Error plotting: do not call this function directly, \
                call drawMap first to create a plot where to draw")
            return False

        if small:
            largo, corto, descentre = [8, 5, 1]
        else:
            largo, corto, descentre = [16, 10, 1]

        trasera_dcha=np.array([-largo,-corto,1])
        trasera_izda=np.array([-largo,corto,1])
        delantera_dcha=np.array([largo,-corto,1])
        delantera_izda=np.array([largo,corto,1])
        frontal_robot=np.array([largo,0,1])

        tita=loc_x_y_th[2]
        Hwe=np.array([[np.cos(tita), -np.sin(tita), loc_x_y_th[0]],
                 [np.sin(tita), np.cos(tita), loc_x_y_th[1]],
                  [0,        0 ,        1]])

        Hec=np.array([[1,0,descentre],
                  [0,1,0],
                  [0,0,1]])

        extremos=np.array([trasera_izda, delantera_izda, delantera_dcha, trasera_dcha, trasera_izda, frontal_robot, trasera_dcha])
        robot=np.dot(Hwe, np.dot(Hec,np.transpose(extremos)))

        self.current_ax.plot(robot[0,:], robot[1,:], robotPlotStyle)

        return True

    def drawMapWithRobotLocations(self,
                                  robotPosVectors=[ [0,0,0], [60, 60, 0] ],
                                  saveSnapshot=True):
        """ Overloaded version of drawMap to include robot positions """
        return self.drawMap(robotPosVectors=robotPosVectors, saveSnapshot=saveSnapshot)

    def drawMap(self, robotPosVectors = None, saveSnapshot=False):
        """
        Generates a plot with currently loaded map status

        NOTE:
        if verbose, it displays the plot
        if saveSnapshot: saves a figure as mapstatus_currenttimestamp_FIGNUM.png
        """
        self.verbose=True
        #self.verbose=False

        # create a new figure and set it as current axis
        current_fig = plt.figure()
        self.current_ax = current_fig.add_subplot(111)

        self._drawGrid()

        # if flag is true, draw also current CostMatrix
        if self.verbose:
            self._drawCostMatrix()

        if robotPosVectors:
            for loc in robotPosVectors:
                print("Robot in pos: ", loc)
                self._drawRobot(loc_x_y_th=loc, robotPlotStyle='b--')
            # plot last robot position with solid green line
            self._drawRobot(loc_x_y_th=loc, robotPlotStyle='g-')

        #if saveSnapshot:
        #    ts = str(time.time())
        #    snapshot_name = "mapstatus_"+ts+"_F"+str(current_fig.number)+".png"
        #    print("saving %s " % snapshot_name)
        #    plt.savefig(snapshot_name)

        if self.verbose:
            current_fig.set_visible(True)
            current_fig.show()
            print("Press ENTER in the plot window to continue ... ")
            current_fig.waitforbuttonpress()
        else:
            current_fig.set_visible(False)

        return current_fig


    # UTILS
    # Insertar una coordenada para la propagacion en fronteras. Por si el bisect no existe
    #def insert(self, a_list, a_node: list):
    #    for i, e_node in enumerate(a_list):
    #        if a_node[0] < e_node[0] or (a_node[0] == e_node[0] and a_node[1] < e_node[1]):
    #            break
    #    return a_list[:i] + [a_node] + a_list[i:]

    def travel(self):
        # Si el path esta vacio, no puede devolver celda
        if not self.path:
            return self.index, [-1,-1], Vector2.zero
        # Obtenemos la celda correspondiente
        index       = self.index
        cell        = self.path[-index]
        self.index += 1
        return index, cell, Vector2(20+cell[1]*self.halfCell, 20+cell[0]*self.halfCell, 1)

    def getPath(self, index):
        cell = self.path[-index]
        return cell, Vector2((1+cell[1])*self.halfCell, (1+cell[0])*self.halfCell, 1)

    # Insertar un nodo para el A*
    def insert(self, a_list, a_node: dict):
        if not a_list:
            return [a_node]
        else: 
            a_value = a_node["coords"]
            # Avanzo mientras el coste del nuevo nodo sea mayor a los de la lista
            for i, e_node in enumerate(a_list):
                e_value = e_node["coords"]
                # Si el valor del coste del nuevo nodo es menor, terminamos
                if self.costMatrix[a_value[0]][a_value[1]] <= self.costMatrix[e_value[0]][e_value[1]]:
                    break
            # Devolvemos la lista con el nuevo elemento
            return a_list[:i] + [a_node] + a_list[i:]
        
    def __repr__(self) -> str:
        """
            Representacion de un mapa en pantalla
        """
        map_str  = "MAPA '" + self.name + "'\n"
        map_str += "- Dimensiones: " + str(self.sizeX) + " x " + str(self.sizeY) + " / " + str(self.sizeCell) + "cm\n"  
        map_str += "- Costes:\n" + str(self.costMatrix) + "\n"
        map_str += "- Camino encontrado:\n"
        map_str += "  " + str(self.path) + "\n"
        for i, row in reversed(list(enumerate(self.connectionMatrix))):
            for j, connection in enumerate(row):
                if not connection:
                    map_str += "■ "
                else:
                    if i%2==1 and j%2==1:
                        cell = [(i-1)//2,(j-1)//2]
                        if cell in self.path:
                            if cell == self.goal:
                                map_str += "⚑ "
                            elif cell == self.start:
                                map_str += "○ "
                            else:
                                map_str += "● "
                        else:
                            map_str += ". "
                    else:
                        map_str += "□ "
            map_str += "\n"

        return map_str