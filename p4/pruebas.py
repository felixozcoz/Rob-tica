import numpy as np
from ReMapLib import Map
from MapLib import Map2D
import os


print("---------------------------------------------------")
rMap = Map("maps/mapa3.txt", [0,0], [0,7], neighborhood=8)
np.set_printoptions(precision=2, suppress=True)
rMap.drawMap(robotPosVectors=[[20,20,np.pi/2], [300, 20, -np.pi/4]], saveSnapshot=False)
print(rMap.path)