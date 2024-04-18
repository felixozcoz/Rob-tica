import numpy as np
from ReMapLib import Map
from geometry import Vector2


print("---------------------------------------------------")
rMap = Map("maps/mapa3.txt", [0,0], [0,7], neighborhood=8)
print(rMap.areConnected(Vector2(20,20,0), Vector2(20,60,0)))
print(rMap.areConnected(Vector2(20,20,0), Vector2(60,20,0)))
print(rMap.areConnected(Vector2(60,20,0), Vector2(60,60,0)))
rMap.deleteConnection([1,2])
rMap.drawMap()