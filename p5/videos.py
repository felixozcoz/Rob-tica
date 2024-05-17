from Robot import Robot
from ReMapLib import Map
from geometry import Vector2

print("Inicializando el robot y sus componentes")
robot = Robot()

points = [[0,0],[0.7,0.3],[1,1],[0.7,1.7],[0,2],[-0.7,2.3],[-1,3],[-0.7,3.7],[0,4],[0.7,3.7],[1,3],[0.7,2.3],[0,2],[-0.7,1.7],[-1,1],[-0.7,0.3],[0,0]]
points = [[0,0],[0.7,0.3],[1,1],[1.5,3],[2,5],[1,6.75],[0,7],[-1,6.75],[-2,5],[-1.5,3],[-1,1],[-0.7,0.3],[0,0]]
points = [[0,0],[0.7,-0.3],[1,-1],[0.7,-1.7],[0.35,-2],[2.35,-4],[2,-4.5], [1.5,-4.5],[-1.5,-2],[0,0]]
robot.playTrajectory(points, 30, showPlot=True)
