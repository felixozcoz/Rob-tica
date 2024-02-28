import math

class Vector2:
    def __init__(self, x=0.0, y=0.0):
        self.x = x
        self.y = y

    # adding two objects 
    def __add__(self, v):
        return Vector2(self.x + v.x, self.y + v.y)

    def __sub__(self, v):
        return Vector2(self.x - v.x, self.y - v.y)
    
    def __mul__(self, s: int):
        return Vector2(self.x * s, self.y * s)
    
    def normalize(self, magnitude):
        return self * (magnitude/self.magnitude)

    def magnitude(self):
        return math.sqrt(x*x + y*y)
def magnitude():



sense   = -1
center  = [2 * r, 0]
epsilon = 2  # margen de error en estimación de la odometría
n_epsilon = 0.5
min     = [center[0] - epsilon, center[1] - epsilon]
max     = [center[0] + epsilon, center[1] + epsilon]

 #time.sleep(robot.P)
 #setSpeed(0,90)


# ...

while True:
    robot.setSpeed(v, sense * w)
    x, y, _ = robot.readOdometry()
    rcvec   = [center[0] - x, center[1] - y]
    dist    = math.sqrt(rcvec[0]*rcvec[0] + rcvec[1]*rcvec[1])
    if dist < n_epsilon:
        sense *= -1
        time.sleep(0.2)


dir_sup = [x2 - x1, y2 - y1]
cos-1 [ (a · b) / (|a| |b|) ]


while True:
    x, y, _ = robot.readOdometry()
    dir_rob = [x2 - x, y2 - y]

    if (angle > -1 and angle < 1):
        robot.setSpeed(v1, 0)
