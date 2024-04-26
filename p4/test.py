# Pruebesita para odometría
from geometry import Vector2, Matrix2, Transform
from matplotlib import animation

import matplotlib.pyplot as plt
import numpy as np



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
