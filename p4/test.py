# Pruebesita para odometría
from geometry import Vector2, Matrix2, Transform
from matplotlib import animation

import matplotlib.pyplot as plt
import numpy as np

gx = 20
gy = 20
gth = 90

ltow = Matrix2.transform(Vector2(gx, gy, 0), gth)
# wtol_1 = Matrix2.transform(Vector2(-gx, -gy, 0), -gth)
# wtol_2 = Matrix2.transform(Vector2(-gx, gy, 0), -gth)
wtol_3 = ltow.invert()
print("gpos: ", [gx, gy, gth])
print(ltow*Vector2(40, -40, 1))
# print(wtol_1*Vector2(60, 60, 1))
# print(wtol_2*Vector2(60, 60, 1))
print(wtol_3*Vector2(60, 60, 1))