from scipy.interpolate import PchipInterpolator
import numpy as np
# # Pruebesita para odometría
# from geometry import Vector2, Matrix2, Transform
# from matplotlib import animation

# import matplotlib.pyplot as plt
# import numpy as np

# gx = 20
# gy = 20
# gth = 90

# ltow = Matrix2.transform(Vector2(gx, gy, 0), gth)
# # wtol_1 = Matrix2.transform(Vector2(-gx, -gy, 0), -gth)
# # wtol_2 = Matrix2.transform(Vector2(-gx, gy, 0), -gth)
# wtol_3 = ltow.invert()
# print("gpos: ", [gx, gy, gth])
# print(ltow*Vector2(40, -40, 1))
# # print(wtol_1*Vector2(60, 60, 1))
# # print(wtol_2*Vector2(60, 60, 1))
# print(wtol_3*Vector2(60, 60, 1))

segments = 3
points = [[0,0], [40,0], [80,40], [160,-40], [190,0], [200,0]]
x = [point[0] for point in points]
y = [point[1] for point in points]
trajectory = PchipInterpolator(x, y)

x_values = []
for i in range(len(x)-1):
    x_values = x_values + list(np.linspace(x[i], x[i+1], segments))
x_values = x_values + [x[-1]]
y_values   = trajectory(x_values)

print(x_values)
#print(list(y_values))