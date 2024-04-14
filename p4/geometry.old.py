from numbers import Number
import math
import numpy as np
import matplotlib.pyplot as plt

D2R = math.pi / 180
POSITION_ERROR = 0.5
ROTATION_ERROR = 2

# Vector2
class Vector2:
    # Constructor 
    def __init__(self, x=0.0, y=0.0, h=1.0, name="", iter=None) -> None:
        self.name = name
        if iter and isinstance(iter, list) and len(iter) == 3:
            self.x = iter[0]
            self.y = iter[1]
            self.h = iter[2]
        else:
            self.x = x
            self.y = y
            self.h = h

    def abs(self) -> 'Vector2':
        """
            Absolute value of a vector
        """
        return Vector2(abs(self.x), abs(self.y), self.h, self.name)
    
    def angle(self, v: 'Vector2', format = 'RAD'):
        """
            Angle between two vectors
        """
        angle = math.acos((self * v) / (self.magnitude() * v.magnitude()))
        if (format == "DEG"):
            angle = np.rad2deg(angle)
        return angle

    def cross(self, v: 'Vector2'):
        """
            Cartesian product of two vectors (in 2d, it is the
            area of the parallelogram they define).
        """
        return (self.x * v.y) - (self.y * v.x)

    def magnitude(self):
        """
            Vector module
        """
        return math.sqrt(self.x*self.x + self.y*self.y)

    def normalize(self, magnitude = 1.0):
        """
            Normalize a vector
        """
        return self * (magnitude/self.magnitude())

    def round(self, n):
        """
            Rounded vector
        """
        return Vector2(round(self.x, n), round(self.y, n), self.h, self.name)

    def __add__(self, v: 'Vector2') -> 'Vector2':
        """
            Vector addition
        """
        return Vector2(self.x + v.x, self.y + v.y)

    def __iadd__(self, v: 'Vector2') -> None:
        """
            Vector addition and assignation
        """
        self.x += v.x
        self.y += v.y

    def __sub__(self, v) -> 'Vector2':
        """
            Vector subtraction
        """
        return Vector2(self.x - v.x, self.y - v.y)

    def __isub__(self, v) -> 'Vector2':
        """
            Vector subtraction and assignation
        """
        self.x -= v.x
        self.y -= v.y

    def __neg__(self) -> 'Vector2':
        """
            Vector sign inversion
        """
        return Vector2(-self.x, -self.y, self.h, self.name)

    def __mul__(self, other):
        """
            1. Scalar product
            2. Scalar product between two vectors
        """
        if isinstance(other, Number):
            return Vector2(self.x * other, self.y * other, self.h, self.name)
        elif isinstance(other, Vector2):
            return (self.x * other.x) + (self.y * other.y)

    def __rmul__(self, s) -> 'Vector2':
        """
            Scalar product (inverse order)
        """
        return self * s

    def __imul__(self, s) -> None:
        """
            Scalar product and assignation
        """
        self.x *= s
        self.y *= s

    def __div__(self, s) -> 'Vector2':
        """
            Scalar division
        """
        return Vector2(self.x / s, self.y / s, self.h, self.name)

    def __idiv__(self, s) -> None:
        """
            Scalar division and assignation
        """
        self.x /= s
        self.y /= s

    def __eq__(self, v: 'Vector2') -> bool:
        """
            Vector equality
        """
        return self.x == v.x and self.y == v.y

    def __ne__(self, v: 'Vector2') -> bool:
        """
            Vector inequality
        """
        return not (self == v)

    def __iter__(self):
        """
            Representtation of a vector in a list
        """
        yield self.x
        yield self.y
        yield self.h
 
    def __repr__(self) -> str:
        """
            Representtation of a vector in screen
        """
        return self.name + "(x=" + str(round(self.x, 7)) + ", y=" + str(round(self.y, 7)) + ", h=" + str(round(self.h, 7)) + ")" 
    
Vector2.zero  = Vector2(0,0,0)
Vector2.one   = Vector2(1,1,1)
Vector2.up    = Vector2(1,0,0)
Vector2.right = Vector2(0,1,0)
Vector2.error = Vector2(POSITION_ERROR, POSITION_ERROR, POSITION_ERROR).normalize()

# Matrix2
class Matrix2:
    # Constructor
    def __init__(self, A):
        self.A = [[],[],[]]
        self.T = [[],[],[]]
        for i in range(3):
            for j in range(3):
                self.A[i].append(A[i][j])
                self.T[i].append(A[j][i])

    def identity():
        return Matrix2([
            [1,0,0],
            [0,1,0],
            [0,0,1]
        ])

    def transform(translation: Vector2, rotation: float, format="DEG"):
        if format == "DEG":
            rotation = np.deg2rad(rotation)
        return Matrix2([
            [np.cos(rotation), -np.sin(rotation), translation.x],
            [np.sin(rotation),  np.cos(rotation), translation.y],
            [               0,                 0,             1]
        ])
    
    def __mul__(self: "Matrix2", other): 
        if isinstance(other, Number):
            return Matrix2([
                [x * other for x in self.A[0]],
                [x * other for x in self.A[1]],
                [x * other for x in self.A[2]]
            ])
        elif isinstance(other, Vector2):
            v = list(other)
            return Vector2(
                sum(x * y for x, y in zip(self.A[0], v)),
                sum(x * y for x, y in zip(self.A[1], v)),
                sum(x * y for x, y in zip(self.A[2], v)),
                other.name
            )
        elif isinstance(other, Matrix2):
            A = []
            for i in range(3):
                A.append([])
                for j in range(3):
                    A[i].append(sum(x * y for x, y in zip(self.A[i], other.T[j])))
            return Matrix2(A)
    
    def __repr__(self) -> str:
        """
            Representacion de una matriz en pantalla
        """
        row_str = []
        for i in range(3):
            row_str.append("\n  [ " + str(round(self.A[i][0], 7)) + "," + str(round(self.A[i][1], 7)) + "," + str(round(self.A[i][2], 7)) + " ]")
        return "(" + row_str[0] + row_str[1] + row_str[2] + "\n)" 

# Transform
class Transform:
    # Constructor
    def __init__(self, localPosition, position, localRotation, rotation: float = None, forward: Vector2 = None, CUSTOM_POSITION_ERROR =None, CUSTOM_ROTATION_ERROR =None):
        """
            Transform class constructor
        """
        # Position
        self.position       = position
        self.localPosition  = localPosition
        self.POSITION_ERROR = POSITION_ERROR
        if not CUSTOM_POSITION_ERROR is None:
            self.POSITION_ERROR = CUSTOM_POSITION_ERROR
        # Rotation and orientation
        if rotation is None and forward is None:
            self.rotation = 0
            self.forward  = Vector2(1,0)
            self.right    = Vector2(0,1)
        elif not rotation is None:
            self.rotation = rotation % 360
            self.forward  = Matrix2.transform(Vector2.zero, self.rotation) * Vector2(1,0)
            self.right    = Matrix2.transform(Vector2.zero, 90) * self.forward
        else:
            self.rotation = forward.angle(Vector2(1, 0))
            self.forward  = self.forward * Matrix2.transform(Vector2.zero, 90)
        self.ROTATION_ERROR = ROTATION_ERROR
        if not CUSTOM_ROTATION_ERROR is None:
            self.ROTATION_ERROR = CUSTOM_ROTATION_ERROR
        # Area error
        position_shift = Vector2.error # Vector2(self.POSITION_ERROR, self.POSITION_ERROR).normalize
        self.position_inf = position - position_shift
        self.position_sup = position + position_shift
        # Distance error
        self.lmin         = {
            "pass": False,
            "last": math.inf
        }
        # Orientation error
        #self.rotation_inf = rotation - ROTATION_ERROR
        #self.rotation_sup = rotation + ROTATION_ERROR

    

    # Equivalencia
    def __eq__(self, transform):
        """
            Transform equality check within an error margin
        """
        # POSITION CHECK
        # 1. Area check
        POSITION = (self.position_inf.x <= transform.position.x and transform.position.x < self.position_sup.x) and \
            (self.position_inf.y <= transform.position.y and transform.position.y < self.position_sup.y)
        # 2. Distance check
        dist = (self.position - transform.position).magnitude()
        POSITION |= self.POSITION_ERROR > dist
        # 3. Local minimum check
        POSITION |= (self.lmin["last"] <  dist) and not self.lmin["pass"]
        self.distance = {
            "pass": (self.lmin["last"] >= dist),
            "last": dist
        }
        # ROTATION CHECK
        ROTATION = self.ROTATION_ERROR > abs(self.rotation - transform.rotation)
        #ROTATION = ROTATION_ERROR > forward.angle(transform.forward)

        # BOTH CHECK
        return POSITION and ROTATION

    # ToString
    def __repr__(self) -> str:
        """
            Representacion de un transform en pantalla
        """
        return "Transform { pos: " + str(self.position) + ", rot: " + str(round(self.rotation, 7)) + ", fwr: " + str(self.forward) + " }"

# Funciones extra
def circunferences_secant_points(fst_radius, snd_radius, axis_dist, plot=False):
    """
        Returns the tangent points of the outer tangent lines of two circumferences
        - rad1: radius of the first circumference
        - rad2: radius of the second circumference
        - distAxes: distance between the centers of the circumferences
    """

    # Coordinates of the centers of the circumferences
    C1 = Vector2(fst_radius, 0)
    C2 = Vector2(fst_radius + axis_dist, 0)
    CV = C2 - C1

    # Angle between the centers of the circumferences
    gamma = -math.atan2(CV.y, CV.x)
    beta  =  math.asin((snd_radius-fst_radius)/axis_dist)
    alpha =  gamma - beta

    # Calculate the tangent points 
    #     (1)                (2)
    #     = x = ----------- = x =
    #  =        =        =        =
    # =          =      =          =
    # =          =      =          =
    #  =        =        =        =
    #     = x = ----------- = x =
    #     (4)                (3)
    P1 = C1 + (fst_radius * Vector2(math.cos( (math.pi/2) - alpha), math.sin( (math.pi/2) - alpha)))
    P2 = C2 + (snd_radius * Vector2(math.cos( (math.pi/2) - alpha), math.sin( (math.pi/2) - alpha)))
    P3 = C2 + (snd_radius * Vector2(math.cos(-(math.pi/2) + alpha), math.sin(-(math.pi/2) + alpha)))
    P4 = C1 + (fst_radius * Vector2(math.cos(-(math.pi/2) + alpha), math.sin(-(math.pi/2) + alpha)))

    # Plot if needed
    if plot:
        ang = np.linspace(0, 2*np.pi, 100)
        # Plot the circumferences
        plt.plot(fst_radius * np.cos(ang) + C1.x, fst_radius * np.sin(ang) + C1.y, "b")
        plt.plot(snd_radius * np.cos(ang) + C2.x, snd_radius * np.sin(ang) + C2.y, "g")
        # Plot the circumferences centers
        plt.plot(C1.x, C1.y, "bo")
        plt.plot(C2.x, C2.y, "go")
        # Plot the tangent points
        plt.plot(P1.x, P1.y, "ro")
        plt.plot(P2.x, P2.y, "ro")
        plt.plot(P3.x, P3.y, "ro")
        plt.plot(P4.x, P4.y, "ro")
        # Plot the tangent lines
        plt.plot([C1.x, P1.x], [C1.y, P1.y], 'y')
        plt.plot([C2.x, P2.x], [C2.y, P2.y], 'y')
        plt.plot([P1.x, P2.x], [P1.y, P2.y], 'y')
        plt.plot([P1.x, P3.x], [P4.y, P3.y], 'y')
        plt.plot([C1.x, P4.x], [C1.y, P4.y], 'y')
        plt.plot([C2.x, P3.x], [C2.y, P3.y], 'y')

        plt.axis("equal")
        plt.show()

    # Returning the points
    return P1, P2, P3, P4

#a = Matrix2.identity() * 3
#b = Matrix2.identity() * 2
#print(a)
#print(b)
#print(a*b)
#
#v = Vector2(1,0)
#print(Matrix2.transform(Vector2.zero,  90) * v)
#print(Matrix2.transform(Vector2.zero, -90) * v)
#circunferences_secant_points(5, 10, 25, True)
