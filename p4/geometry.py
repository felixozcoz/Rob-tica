from numbers import Number
import numpy as np
import matplotlib.pyplot as plt

POSITION_ERROR = 0.5
ROTATION_ERROR = 2

###########################################################
# VECTOR2
###########################################################
class Vector2:
    # Constructor 
    def __init__(self, x=0.0, y=0.0, h=1.0, iter=None) -> None:
        if iter is not None and len(iter) == 3:
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
        return Vector2(abs(self.x), abs(self.y), self.h)
    
    def angle(self, v: 'Vector2', format="RAD"):
        """
            Angle between two vectors
        """
        angle = np.arccos((self * v) / (self.magnitude() * v.magnitude()))
        if format == "DEG":
            angle = np.rad2deg(angle)

        return np.sign(self.cross(v)) * angle

    def cross(self, v: 'Vector2'):
        """
            Cartesian product of two vectors (in 2d, represents the
            projection of the first vector onto the second one).
        """
        return (self.x * v.y) - (self.y * v.x)

    def magnitude(self):
        """
            Vector module
        """
        return np.sqrt(self.x*self.x + self.y*self.y)

    def normalize(self, magnitude = 1.0):
        """
            Normalize a vector
        """
        return self * (magnitude/self.magnitude())
    
    def rotate(self, angle, format="DEG"):
        if format == "DEG":
            angle = np.deg2rad(angle)
        return Vector2(self.x*np.cos(angle) - self.y*np.sin(angle), self.x*np.sin(angle) + self.y*np.cos(angle), self.h)

    def round(self, n):
        """
            Rounded vector
        """
        return Vector2(round(self.x, n), round(self.y, n), self.h)

    def __add__(self, v: 'Vector2') -> 'Vector2':
        """
            Vector addition
        """
        return Vector2(self.x + v.x, self.y + v.y, np.floor((self.h + v.h)/2))

    def __iadd__(self, v: 'Vector2') -> None:
        """
            Vector addition and assignation
        """
        self.x += v.x
        self.y += v.y
        self.h  = np.floor((self.h + v.h)/2)

    def __sub__(self, v) -> 'Vector2':
        """
            Vector subtraction
        """
        return Vector2(self.x - v.x, self.y - v.y, abs(self.h - v.h))

    def __isub__(self, v) -> 'Vector2':
        """
            Vector subtraction and assignation
        """
        self.x -= v.x
        self.y -= v.y
        self.h  = abs(self.h - v.h)

    def __neg__(self) -> 'Vector2':
        """
            Vector sign inversion
        """
        return Vector2(-self.x, -self.y, self.h)

    def __mul__(self, other):
        """
            1. Scalar product
            2. Scalar product between two vectors
        """
        if isinstance(other, Number):
            return Vector2(self.x * other, self.y * other, self.h)
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
        return Vector2(self.x / s, self.y / s, self.h)

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
            Representation of a vector in a list
        """
        yield self.x
        yield self.y
        yield self.h
 
    def __repr__(self) -> str:
        """
            Representation of a vector in screen
        """
        return "(x=" + str(round(self.x, 7)) + ", y=" + str(round(self.y, 7)) + ", h=" + str(round(self.h, 7)) + ")" 


# Vector del eje -Y
Vector2.down = Vector2(0,-1)
# Vector error
Vector2.error = Vector2(POSITION_ERROR, POSITION_ERROR).normalize() 
# Vector del eje -X
Vector2.left = Vector2(-1,0)
# Vector unidad
Vector2.one = Vector2(1,1)
# Vector del eje +X
Vector2.right = Vector2(1,0)
# Vector del eje +Y
Vector2.up = Vector2(0,1)
# Vector nulo
Vector2.zero = Vector2(0,0)


###########################################################
# VECTOR3
###########################################################
class Vector3:
    # Constructor
    def __init__(self, x=0.0, y=0.0, z=0.0, h=0.0, iter=None):
        if iter is not None and len(iter) == 4:
            self.x = iter[0]
            self.y = iter[1]
            self.z = iter[2]
            self.h = iter[3]
        else:
            self.x = x
            self.y = y
            self.z = z
            self.h = h

    @staticmethod
    def zero(h = 0.0):
        return Vector3(0,0,0,h)
    
    @staticmethod
    def one(h = 0.0):
        return Vector3(1,1,1,h)
    
    @staticmethod
    def error(h = 0.0):
        return Vector3(POSITION_ERROR, POSITION_ERROR, POSITION_ERROR, h).normalize()
    
    def abs(self) -> 'Vector3':
        """
            Absolute value of a vector
        """  
        return Vector3(abs(self.x), abs(self.y), abs(self.z), self.h)

    def angle(self, v: 'Vector3', format="RAD"):
        """
            Angle between two vectors
        """
        angle = np.arccos((self * v)/(self.magnitude() * v.magnitude()))
        if format == "DEG":
            angle = np.rad2deg(angle)
        return angle

    def cross(self, v: 'Vector3'):
        """
            Cross product of two vectors (in 3d, it represents a vector
            perpendicular to the plane conformed by the first two vectors).
        """
        return Vector3(self.y*v.z - self.z*v.y, self.z*v.x - self.x*v.z, self.x*v.y - self.y*v.x, self.h)

    def magnitude(self):
        """
            Vector module
        """
        return np.sqrt(self.x*self.x + self.y*self.y + self.z*self.z)
    
    def normalize(self, magnitude=1.0):
        """
            Normalize a vector
        """
        return self * (magnitude/self.magnitude())
    
    def round(self, n):
        """
            Rounded vector
        """
        return Vector3(round(self.x, n), round(self.y, n), round(self.z, n), self.h)

    def __add__(self, v: 'Vector3') -> 'Vector3':
        """
            Vector addition
        """
        return Vector3(self.x + v.x, self.y + v.y, self.z + v.z, np.floor((self.h + v.h)/2))
    
    def __iadd__(self, v: 'Vector3') -> None:
        """
            Vector addition and assignation
        """
        self.x += v.x
        self.y += v.y
        self.z += v.z
        self.h  = np.floor((self.h + v.h)/2)

    def __sub__(self, v) -> 'Vector3':
        """
            Vector subtraction
        """
        return Vector3(self.x - v.x, self.y - v.y, self.z - v.z, abs(self.h - v.h))
    
    def __isub__(self, v) -> 'Vector3':
        """
            Vector subtraction and assignation
        """
        self.x -= v.x
        self.y -= v.y
        self.z -= v.z
        self.h  = abs(self.h - v.h)

    def __neg__(self) -> 'Vector3':
        """
            Vector sign inversion
        """
        return Vector3(-self.x, -self.y, -self.z, self.h)

    def __mul__(self, other):
        """
            1. Scalar product
            2. Scalar product between two vectors
        """
        if isinstance(other, Number):
            return Vector3(self.x * other, self.y * other, self.z * other, self.h)
        elif isinstance(other, Vector3):
            return (self.x * other.x) + (self.y * other.y) + (self.z * other.z)

    def __rmul__(self, s) -> 'Vector3':
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
        self.z *= s

    def __div__(self, s) -> 'Vector3':
        """
            Scalar division
        """
        return Vector3(self.x / s, self.y / s, self.z / s, self.h)

    def __idiv__(self, s) -> None:
        """
            Scalar division and assignation
        """
        self.x /= s
        self.y /= s
        self.z /= s

    def __eq__(self, v: 'Vector3') -> bool:
        """
            Vector equality
        """
        return self.x == v.x and self.y == v.y and self.z == v.z
    
    def __ne__(self, v: 'Vector3') -> bool:
        """
            Vector inequality
        """
        return not self == v
    
    def __iter__(self):
        """
            Representation of a vector in a list
        """
        yield self.x
        yield self.y
        yield self.z
        yield self.h

    def __vector2__(self) -> Vector2:
        return Vector2(self.x, self.y, self.h)

    def __repr__(self) -> str:
        """
            Representation of a vector in screen
        """
        return "(x=" + str(round(self.x, 7)) + ", y=" + str(round(self.y, 7)) + ", z=" + str(round(self.z, 7)) + ", h=" + str(round(self.h, 7)) + ")" 

def vector2(v: Vector3):
    return Vector2(v.x, v.y, v.h)

# Vector del eje -Z
Vector3.back = Vector3(0,0,-1) 
# Vector del eje -Y
Vector3.down = Vector3(0,-1,0)
# Vector error
Vector3.error = Vector3(POSITION_ERROR, POSITION_ERROR, POSITION_ERROR).normalize() 
# Vector del eje +Z
Vector3.forward = Vector3(0,0,1)
# Vector del eje -X
Vector3.left = Vector3(-1,0,0)
# Vector unidad
Vector3.one = Vector3(1,1,1)
# Vector del eje +X
Vector3.right = Vector3(1,0,0)
# Vector del eje +Y
Vector3.up = Vector3(0,1,0)
# Vector nulo
Vector3.zero = Vector3(0,0,0)

###########################################################
# MATRIX2
###########################################################
class Matrix2:
    # Constructor
    def __init__(self, A):
        self.A = [[],[],[]]
        self.T = [[],[],[]]
        for i in range(3):
            for j in range(3):
                self.A[i].append(A[i][j])
                self.T[i].append(A[j][i])

    @staticmethod
    def zero():
        return Matrix3([
            [0, 0, 0],
            [0, 0, 0],
            [0, 0, 0]
        ])

    @staticmethod
    def identity():
        return Matrix2([
            [1, 0, 0],
            [0, 1, 0],
            [0, 0, 1]
        ])

    @staticmethod
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
            return Matrix2(np.array(self.A)*other)
        elif isinstance(other, Vector2):
            return Vector2(iter=np.matmul(self.A, list(other)))
        elif isinstance(other, Matrix2):
            return Matrix2(np.matmul(self.A, other.A))

    
    def __repr__(self) -> str:
        """
            Representacion de una matriz en pantalla
        """
        row_str = []
        for i in range(3):
            row_str.append("\n  [ " + str(round(self.A[i][0], 7)) + "," + str(round(self.A[i][1], 7)) + "," + str(round(self.A[i][2], 7)) + " ]")
        return "(" + row_str[0] + row_str[1] + row_str[2] + "\n)" 


###########################################################
# MATRIX3
###########################################################
class Matrix3:
    # Constructor
    def __init__(self, A):
        self.A = [[],[],[],[]]
        self.T = [[],[],[],[]]
        for i in range(4):
            for j in range(4):
                self.A[i].append(A[i][j])
                self.T[i].append(A[j][i])

    @staticmethod
    def zero():
        return Matrix3([
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 0]
        ])

    @staticmethod
    def identity():
        return Matrix3([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
    
    @staticmethod
    def transform(translation: Vector3, rotation: float, rotation_axis="X-AXIS", format="DEG"):
        if format == "DEG":
            rotation = np.deg2rad(rotation)
        if rotation_axis == "X-AXIS":
            return Matrix3([
                [1,                0,                 0, translation.x],
                [0, np.cos(rotation), -np.sin(rotation), translation.y],
                [0, np.sin(rotation),  np.cos(rotation), translation.z],
                [0,                0,                 0,             1]
            ])
        elif rotation_axis == "Y-AXIS":
            return Matrix3([
                [ np.cos(rotation), 0, np.sin(rotation), translation.x],
                [                0, 1,                0, translation.y],
                [-np.sin(rotation), 0, np.cos(rotation), translation.z],
                [                0, 0,                0,             1]
            ])
        elif rotation_axis == "Z-AXIS":
            return Matrix3([
                [ np.cos(rotation), -np.sin(rotation), 0, translation.x],
                [ np.sin(rotation),  np.cos(rotation), 0, translation.y],
                [                0,                 0, 1, translation.z],
                [                0,                 0, 0,             1]
            ]) 

    def __mul__(self: "Matrix3", other): 
        if isinstance(other, Number):
            return Matrix2(np.array(self.A)*other)
        elif isinstance(other, Vector3):
            return Vector3(iter=np.matmul(self.A, list(other)))
        elif isinstance(other, Matrix3):
            return Matrix3(np.matmul(self.A, other.A))   

    def __repr__(self) -> str:
        """
            Representacion de una matriz en pantalla
        """
        row_str = []
        for i in range(4):
            row_str.append("\n  [ " + str(round(self.A[i][0], 7)) + "," + str(round(self.A[i][1], 7)) + "," + str(round(self.A[i][2], 7)) + "," + str(round(self.A[i][3], 7)) + " ]")
        return "(" + row_str[0] + row_str[1] + row_str[2] + row_str[3] + "\n)" 


###########################################################
# TRANSFORM
###########################################################
class Transform:
    # Constructor
    def __init__(self, position: Vector2, rotation: float =None, forward: Vector2 =None, CUSTOM_POSITION_ERROR =None, CUSTOM_ROTATION_ERROR =None):
        """
            Transform class constructor
        """
        # Position
        self.position = position
        self.POSITION_ERROR = POSITION_ERROR
        self.VECTOR_ERROR   = Vector2.error
        if CUSTOM_POSITION_ERROR is not None:
            self.POSITION_ERROR = CUSTOM_POSITION_ERROR
            #self.VECTOR_ERROR   = Vector2(CUSTOM_POSITION_ERROR, CUSTOM_POSITION_ERROR, 0).normalize()
        # Rotation and orientation
        if rotation is None and forward is None:
            self.rotation = 0
            self.forward  = Vector2.right
            self.right    = Vector2.up
        elif rotation is not None:
            self.rotation = rotation % 360
            self.forward  = Matrix2.transform(Vector2.zero, self.rotation) * Vector2.right
            self.right    = Matrix2.transform(Vector2.zero, 90) * self.forward
        else:
            self.rotation = forward.angle(Vector2.right)
            self.forward  = self.forward * Matrix2.transform(Vector2.zero, 90)
        self.ROTATION_ERROR = ROTATION_ERROR
        if CUSTOM_ROTATION_ERROR is not None:
            self.ROTATION_ERROR = CUSTOM_ROTATION_ERROR
        # Area error
        position_shift    = self.VECTOR_ERROR
        self.position_inf = position - position_shift
        self.position_sup = position + position_shift
        # Distance error
        self.lmin         = {
            "pass": False,
            "last": np.inf
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
    gamma = -np.arctan2(CV.y, CV.x)
    beta  =  np.arcsin((snd_radius-fst_radius)/axis_dist)
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
    angle = np.pi/2
    P1 = C1 + (fst_radius * Vector2(np.cos( (angle) - alpha), np.sin( (angle) - alpha)))
    P2 = C2 + (snd_radius * Vector2(np.cos( (angle) - alpha), np.sin( (angle) - alpha)))
    P3 = C2 + (snd_radius * Vector2(np.cos(-(angle) + alpha), np.sin(-(angle) + alpha)))
    P4 = C1 + (fst_radius * Vector2(np.cos(-(angle) + alpha), np.sin(-(angle) + alpha)))

    # Plot if needed
    if plot:
        angle = np.linspace(0, 2*np.pi, 100)
        # Plot the circumferences
        plt.plot(fst_radius * np.cos(angle) + C1.x, fst_radius * np.sin(angle) + C1.y, "b")
        plt.plot(snd_radius * np.cos(angle) + C2.x, snd_radius * np.sin(angle) + C2.y, "g")
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
