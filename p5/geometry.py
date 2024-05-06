from numbers import Number
import matplotlib.pyplot as plt
import numpy as np
import random as rnd

np.set_printoptions(precision=2, suppress=True)
POSITION_ERROR = 1
ROTATION_ERROR = 1

class Vector2:
    # Constructor 
    def __init__(self, x=0.0, y=0.0, h=0.0, iter=None) -> None:
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
    
    def angle(self, v: 'Vector2', format="DEG"):
        """
            Angle between two vectors
        """
        dot   = self.normalize()*v.normalize()
        if dot > 1.0 or dot < -1.0:
            dot = round(dot)
        angle = np.arccos(dot)
        if format == "DEG":
            angle = np.rad2deg(angle)

        return angle

    def cross(self, v: 'Vector2'):
        """
            Cartesian product of two vectors (in 2d, represents the
            projection of the first vector onto the second one).
        """
        return (self.x * v.y) - (self.y * v.x)
    
    def sense(self, v: 'Vector2'):
        sense = self.cross(v)
        if sense == 0:
            return rnd.choice([-1,1]) * (round(self.angle(v))/180)
        else:
            return np.sign(sense)

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

    def __iadd__(self: 'Vector2', v: 'Vector2') -> 'Vector2':
        """
            Vector addition and assignation
        """
        self.x += v.x
        self.y += v.y
        self.h  = np.floor((self.h + v.h)/2)
        return self

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
        return self

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
        return self

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
        return self

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
    
    def __gt__(self, v: 'Vector2') -> bool:
        """
            Vector greater than (>)
        """
        return self.x > v.x or self.x == v.x and self.y > v.y
    
    def __ge__(self, v: 'Vector2') -> bool:
        """
            Vector greater than or equal (>=)
        """
        return self.x > v.x or self.x == v.x and self.y >= v.y
    
    def __lt__(self, v: 'Vector2') -> bool:
        """
            Vector less than (<)
        """
        return self.x < v.x or self.x == v.x and self.y < v.y
    
    def __le__(self, v: 'Vector2') -> bool:
        """
            Vector less than or equal (<=)
        """
        return self.x < v.x or self.x == v.x and self.y <= v.y

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
Vector2.error = Vector2(POSITION_ERROR, POSITION_ERROR)
#print(Vector2.error)
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
        return self

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
        return self

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
        return self

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
        return self

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
    
    def __gt__(self, v: 'Vector2') -> bool:
        """
            Vector greater than (>)
        """
        return self.x > v.x or (self.x == v.x and self.y > v.y) or (self.x == v.x and self.y == v.y and self.z > v.z)
    
    def __ge__(self, v: 'Vector2') -> bool:
        """
            Vector greater than or equal (>=)
        """
        return self.x > v.x or (self.x == v.x and self.y > v.y) or (self.x == v.x and self.y == v.y and self.z >= v.z) 
    
    def __lt__(self, v: 'Vector2') -> bool:
        """
            Vector less than (<)
        """
        return self.x < v.x or (self.x == v.x and self.y < v.y) or (self.x == v.x and self.y == v.y and self.z < v.z)
    
    def __le__(self, v: 'Vector2') -> bool:
        """
            Vector less than or equal (<=)
        """
        return self.x < v.x or (self.x == v.x and self.y <= v.y ) or (self.x == v.x and self.y == v.y and self.z <= v.z)
    
    def __iter__(self):
        """
            Representation of a vector in a list
        """
        yield self.x
        yield self.y
        yield self.z
        yield self.h

    def __repr__(self) -> str:
        """
            Representation of a vector in screen
        """
        return "(x=" + str(round(self.x, 7)) + ", y=" + str(round(self.y, 7)) + ", z=" + str(round(self.z, 7)) + ", h=" + str(round(self.h, 7)) + ")" 

def vector2(v: Vector3):
    return Vector2(v.x, v.y, v.h)

def vector3(v: Vector2):
    return Vector3(v.x, v.y, 0, v.h)

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

    def invert(self):
        """
            Invert a matrix
        """
        R = np.array(self.A)[0:2, 0:2]
        p = np.array(self.A)[0:2, 2]

        # Calculamos la transpuesta de R
        R_T = np.transpose(R)

        # Calculamos -R^T*p
        p_new = -np.dot(R_T, p)

        # Construimos la matriz de transformacion de B a A
        T_BA = np.eye(3)
        T_BA[0:2, 0:2] = R_T
        T_BA[0:2, 2] = p_new
        return Matrix2(T_BA)

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
    def __init__(self, position: Vector2 = Vector2.zero, rotation: float = None, forward: Vector2 = None, CUSTOM_POSITION_ERROR = None, CUSTOM_ROTATION_ERROR = None):
        """
            Transform class constructor
        """
        # POSITION
        self.POSITION = False
        self.position = position
        # . Errores
        self.POSITION_ERROR = POSITION_ERROR
        self.POSITION_SHIFT = Vector2.one.normalize(POSITION_ERROR)
        if CUSTOM_POSITION_ERROR is not None:
            self.POSITION_ERROR = CUSTOM_POSITION_ERROR
            self.POSITION_SHIFT = Vector2.one.normalize(CUSTOM_POSITION_ERROR)
        # . Verificacion de area
        self.position_inf = self.position - self.POSITION_SHIFT
        self.position_sup = self.position + self.POSITION_SHIFT
        # . Minimo local
        self.dmin = [-np.inf, -np.inf, -np.inf]
        self.dmin_counter = 0

        # ROTACION Y ORIENTACION
        self.ROTATION = False
        if rotation is None and forward is None:
            self.rotation = 0
            self.forward  = Vector2.right
        elif rotation is not None:
            self.rotation = (rotation + 180) % 360 - 180
            self.forward  = Vector2.right.rotate(rotation % 360)
        else:
            self.rotation = forward.sense(Vector2.right) * forward.angle(Vector2.right)
            self.forward  = forward
        # . Errores
        self.ROTATION_ERROR = ROTATION_ERROR
        if CUSTOM_ROTATION_ERROR is not None: # Si no le pongo is not none, comprueba si es 0
            self.ROTATION_ERROR = CUSTOM_ROTATION_ERROR
        # . Verificacion de area
        #self.rotation_inf = (self.rotation - self.ROTATION_ERROR + 180) % 360 - 180
        #self.rotation_sup = (self.rotation + self.ROTATION_ERROR + 180) % 360 - 180
        # . Minimo local (angulo entre dos rotaciones)
        self.rmin = [-np.inf, -np.inf, -np.inf]
        self.rmin_counter = 0
        # . Minimo local (angulo entre dos vectores)
        self.omin = [-np.inf, -np.inf, -np.inf]
        self.omin_counter = 0

    # Equivalencia
    def __eq__(self, transform):
        """
            Transform equality check within an error margin
        """
        # VERIFICAR POSICION
        # A. Area
        #self.POSITION |= (self.position_inf.x <= transform.position.x < self.position_sup.x) and (self.position_inf.y <= transform.position.y < self.position_sup.y)
        #if  transform.position != Vector2.zero and (self.position_inf.x <= transform.position.x < self.position_sup.x) and (self.position_inf.y <= transform.position.y < self.position_sup.y):
        #    print("Check por area:", self.position_inf, ">", transform.position, "<", self.position_sup)
        # B. Distancia (cm)
        distance = (self.position - transform.position).magnitude()
        #self.POSITION |= self.POSITION_ERROR > distance
        #if self.POSITION_ERROR > distance and transform.position != Vector2.zero:
        #    print("Check por distancia de posiciones:", self.POSITION_ERROR, ">", distance)
        # C. Minimo local en distancia (cm)
        # \ . / -> El valor anterior [1] y siguiente [2] son mayores que el actual [1].
        self.POSITION |= self.dmin[0] > self.dmin[1] < self.dmin[2] and 0.001 <= (self.dmin[0]-self.dmin[1]) and 0.001 <= (self.dmin[2]-self.dmin[1])
        print(self.dmin)
        if self.dmin[0] > self.dmin[1] < self.dmin[2] and 0.001 <= (self.dmin[0]-self.dmin[1]) and 0.001 <= (self.dmin[2]-self.dmin[1]):
            print("Check por minimo local de distancia:", self.dmin)
        if self.dmin_counter >= 250:
            self.dmin = self.dmin[1:] + [distance]
            self.dmin_counter  = 0
        else:
            self.dmin_counter += 1

        # VERIFICAR ROTACION
        # A. Area
        #self.ROTATION |= (transform.rotation - self.rotation_inf) % 360 <= (self.rotation_sup - self.rotation_inf) % 360
        #if (transform.rotation - self.rotation_inf) % 360 <= (self.rotation_sup - self.rotation_inf) % 360:
        #    print("Check por area:", self.rotation_inf, ">", transform.rotation, "<", self.rotation_sup)
        # B. Distancia (grados)
        distance = abs((self.rotation - transform.rotation + 180) % 360 - 180)
        self.ROTATION |= self.ROTATION_ERROR > distance
        #if (self.ROTATION_ERROR >= distance):
        #    print("Check por distancia de angulos:", self.ROTATION_ERROR, ">", distance, "(", self.rotation, ",", transform.rotation, ")")
        # C. Minimo local en distancia (grados)
        # \ . / -> El valor anterior [1] y siguiente [2] son mayores que el actual [1].
        #self.ROTATION |= self.rmin[0] > self.rmin[1] < self.rmin[2] and self.ROTATION_ERROR >= (self.rmin[0]-self.rmin[1]) and self.ROTATION_ERROR >= (self.rmin[2]-self.rmin[1])
        #if self.rmin[0] > self.rmin[1] < self.rmin[2] and self.ROTATION_ERROR >= (self.rmin[0]-self.rmin[1]) and self.ROTATION_ERROR >= (self.rmin[2]-self.rmin[1]):
        #    print("Check por minimo local de distancia de grados:", self.rmin)
        #if self.rmin_counter >= 250:
        #    self.rmin = self.rmin[1:] + [distance]
        #    self.rmin_counter  = 0
        #else:
        #    self.rmin_counter += 1
        # D. Orientacion    
        distance = abs((self.forward.angle(transform.forward) + 180) % 360 - 180)
        self.ROTATION |= self.ROTATION_ERROR >= distance
        # if (self.ROTATION_ERROR >= distance):
        #     print("Check por angulo de vectores:", self.ROTATION_ERROR, ">", distance, "(", self.forward, ",", transform.forward, ")")
        # E. Minimo local en orientacion (grados)
        # \ . / -> El valor anterior [1] y siguiente [2] son mayores que el actual [1].
        #self.ROTATION |= self.omin[0] > self.omin[1] < self.omin[2] and self.ROTATION_ERROR >= (self.omin[0]-self.omin[1]) and self.ROTATION_ERROR >= (self.omin[2]-self.omin[1])
        #if self.omin[0] > self.omin[1] < self.omin[2] and self.ROTATION_ERROR >= (self.omin[0]-self.omin[1]) and self.ROTATION_ERROR >= (self.omin[2]-self.omin[1]):
        #    print("Check por minimo local de distancia entre vectores:", self.omin)
        #if self.omin_counter >= 250:
        #    self.omin = self.omin[1:] + [distance]
        #    self.omin_counter  = 0
        #else:
        #    self.omin_counter += 1
        # Si cumple posicion y rotacion, ha llegado
        if self.POSITION and self.ROTATION:
            # Reset state
            self.POSITION = False
            self.ROTATION = False
            # Reset local minimum registries
            self.dmin = [-np.inf, -np.inf, -np.inf]
            self.dmin_counter = 0
            self.rmin = [-np.inf, -np.inf, -np.inf]
            self.rmin_counter = 0
            self.omin = [-np.inf, -np.inf, -np.inf]
            self.omin_counter = 0
            return True
        # Si no, no ha llegado
        return False

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

# def normalize(value, start, end):
#     """ Normaliza un valor entre los extremos dados """
#     width  = end - start
#     offset = value - start
#     return (offset - (np.floor(offset/width) * width)) + start

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
