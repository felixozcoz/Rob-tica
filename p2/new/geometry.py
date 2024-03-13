from numbers import Number
import math

R2D = 180 / math.pi
D2R = math.pi / 180
POSITION_ERROR = 0.5
ROTATION_ERROR = 2

# Vector 2d
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
            Vector absolutizado
        """
        return Vector2(abs(self.x), abs(self.y), self.h, self.name)
    
    def angle(self, v: 'Vector2', format = 'RAD'):
        """
            Angulo entre dos vectores
        """
        res = math.acos((self * v) / (self.magnitude() * v.magnitude()))
        if (format == "DEG"):
            res *= R2D
        return res

    def cross(self, v: 'Vector2'):
        """
            Producto cartesiano de dos vectores (en 2d, es el 
            area del paralelogramo que definen).
        """
        return (self.x * v.y) - (self.y * v.x)

    def magnitude(self):
        """
            Magnitud de un vector
        """
        return math.sqrt(self.x*self.x + self.y*self.y)

    def normalize(self, magnitude = 1.0):
        """
            Vector normalizado
        """
        return self * (magnitude/self.magnitude())

    def round(self, n):
        """
            Vector con valores redondeados
        """
        return Vector2(round(self.x, n), round(self.y, n), self.h, self.name)

    def __add__(self, v: 'Vector2') -> 'Vector2':
        """
            Suma de dos vectores
        """
        return Vector2(self.x + v.x, self.y + v.y)

    def __iadd__(self, v: 'Vector2') -> None:
        """
            Suma de un vector y asignacion
        """
        self.x += v.x
        self.y += v.y

    def __sub__(self, v) -> 'Vector2':
        """
            Resta de dos vectores
        """
        return Vector2(self.x - v.x, self.y - v.y)

    def __isub__(self, v) -> 'Vector2':
        """
            Resta de un vector y asignacion
        """
        self.x -= v.x
        self.y -= v.y

    def __neg__(self) -> 'Vector2':
        """
            Cambiar de signo un vector
        """
        return Vector2(-self.x, -self.y, self.h, self.name)

    def __mul__(self, other):
        """
            1. Multiplicacion de un escalar
            2. Multiplicacion escalar entre dos vectores
        """
        if isinstance(other, Number):
            return Vector2(self.x * other, self.y * other, self.h, self.name)
        elif isinstance(other, Vector2):
            return (self.x * other.x) + (self.y * other.y)

    def __rmul__(self, s) -> 'Vector2':
        """
            Multiplicacion de un escalar (orden inverso)
        """
        return self * s

    def __imul__(self, s) -> None:
        """
            Multiplicacion de un escalar y asignacion
        """
        self.x *= s
        self.y *= s

    def __div__(self, s) -> 'Vector2':
        """
            Division de un escalar
        """
        return Vector2(self.x / s, self.y / s, self.h, self.name)

    def __idiv__(self, s) -> None:
        """
            Division de un escalar y asignacion
        """
        self.x /= s
        self.y /= s

    def __eq__(self, v: 'Vector2') -> bool:
        """
            Igualdad de vectores
        """
        return self.x == v.x and self.y == v.y

    def __ne__(self, v: 'Vector2') -> bool:
        """
            Desigualdad de vectores
        """
        return not (self == v)

    def __iter__(self):
        """
            Representacion de un vector en lista
        """
        yield self.x
        yield self.y
        yield self.h
 
    def __repr__(self) -> str:
        """
            Representacion de un vector en pantalla
        """
        return self.name + "(x=" + str(round(self.x, 7)) + ", y=" + str(round(self.y, 7)) + ", h=" + str(round(self.h, 7)) + ")" 
    
Vector2.zero  = Vector2(0,0,0)
Vector2.one   = Vector2(1,1,1)
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

    def transform(translate: Vector2, rotate: float, format="DEG"):
        if format == "DEG":
            rotate = rotate * D2R
        return Matrix2([
            [math.cos(rotate), -math.sin(rotate), translate.x],
            [math.sin(rotate),  math.cos(rotate), translate.y],
            [               0,                 0,           1]
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
            Representacion de un vector en pantalla
        """
        row_str = []
        for i in range(3):
            row_str.append("\n  [ " + str(round(self.A[i][0], 7)) + "," + str(round(self.A[i][1], 7)) + "," + str(round(self.A[i][2], 7)) + " ]")
        return "(" + row_str[0] + row_str[1] + row_str[2] + "\n)" 


a = Matrix2.identity() * 3
b = Matrix2.identity() * 2
print(a)
print(b)
print(a*b)

v = Vector2(1,0)
print(Matrix2.transform(Vector2.zero,  90) * v)
print(Matrix2.transform(Vector2.zero, -90) * v)
