import math

R2D = 180 / math.pi
D2R = math.pi / 180

# Vector 2d
class Vector2:
    # Constructor 
    def __init__(self, x=0.0, y=0.0, h=0.0, name="") -> None:
        self.name = name
        self.x = x
        self.y = y
        self.h = h
    # Vector absolutizado
    def abs(self) -> 'Vector2':
        return Vector2(abs(self.x), abs(self.y), self.h, self.name)
    # Angulo entre dos vectores
    def angle(self, v: 'Vector2', format = 'RAD'):
        res = math.acos((self * v) / (self.magnitude() * v.magnitude()))
        if (format == "DEG"):
            res *= R2D
        return res
    # Producto cartesiano de dos vectores
    def cross(self, v: 'Vector2'):
        return (self.x * v.y) - (self.y * v.x)
    # Magnitud de un vector
    def magnitude(self):
        return math.sqrt(self.x*self.x + self.y*self.y)
    # Vector normalizado
    def normalize(self, magnitude = 1.0):
        return self * (magnitude/self.magnitude())
    # Vector redondeado
    def round(self, n):
        return Vector2(round(self.x, n), round(self.y, n), self.h, self.name)
    # Suma de dos vectores
    def __add__(self, v: 'Vector2') -> 'Vector2':
        return Vector2(self.x + v.x, self.y + v.y)
    # Suma de un vector y asignacion
    def __iadd__(self, v: 'Vector2') -> None:
        self.x += v.x
        self.y += v.y
    # Resta de dos vectores
    def __sub__(self, v) -> 'Vector2':
        return Vector2(self.x - v.x, self.y - v.y)
    # Resta de un vector y asignacion
    def __isub__(self, v) -> 'Vector2':
        self.x -= v.x
        self.y -= v.y
    # Cambiar de signo un vector
    def __neg__(self) -> 'Vector2':
        return Vector2(-self.x, -self.y, self.h, self.name)
    # Multiplicacion escalar entre dos vectores
    def __mul__(self, v: 'Vector2'):
        return (self.x * v.x) + (self.y * v.y)
    # Multiplicacion de un escalar
    def __mul__(self, s) -> 'Vector2':
        return Vector2(self.x * s, self.y * s, self.h, self.name)
    # Multiplicacion de un escalar (orden inverso)
    def __rmul__(self, s) -> 'Vector2':
        return self * s
    # Multiplicacion de un escalar y asignacion
    def __imul__(self, s) -> None:
        self.x *= s
        self.y *= s
    # Division de un escalar
    def __div__(self, s) -> 'Vector2':
        return Vector2(self.x / s, self.y / s, self.h, self.name)
    # Division de un escalar y asignacion
    def __idiv__(self, s) -> None:
        self.x /= s
        self.y /= s
    # Igualdad de vectores
    def __eq__(self, v: 'Vector2') -> bool:
        return self.x == v.x and self.y == v.y
    # Desigualdad de vectores:
    def __ne__(self, v: 'Vector2') -> bool:
        return not (self == v)
    # Representacion de un vector en lista
    def __iter__(self) -> list:
        yield self.x
        yield self.y
        yield self.h
    # Representacion de un vector en pantalla
    def __repr__(self) -> str:
        return self.name + "(x=" + str(self.x) + ", y=" + str(self.y) + ", h=" + str(self.h) + ")" 

class Matrix2:
    # Constructor
    def __init__(self):
        self.a = "hola"

v = Vector2(1,1,1, "v")
w = Vector2(1,1,1, "w")
print(v + w)
print(v - w)
print(v * 2.0)
print(2 * v)
print(((v*2).normalize(2)).magnitude())