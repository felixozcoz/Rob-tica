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

    def __mul__(self, v: 'Vector2'):
        """
            Multiplicacion escalar entre dos vectores
        """
        return (self.x * v.x) + (self.y * v.y)

    def __mul__(self, s) -> 'Vector2':
        """
            Multiplicacion de un escalar
        """
        return Vector2(self.x * s, self.y * s, self.h, self.name)

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