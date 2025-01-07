import math


class Vector:
    """
    Represents a 2D vector with basic mathematical operations.
    """

    def __init__(self, x=0, y=0):
        self._x = x
        self._y = y

    @property
    def x(self):
        return self._x

    @property
    def y(self):
        return self._y

    def length(self):
        return math.sqrt(self._x**2 + self._y**2)

    def normalize(self):
        length = self.length()
        if length == 0:
            raise ValueError("Cannot normalize a zero vector.")
        return Vector(self._x / length, self._y / length)

    def angle(self):
        return math.degrees(math.atan2(self._y, self._x))

    def rotate(self, angle):
        radians = math.radians(angle)
        cos_theta = math.cos(radians)
        sin_theta = math.sin(radians)
        new_x = self._x * cos_theta - self._y * sin_theta
        new_y = self._x * sin_theta + self._y * cos_theta
        return Vector(new_x, new_y)

    def distance_to(self, other: 'Vector') -> float:
        """
        Calculates the Euclidean distance to another vector.
        """
        dx = self.x - other.x
        dy = self.y - other.y
        return math.hypot(dx, dy)

    @staticmethod
    def cross(v1, v2):
        return v1._x * v2._y - v1._y * v2._x

    @staticmethod
    def from_angle(angle):
        radians = math.radians(angle)
        return Vector(math.cos(radians), math.sin(radians))

    @staticmethod
    def from_polar(radius, angle):
        radians = math.radians(angle)
        return Vector(radius * math.cos(radians), radius * math.sin(radians))

    def to_tuple(self):
        return self._x, self._y

    @staticmethod
    def from_tuple(tup):
        return Vector(tup[0], tup[1])

    @staticmethod
    def dot(v1, v2):
        return v1._x * v2._x + v1._y * v2._y

    def __add__(self, other):
        return Vector(self._x + other._x, self._y + other._y)

    def __sub__(self, other):
        return Vector(self._x - other._x, self._y - other._y)

    def __mul__(self, scalar):
        return Vector(self._x * scalar, self._y * scalar)

    def __truediv__(self, scalar):
        if scalar == 0:
            raise ZeroDivisionError("Division by zero.")
        return Vector(self._x / scalar, self._y / scalar)

    def __eq__(self, other):
        epsilon = 1e-6
        return math.isclose(self._x, other._x, abs_tol=epsilon) and math.isclose(self._y, other._y, abs_tol=epsilon)

    def __ne__(self, other):
        return not self == other

    def __abs__(self):
        return self.length()

    def __str__(self):
        return f"Vector(X: {self._x}, Y: {self._y})"

    def __repr__(self):
        return self.__str__()
