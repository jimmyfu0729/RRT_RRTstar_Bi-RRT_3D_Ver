# Node class for the position in the grid
class Node(object):

    def __init__(self, x, y, z):
        self._x = x
        self._y = y
        self._z = z

    @property
    def x(self):
        return self._x

    @property
    def y(self):
        return self._y
    
    @property
    def z(self):
        return self._z

    def __eq__(self, other):
        return self._x == other.x and self._y == other.y and self._z == other.z

    def __ne__(self, other):
        return not self.__eq__(other)

    def __str__(self):
        return 'Node({}, {}, {})'.format(self._x, self._y, self._z)

    def __repr__(self):
        return str(self)

    def __hash__(self):
        return hash((self._x, self._y, self._z))
