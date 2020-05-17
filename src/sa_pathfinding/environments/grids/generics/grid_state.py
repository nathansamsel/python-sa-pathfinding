from sa_pathfinding.environments.generics.state import State


class GridState(State):

    def __init__(self, x: int, y: int, valid: bool = False):
        self._x = x
        self._y = y
        self._valid = valid

    def __str__(self):
        return str(self._x) + ', ' + str(self._y)

    def __repr__(self):
        return '<' + self.__str__() + '>'

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return NotImplemented
        return self._x == other._x and self._y == other._y

    def __ne__(self, other):
        return not self.__eq__(other)

    def get_state(self) -> (int, int):
        return self._x, self._y, self._valid

    @property
    def x(self):
        return self._x

    @property
    def y(self):
        return self._y

    @property
    def valid(self):
        return self._valid
