from abc import abstractmethod
from typing import Tuple
from typing import List
import random

from sa_pathfinding.environments.generics.env import StateDoesNotExistError
from sa_pathfinding.environments.generics.env import Environment
from sa_pathfinding.environments.generics.state import State
from sa_pathfinding.environments.generics.env import Action


class GridState(State):

    def __init__(self, x: int, y: int, valid: bool = False):
        self._x = x
        self._y = y
        self._valid = valid

    def __eq__(self, other) -> bool:
        if not isinstance(other, self.__class__):
            return NotImplemented
        return self._x == other._x and self._y == other._y

    def __ne__(self, other) -> bool:
        return not self.__eq__(other)
    
    def __str__(self) -> str:
        return str(self._x) + ', ' + str(self._y)

    def __repr__(self) -> str:
        return '<' + self.__str__() + '>'

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


class Grid(Environment):

    # TODO(Nathan): write a script to remove top meta info about the maps (file is only map data)
    # TODO(Nathan): add boolean 2d-array as option to create map from
    # TODO(Nathan): default should be 2d-array unless filename provided

    def __init__(self, filename: str) -> None:
        # open map file
        file = open(filename, "r")

        # grab map type, height, and weight info from file
        self._type: str = file.readline()[5:-1]
        self._height: int = int(file.readline()[7:])
        self._width: int = int(file.readline()[6:])

        # skip over useless line before map
        file.readline()

        # read map from file
        grid_map = []
        for y in range(0, self._height):
            grid_map.append(list(file.readline()[:-1]))

        # convert char map to 2D bool array (true = passable, false = obstacle)
        self._env: List[List[GridState]] = list()
        for y in range(0, self._height):
            self._env.append(list())
            for x in range(0, self._width):
                passable = True if grid_map[y][x] == '.' else False
                self._env[y].append(GridState(x, y, valid=passable))

    def __str__(self) -> str:
        return str(self._width) + 'x' + str(self._height)

    def is_defined(self, state: GridState) -> bool:
        if state.x > self._width - 1 or state.x < 0 or \
           state.y > self._height - 1 or state.y < 0:
            return False
        else:
            return True

    def is_valid(self, state: GridState) -> bool:
        if not self.is_defined(state):
            raise StateDoesNotExistError(state)
        return self._env[state.y][state.x].valid

    def print(self) -> None:
        line = ''
        for y in range(self._height):
            for x in range(self._width):
                line += '\u2591' if self._env[y][x].valid else '\u2588'
            line += '\n'
        return line

    def get_random(self, valid: bool = True) -> GridState:
        x = random.randint(0, self._width - 1)
        y = random.randint(0, self._height - 1)
        while not self._env[y][x].valid == valid:
            x = random.randint(0, self._width - 1)
            y = random.randint(0, self._height - 1)
        return GridState(x, y, valid=valid)

    @property
    def type(self):
        return self._type

    @property
    def height(self):
        return self._height

    @property
    def width(self):
        return self._width

    @property
    def env(self):
        return self._env

    @abstractmethod
    def apply_action(self,
                     state: GridState,
                     action: Action) -> GridState:
        pass

    @abstractmethod
    def get_actions(self,
                    state: GridState,
                    parent: GridState) -> List[Tuple[Action, float]]:
        pass
