from abc import abstractmethod
from abc import ABC

from sa_pathfinding.environments.generics.state import State


class Heuristic(ABC):

    __slots__ = '_name'

    def __init__(self):
        self._name = 'Uninitialized'

    def __str__(self):
        return f'Heuristic, type: {self._name}'

    @property
    def name(self):
        return self._name

    @abstractmethod
    def get_cost(self, node: State, goal: State):
        pass


class ZeroHeuristic(Heuristic):

    def __init__(self):
        super().__init__()
        self._name = 'ZERO'

    def __str__(self):
        return super().__str__()

    def get_cost(self, start: State = None, goal: State = None):
        return 0.0
