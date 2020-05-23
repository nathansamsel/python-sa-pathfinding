import math

from sa_pathfinding.environments.grids.generics.grid import GridState
from sa_pathfinding.heuristics.heuristic import Heuristic


class GridHeuristic(Heuristic):

    def __init__(self):
        super().__init__()

    def __str__(self):
        return super().__str__() + ', env: Grid'

    def get_cost(self, start: GridState, goal: GridState):
        if not isinstance(start, GridState):
            raise NotImplementedError(f'node {start} missing attributes needed '
                                      f'to calculate heuristic cost')
        if not isinstance(goal, GridState):
            raise NotImplementedError(f'node {goal} missing attributes needed '
                                      f'to calculate heuristic cost')


class OctileGridHeuristic(GridHeuristic):

    def __init__(self):
        super().__init__()
        self._name = 'OCTILE'

    def __str__(self):
        return super().__str__()

    def get_cost(self, start: GridState, goal: GridState):
        super().get_cost(start, goal)
        return max(abs(start.x - goal.x), abs(start.y - goal.y)) + \
               (math.sqrt(2) - 1) * \
               min(abs(start.x - goal.x), abs(start.y - goal.y))


class ManhattanGridHeuristic(GridHeuristic):

    def __init__(self):
        super().__init__()
        self._name = 'MANHATTAN'

    def __str__(self):
        return super().__str__()

    def get_cost(self, start: GridState, goal: GridState):
        super().get_cost(start, goal)
        return float(abs(start.x - goal.x) + abs(start.y - goal.y))


class EuclideanGridHeuristic(GridHeuristic):

    def __init__(self):
        super().__init__()
        self._name = 'EUCLIDEAN'

    def __str__(self):
        return super().__str__()

    def get_cost(self, start: GridState, goal: GridState):
        super().get_cost(start, goal)
        return math.sqrt((start.x - goal.x) ** 2 + (start.y - goal.y) ** 2)
