from typing import Tuple
from typing import List
import math

from sa_pathfinding.environments.generics.env import StateDoesNotExistError
from sa_pathfinding.environments.generics.env import StateNotValidError
from sa_pathfinding.environments.grids.generics.grid import GridState
from sa_pathfinding.environments.grids.generics.grid import Grid
from sa_pathfinding.environments.generics.env import Action


class OctileGridAction(Action):
    UP = 0
    UP_RIGHT = 1
    RIGHT = 2
    DOWN_RIGHT = 3
    DOWN = 4
    DOWN_LEFT = 5
    LEFT = 6
    UP_LEFT = 7


class OctileGrid(Grid):

    def __init__(self, filename: str) -> None:
        super().__init__(filename)

    def __repr__(self):
        repr(super())

    def apply_action(self,
                     state: GridState,
                     action: OctileGridAction) -> GridState:
        if not self.is_valid(state):
            raise StateNotValidError(state)
        x = 0
        y = 0
        if action == OctileGridAction.UP:
            y -= 1
        elif action == OctileGridAction.RIGHT:
            x += 1
        elif action == OctileGridAction.DOWN:
            y += 1
        elif action == OctileGridAction.LEFT:
            x -= 1
        elif action == OctileGridAction.UP_RIGHT:
            y -= 1; x += 1
        elif action == OctileGridAction.DOWN_RIGHT:
            y += 1; x += 1
        elif action == OctileGridAction.DOWN_LEFT:
            y += 1; x -= 1
        elif action == OctileGridAction.UP_LEFT:
            y -= 1; x -= 1
        else:
            raise NotImplementedError(f"GridAction {action} not supported.")
        test_state = GridState(state.x + x, state.y + y)
        valid = self.is_valid(test_state)
        if not valid:
            raise StateNotValidError(test_state)
        new_state = GridState(state.x + x, state.y + y, valid=valid)
        if not self.is_defined(state):
            raise StateDoesNotExistError
        return new_state

    def get_actions(self,
                    state: GridState,
                    parent: GridState) -> List[Tuple[OctileGridAction, float]]:
        if not self.is_defined(state):
            raise StateDoesNotExistError(state)
        if not state.valid:
            raise StateNotValidError(state)

        actions = list()

        # down-right action check
        if state.x < self._width - 1 and \
                state.y < self._height - 1 and \
                self._env[state.y + 1][state.x + 1].valid and \
                self._env[state.y][state.x + 1].valid and \
                self._env[state.y + 1][state.x].valid and \
                (parent is None or (parent.y != state.y + 1 or
                                    parent.x != state.x + 1)):
            actions.append((OctileGridAction.DOWN_RIGHT, math.sqrt(2)))

        # down-left action check
        if state.y < self._height - 1 and \
                state.x > 0 and \
                self._env[state.y + 1][state.x - 1].valid and \
                self._env[state.y][state.x - 1].valid and \
                self._env[state.y + 1][state.x].valid and \
                (parent is None or (parent.y != state.y + 1 or
                                    parent.x != state.x - 1)):
            actions.append((OctileGridAction.DOWN_LEFT, math.sqrt(2)))

        # up-right action check
        if state.x < self._width - 1 and \
                state.y > 0 and \
                self._env[state.y - 1][state.x + 1].valid and \
                self._env[state.y][state.x + 1].valid and \
                self._env[state.y - 1][state.x].valid and \
                (parent is None or (parent.y != state.y - 1 or
                                    parent.x != state.x + 1)):
            actions.append((OctileGridAction.UP_RIGHT, math.sqrt(2)))

        # up-left action check
        if state.x > 0 and \
                state.y > 0 and \
                self._env[state.y - 1][state.x - 1].valid and \
                self._env[state.y][state.x - 1].valid and \
                self._env[state.y - 1][state.x].valid and \
                (parent is None or (parent.y != state.y - 1 or
                                    parent.x != state.x - 1)):
            actions.append((OctileGridAction.UP_LEFT, math.sqrt(2)))

        # up action check
        if state.y > 0 and \
                self._env[state.y - 1][state.x].valid and \
                (parent is None or (parent.y != state.y - 1 or
                                    parent.x != state.x)):
            actions.append((OctileGridAction.UP, 1))

        # right action check
        if state.x < self._width - 1 and \
                self._env[state.y][state.x + 1].valid and \
                (parent is None or (parent.y != state.y or
                                    parent.x != state.x + 1)):
            actions.append((OctileGridAction.RIGHT, 1))

        # down action check
        if state.y < self._height - 1 and \
                self._env[state.y + 1][state.x].valid and \
                (parent is None or (parent.y != state.y + 1 or
                                    parent.x != state.x)):
            actions.append((OctileGridAction.DOWN, 1))

        # left action check
        if state.x > 0 and \
                self._env[state.y][state.x - 1].valid and \
                (parent is None or (parent.y != state.y or
                                    parent.x != state.x - 1)):
            actions.append((OctileGridAction.LEFT, 1))

        return actions
