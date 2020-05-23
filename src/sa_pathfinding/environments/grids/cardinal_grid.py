from typing import Tuple
from typing import List


from sa_pathfinding.environments.generics.env import StateDoesNotExistError
from sa_pathfinding.environments.generics.env import StateNotValidError
from sa_pathfinding.environments.grids.generics.grid import GridState
from sa_pathfinding.environments.grids.generics.grid import Grid
from sa_pathfinding.environments.generics.env import Action


class CardinalGridAction(Action):
    UP = 0
    RIGHT = 1
    DOWN = 2
    LEFT = 3


class CardinalGrid(Grid):

    def __init__(self, filename: str) -> None:
        super().__init__(filename)

    def __repr__(self):
        repr(super())

    def apply_action(self,
                     state: GridState,
                     action: CardinalGridAction) -> GridState:
        if not self.is_defined(state):
            raise StateDoesNotExistError(state)
        if not self.is_valid(state):
            raise StateNotValidError(state)
        x = 0
        y = 0
        if action == CardinalGridAction.UP:
            y -= 1
        elif action == CardinalGridAction.RIGHT:
            x += 1
        elif action == CardinalGridAction.DOWN:
            y += 1
        elif action == CardinalGridAction.LEFT:
            x -= 1
        else:
            raise NotImplementedError(f"GridAction {action} not supported.")
        test_state = GridState(state.x + x, state.y + y)
        valid = self.is_valid(test_state)
        if not valid:
            raise StateNotValidError(test_state)
        new_state = GridState(state.x + x, state.y + y, valid=valid)
        if not self.is_defined(new_state):
            raise StateDoesNotExistError
        return new_state

    def get_actions(self,
                    state: GridState,
                    parent: GridState) -> List[Tuple[CardinalGridAction, float]]:
        if not state.valid:
            raise StateNotValidError(state)
        if not self.is_defined(state):
            raise StateDoesNotExistError(state)
        actions = list()

        # up action check
        if state.y > 0 and \
                self._env[state.y - 1][state.x].valid and \
                (parent is None or (parent.y != state.y - 1 or
                                    parent.x != state.x)):
            actions.append((CardinalGridAction.UP, 1))

        # right action check
        if state.x < self._width - 1 and \
                self._env[state.y][state.x + 1].valid and \
                (parent is None or (parent.y != state.y or
                                    parent.x != state.x + 1)):
            actions.append((CardinalGridAction.RIGHT, 1))

        # down action check
        if state.y < self._height - 1 and \
                self._env[state.y + 1][state.x].valid and \
                (parent is None or (parent.y != state.y + 1 or
                                    parent.x != state.x)):
            actions.append((CardinalGridAction.DOWN, 1))

        # left action check
        if state.x > 0 and \
                self._env[state.y][state.x - 1].valid and \
                (parent is None or (parent.y != state.y or
                                    parent.x != state.x - 1)):
            actions.append((CardinalGridAction.LEFT, 1))

        return actions
