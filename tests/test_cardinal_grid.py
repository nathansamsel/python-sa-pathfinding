import pytest
import os

from sa_pathfinding.environments.grids.octile_grid import StateDoesNotExistError
from sa_pathfinding.environments.grids.cardinal_grid import CardinalGridAction
from sa_pathfinding.environments.grids.octile_grid import StateNotValidError
from sa_pathfinding.environments.grids.cardinal_grid import CardinalGrid
from sa_pathfinding.environments.grids.generics.grid import GridState

env = CardinalGrid(os.path.join(os.path.dirname(__file__))[:-5] + '/data/maps/small/den403d.map')

def test_grid_state_passable():
    passable_state = GridState(18, 10)
    assert env.is_valid(passable_state)
    assert not env.is_valid(GridState(18, 9))


def test_grid_state_undefined():
    undefined_state = GridState(-1, env.get_random().y)
    assert not env.is_defined(undefined_state)
    undefined_state = GridState(env.get_random().x, -1)
    assert not env.is_defined(undefined_state)
    undefined_state = GridState(env._width + 1, env.get_random().y)
    assert not env.is_defined(undefined_state)
    undefined_state = GridState(env.get_random().x, env._height + 1)
    assert not env.is_defined(undefined_state)


def test_grid_apply_action():
    undefined_state = GridState(-1, env.get_random().y)
    with pytest.raises(StateDoesNotExistError):
        env.apply_action(undefined_state, CardinalGridAction.UP)
    impassable_state = GridState(18, 9)
    with pytest.raises(StateNotValidError):
        env.apply_action(impassable_state, CardinalGridAction.UP)
    middle_state = GridState(18, 23, valid=True)
    assert env.apply_action(middle_state,
                            CardinalGridAction.UP) == GridState(18, 22)
    assert env.apply_action(middle_state,
                            CardinalGridAction.LEFT) == GridState(17, 23)
    assert env.apply_action(middle_state,
                            CardinalGridAction.DOWN) == GridState(18, 24)
    assert env.apply_action(middle_state,
                            CardinalGridAction.RIGHT) == GridState(19, 23)


def test_grid_get_actions():
    middle_state = GridState(18, 23, valid=True)
    action_cost_tuples = env.get_actions(middle_state, None)
    actions = [CardinalGridAction.UP, CardinalGridAction.DOWN, CardinalGridAction.LEFT, CardinalGridAction.RIGHT]
    for action, _ in action_cost_tuples:
        assert action in actions
