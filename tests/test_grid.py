import pytest
import os

from sa_pathfinding.environments.grids.octile_grid import StateDoesNotExistError
from sa_pathfinding.environments.grids.octile_grid import StateNotValidError
from sa_pathfinding.environments.grids.generics.grid_state import GridState
from sa_pathfinding.environments.grids.octile_grid import OctileGridAction
from sa_pathfinding.environments.grids.octile_grid import OctileGrid

env = OctileGrid(os.path.join(os.path.dirname(__file__))[:-5] + '/data/maps/small/den403d.map')


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
        env.apply_action(undefined_state, OctileGridAction.UP)
    impassable_state = GridState(18, 9)
    with pytest.raises(StateNotValidError):
        env.apply_action(impassable_state, OctileGridAction.UP)
    middle_state = GridState(18, 23, valid=True)
    assert env.apply_action(middle_state,
                            OctileGridAction.UP) == GridState(18, 22)
    assert env.apply_action(middle_state,
                            OctileGridAction.UP_LEFT) == GridState(17, 22)
    assert env.apply_action(middle_state,
                            OctileGridAction.LEFT) == GridState(17, 23)
    assert env.apply_action(middle_state,
                            OctileGridAction.DOWN_LEFT) == GridState(17, 24)
    assert env.apply_action(middle_state,
                            OctileGridAction.DOWN) == GridState(18, 24)
    assert env.apply_action(middle_state,
                            OctileGridAction.DOWN_RIGHT) == GridState(19, 24)
    assert env.apply_action(middle_state,
                            OctileGridAction.RIGHT) == GridState(19, 23)
    assert env.apply_action(middle_state,
                            OctileGridAction.UP_RIGHT) == GridState(19, 22)


def test_grid_get_actions():
    pass
