import os
import pytest

from sa_pathfinding.algorithms.astar.grid_optimized_astar import GridOptimizedAstar
from sa_pathfinding.environments.grids.octile_grid import StateDoesNotExistError
from sa_pathfinding.environments.grids.octile_grid import StateNotValidError
from sa_pathfinding.environments.grids.generics.grid_state import GridState
from sa_pathfinding.heuristics.grid_heuristic import OctileGridHeuristic
from sa_pathfinding.algorithms.generics.search_node import SearchNode
from sa_pathfinding.environments.grids.octile_grid import OctileGrid

env = OctileGrid(os.path.join(os.path.dirname(__file__))[:-5] + '/data/maps/small/den403d.map')


def test_impassable_start():
    """Try to create search with start node that is not passable"""
    bad_start = env.get_random(valid=False)
    with pytest.raises(StateNotValidError):
        GridOptimizedAstar(env,
                           heuristic=OctileGridHeuristic(),
                           start=SearchNode(bad_start))


def test_impassable_goal():
    """Try to create search with goal node that is not passable"""
    bad_goal = env.get_random(valid=False)
    with pytest.raises(StateNotValidError):
        GridOptimizedAstar(env,
                           heuristic=OctileGridHeuristic(),
                           goal=SearchNode(bad_goal))


def test_undefined_start():
    """Test all 4 map boundaries <0, >width, <0, >height.
    if unchecked, negatives would index backwards into grid.env[][]
    """
    bad_start = GridState(-1, env.get_random().y)
    with pytest.raises(StateDoesNotExistError):
        GridOptimizedAstar(env,
                           heuristic=OctileGridHeuristic(),
                           start=SearchNode(bad_start))
    bad_start = GridState(env.get_random().x, -1)
    with pytest.raises(StateDoesNotExistError):
        GridOptimizedAstar(env,
                           heuristic=OctileGridHeuristic(),
                           start=SearchNode(bad_start))
    bad_start = GridState(env.width + 1, env.get_random().y)
    with pytest.raises(StateDoesNotExistError):
        GridOptimizedAstar(env,
                           heuristic=OctileGridHeuristic(),
                           start=SearchNode(bad_start))
    bad_start = GridState(env.get_random().x, env._height + 1)
    with pytest.raises(StateDoesNotExistError):
        GridOptimizedAstar(env,
                           heuristic=OctileGridHeuristic(),
                           start=SearchNode(bad_start))


def test_undefined_goal():
    bad_goal = GridState(-1, env.get_random().y)
    with pytest.raises(StateDoesNotExistError):
        GridOptimizedAstar(env,
                           heuristic=OctileGridHeuristic(),
                           goal=SearchNode(bad_goal))
    bad_goal = GridState(env.get_random().x, -1)
    with pytest.raises(StateDoesNotExistError):
        GridOptimizedAstar(env,
                           heuristic=OctileGridHeuristic(),
                           goal=SearchNode(bad_goal))
    bad_goal = GridState(env.width + 1, env.get_random().y)
    with pytest.raises(StateDoesNotExistError):
        GridOptimizedAstar(env,
                           heuristic=OctileGridHeuristic(),
                           goal=SearchNode(bad_goal))
    bad_goal = GridState(env.get_random().x, env._height + 1)
    with pytest.raises(StateDoesNotExistError):
        GridOptimizedAstar(env,
                           heuristic=OctileGridHeuristic(),
                           goal=SearchNode(bad_goal))


def test_simple_randomized_search():
    astar = GridOptimizedAstar(env, heuristic=OctileGridHeuristic())
    path = astar.get_path()
    # simplest possible path must include start and goal
    # each should have been chosen for expansion from the open list
    assert astar.nodes_expanded >= 2
    assert len(path) >= 2


def test_simple_shortest_search():
    start = GridState(18, 24, valid=True)
    goal = GridState(19, 24, valid=True)
    astar = GridOptimizedAstar(env,
                               heuristic=OctileGridHeuristic(),
                               start=SearchNode(start),
                               goal=SearchNode(goal))
    astar.get_path()
    print(astar.open)
    assert len(astar.path) == 2  # start, goal
    assert astar.nodes_expanded == 2  # start, goal
    assert len(astar.open) == 7  # start + 8 neighbors - start - goal

def test_search_failure_invalid_goal():
    """
    Above tests show that search validation won't allow the instantiation of a search
    without a both defined and valid start and goal. So, a search should never really
    fail if its a complete search. But, someone could bypass the properties and
    mess with the private _start and _goal members. Chaning the start or making the start invalid or undefined should not
    impact the search at all. The _start member achieves its use in the constructor.
     Making the goal invalid or undefined should let the search run until it has exhausted 
     the state space and then return no path. This seems like appropriate
    behaviour for something that is being messed with.
    """
    start = env.get_random(valid=True)
    goal = env.get_random(valid=True)
    gastar = GridOptimizedAstar(env,
                                OctileGridHeuristic(), 
                                start=SearchNode(start), 
                                goal=SearchNode(goal))
    gastar._goal = SearchNode(env.get_random(valid=False))
    assert len(gastar.get_path()) == 0

def test_search_failure_undefined_goal():
    start = env.get_random(valid=True)
    goal = env.get_random(valid=True)
    gastar = GridOptimizedAstar(env,
                                OctileGridHeuristic(), 
                                start=SearchNode(start), 
                                goal=SearchNode(goal))
    gastar._goal = SearchNode(GridState(-1, -1))
    assert len(gastar.get_path()) == 0


def test_history_on_success():
    start = env.get_random(valid=True)
    goal = env.get_random(valid=True)
    gastar = GridOptimizedAstar(env,
                                OctileGridHeuristic(), 
                                start=SearchNode(start), 
                                goal=SearchNode(goal))
    gastar.get_path()
    assert 'start' in gastar.history
    assert 'goal' in gastar.history
    assert 'heuristic' in gastar.history
    assert 'path' in gastar.history
    assert 'step-1' in gastar.history

def test_history_on_failure():
    start = env.get_random(valid=True)
    goal = env.get_random(valid=True)
    gastar = GridOptimizedAstar(env,
                                OctileGridHeuristic(), 
                                start=SearchNode(start), 
                                goal=SearchNode(goal))
    gastar._goal = SearchNode(GridState(-1, -1))
    gastar.get_path()
    assert 'path' not in gastar.history

def test_large_map():
    env = OctileGrid(os.path.join(os.path.dirname(__file__))[:-5] + '/data/maps/large/brc202d.map')

    start = env.get_random(valid=True)
    goal = env.get_random(valid=True)
    gastar = GridOptimizedAstar(env,
                            OctileGridHeuristic(), 
                            start=SearchNode(start), 
                            goal=SearchNode(goal))
    assert len(gastar.get_path()) >= 1
