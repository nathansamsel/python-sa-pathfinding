import pytest
import time
import os

from sa_pathfinding.algorithms.astar.grid_optimized_astar import GridOptimizedAstar
from sa_pathfinding.environments.grids.octile_grid import StateDoesNotExistError
from sa_pathfinding.environments.grids.octile_grid import StateNotValidError
from sa_pathfinding.environments.grids.generics.grid_state import GridState
from sa_pathfinding.heuristics.grid_heuristic import OctileGridHeuristic
from sa_pathfinding.algorithms.astar.generic_astar import GenericAstar
from sa_pathfinding.algorithms.generics.search_node import SearchNode
from sa_pathfinding.environments.grids.octile_grid import OctileGrid


env = OctileGrid(os.path.join(os.path.dirname(os.path.dirname(__file__)) + '/data/maps/small/den403d.map'))

def get_random_search(environment=env, heuristic=OctileGridHeuristic()) -> GenericAstar:
    start = environment.get_random(valid=True)
    goal = environment.get_random(valid=True)
    gastar = GenericAstar(environment,
                                heuristic, 
                                start=SearchNode(start), 
                                goal=SearchNode(goal))
    return gastar


def test_impassable_goal():
    """Try to create search with goal node that is not passable"""
    bad_goal = env.get_random(valid=False)
    with pytest.raises(StateNotValidError):
        GenericAstar(env,
                    heuristic=OctileGridHeuristic(),
                    goal=SearchNode(bad_goal))


def test_undefined_start():
    """Test all 4 map boundaries <0, >width, <0, >height.
    if unchecked, negatives would index backwards into grid.env[][]
    """
    bad_start = GridState(-1, env.get_random().y)
    with pytest.raises(StateDoesNotExistError):
        GenericAstar(env,
                    heuristic=OctileGridHeuristic(),
                    start=SearchNode(bad_start))
    bad_start = GridState(env.get_random().x, -1)
    with pytest.raises(StateDoesNotExistError):
        GenericAstar(env,
                    heuristic=OctileGridHeuristic(),
                    start=SearchNode(bad_start))
    bad_start = GridState(env.width + 1, env.get_random().y)
    with pytest.raises(StateDoesNotExistError):
        GenericAstar(env,
                    heuristic=OctileGridHeuristic(),
                    start=SearchNode(bad_start))
    bad_start = GridState(env.get_random().x, env._height + 1)
    with pytest.raises(StateDoesNotExistError):
        GenericAstar(env,
                    heuristic=OctileGridHeuristic(),
                    start=SearchNode(bad_start))


def test_undefined_goal():
    """Test all 4 map boundaries <0, >width, <0, >height.
    if unchecked, negatives would index backwards into grid.env[][]
    """
    bad_goal = GridState(-1, env.get_random().y)
    with pytest.raises(StateDoesNotExistError):
        GenericAstar(env,
                    heuristic=OctileGridHeuristic(),
                    goal=SearchNode(bad_goal))
    bad_goal = GridState(env.get_random().x, -1)
    with pytest.raises(StateDoesNotExistError):
        GenericAstar(env,
                    heuristic=OctileGridHeuristic(),
                    goal=SearchNode(bad_goal))
    bad_goal = GridState(env.width + 1, env.get_random().y)
    with pytest.raises(StateDoesNotExistError):
        GenericAstar(env,
                    heuristic=OctileGridHeuristic(),
                    goal=SearchNode(bad_goal))
    bad_goal = GridState(env.get_random().x, env._height + 1)
    with pytest.raises(StateDoesNotExistError):
        GenericAstar(env,
                    heuristic=OctileGridHeuristic(),
                    goal=SearchNode(bad_goal))


def test_simple_randomized_search():
    """Test a simple reandomized search. Start and goal should be
    chosen to be a random, valid GridState because that is
    default behavior when they are not provided. As long as the
    state space is not disconnected, a path should be returned
    and it must include at least the start ang goal
    """
    astar = GenericAstar(env, heuristic=OctileGridHeuristic())
    path = astar.get_path()
    # simplest possible path must include start and goal
    # each should have been chosen for expansion from the open list
    assert astar.nodes_expanded >= 2
    assert len(path) >= 2


def test_simple_shortest_search():
    """Test a search where the start and goal are neighbors. Ensure
    correct path and open list sizes. Assumes start is in an
    'open area' where all neighbors are valid.
    """
    start = GridState(18, 24, valid=True)
    goal = GridState(19, 24, valid=True)
    astar = GenericAstar(env,
                        heuristic=OctileGridHeuristic(),
                        start=SearchNode(start),
                        goal=SearchNode(goal))
    astar.get_path()
    print(astar.open)
    assert len(astar.path) == 2  # start, goal
    assert astar.nodes_expanded == 2  # start, goal
    assert len(astar.open) == 7  # start + 8 neighbors - start - goal

def test_search_failure_invalid_goal():
    """Test scenario where someone bypasses the properties and messes with the private _start and _goal members. 
    Changing the start or making the start invalid or undefined should not impact the search at all.
    Making the goal invalid should let the search run until it has exhausted the state space and then 
    return no path.
    """
    gastar = get_random_search()
    gastar._goal = SearchNode(env.get_random(valid=False))
    assert len(gastar.get_path()) == 0

def test_search_failure_undefined_goal():
    """Test scenario where someone bypasses the properties and messes with the private _goal member. 
    Making the _start invalid or undefined after initialization should not impact the search at all.
    Making the goal invalid should let the search run until it has exhausted the state space and then 
    return no path.
    """
    gastar = get_random_search()
    gastar._goal = SearchNode(GridState(-1, -1))
    assert len(gastar.get_path()) == 0

def test_search_start_replaced():
    """Test scenario where someone bypasses the properties and messes with the private _start member. 
    Making the _start invalid or undefined after initialization should not impact the search at all
    because _start costs are initially set and _start is added to open list in the constructor,
    which validates the start ang goal to ensure they are both defined and valid. The only other place 
    astar touches the 'start' is when walking back through the parents from the goal to generate the path.
    Stopping condition was changed from 'while node != start' to 'while node.parent is not None' because
    the parent of start was set in the constuctor and presumably hasn't been changed and should be the only
    node with parent=None. This code change makes messing with _start after initialization pointless.
    """
    gastar = get_random_search()
    gastar._start = SearchNode(env.get_random(valid=False))
    assert len(gastar.get_path()) > 0

def test_history_on_success():
    """Test to make sure history dict is being filled in with all the correct info
    for a successful search. Checking keys are correct - not validating data.
    """
    gastar = get_random_search()
    gastar.get_path()
    assert 'start' in gastar.history
    assert 'goal' in gastar.history
    assert 'heuristic' in gastar.history
    assert 'path' in gastar.history
    assert 'nodes_expanded' in gastar.history
    assert 'steps' in gastar.history
    assert len(gastar.history['path']) > 0
    for i in range(gastar.history['nodes_expanded'] - 1):
        step = 'step-' + str(i+1)
        assert step in gastar.history['steps']
        assert 'expanded' in gastar.history['steps'][step]
        assert 'to_open' in gastar.history['steps'][step]

def test_history_on_failure():
    """Test to make sure history dict is being filled in with all the correct info
    for a failed search. Checking keys are correct - not validating data.
    """
    gastar = get_random_search()
    gastar._goal = SearchNode(GridState(-1, -1))
    gastar.get_path()
    assert 'start' in gastar.history
    assert 'goal' in gastar.history
    assert 'heuristic' in gastar.history
    assert 'nodes_expanded' in gastar.history
    assert 'steps' in gastar.history
    assert 'path' not in gastar.history
    for i in range(gastar.history['nodes_expanded'] - 1):
        step = 'step-' + str(i+1)
        assert step in gastar.history['steps']
        assert 'expanded' in gastar.history['steps'][step]
        assert 'to_open' in gastar.history['steps'][step]

def test_large_map():
    """Test on large sized map.
    """
    env = OctileGrid(os.path.join(os.path.dirname(os.path.dirname(__file__)) + '/data/maps/large/brc202d.map'))
    gastar = get_random_search(environment=env)
    assert len(gastar.get_path()) >= 1

def test_medium_map():
    """Test on medium sized map.
    """
    env = OctileGrid(os.path.join(os.path.dirname(os.path.dirname(__file__)) + '/data/maps/medium/lak302d.map'))
    gastar = get_random_search(environment=env)
    assert len(gastar.get_path()) >= 1

def test_small_map():
    """Test on small sized map.
    """
    env = OctileGrid(os.path.join(os.path.dirname(os.path.dirname(__file__)) + '/data/maps/small/den403d.map'))
    gastar = get_random_search(environment=env)
    assert len(gastar.get_path()) >= 1
