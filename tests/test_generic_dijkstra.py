import os
import pytest

from sa_pathfinding.environments.grids.octile_grid import StateDoesNotExistError
from sa_pathfinding.algorithms.dijkstra.generic_dijkstra import GenericDijkstra
from sa_pathfinding.algorithms.dijkstra.generic_dijkstra import GenericDijkstra
from sa_pathfinding.environments.grids.octile_grid import StateNotValidError
from sa_pathfinding.heuristics.grid_heuristic import OctileGridHeuristic
from sa_pathfinding.environments.grids.generics.grid import GridState
from sa_pathfinding.algorithms.generics.search_node import SearchNode
from sa_pathfinding.environments.grids.octile_grid import OctileGrid

env = OctileGrid(os.path.join(os.path.dirname(__file__))[:-5] + '/data/maps/small/den403d.map')

def test_gridoptimized_impassable_start():
    """Try to create search with start node that is not passable"""
    bad_start = env.get_random(valid=False)
    with pytest.raises(StateNotValidError):
        GenericDijkstra(env, start=SearchNode(bad_start))


def test_gridoptimized_impassable_goal():
    """Try to create search with goal node that is not passable"""
    bad_goal = env.get_random(valid=False)
    with pytest.raises(StateNotValidError):
        GenericDijkstra(env, goal=SearchNode(bad_goal))