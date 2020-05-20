import pytest
import os

from sa_pathfinding.environments.grids.octile_grid import OctileGrid
from sa_pathfinding.environments.grids.generics.grid_state import GridState
from sa_pathfinding.algorithms.generics.search_node import SearchNode
from sa_pathfinding.algorithms.bfs.generic_bfs import GenericBFS

env = OctileGrid(os.path.join(os.path.dirname(os.path.dirname(__file__)) + '/data/maps/small/den403d.map'))

def test_simple_shortest_search():
    """Test a search where the start and goal are neighbors. Ensure
    correct path and open list sizes. Assumes start is in an
    'open area' where all neighbors are valid.
    """
    start = GridState(18, 24, valid=True)
    goal = GridState(19, 24, valid=True)
    astar = GenericBFS(env,
                        start=SearchNode(start),
                        goal=SearchNode(goal))
    astar.get_path()
    assert len(astar.path) == 2  # start, goal