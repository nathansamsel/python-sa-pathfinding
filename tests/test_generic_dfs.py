import pytest
import os

from sa_pathfinding.environments.grids.octile_grid import OctileGrid
from sa_pathfinding.environments.grids.generics.grid_state import GridState
from sa_pathfinding.algorithms.generics.search_node import SearchNode
from sa_pathfinding.algorithms.dfs.generic_dfs import GenericDFS

# env = OctileGrid(os.path.join(os.path.dirname(os.path.dirname(__file__)) + '/data/maps/small/tiny.map'))

# def test_simple_shortest_search():
#     """Test a search where the start and goal are neighbors. Ensure
#     correct path and open list sizes. Assumes start is in an
#     'open area' where all neighbors are valid.
#     """
#     start = env.get_random(valid=True)
#     goal = env.get_random(valid=True)
#     astar = GenericDFS(env,
#                         start=SearchNode(start),
#                         goal=SearchNode(goal))
#     astar.get_path()
#     assert len(astar.path) > 1  # start, goal