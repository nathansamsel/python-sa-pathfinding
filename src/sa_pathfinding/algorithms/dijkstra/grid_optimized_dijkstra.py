from sa_pathfinding.algorithms.astar.grid_optimized_astar import GridOptimizedAstar
from sa_pathfinding.algorithms.generics.search_node import SearchNode
from sa_pathfinding.environments.grids.generics.grid import Grid
from sa_pathfinding.heuristics.heuristic import ZeroHeuristic


class GridOptimizedDijkstra(GridOptimizedAstar):

    def __init__(self,
                 env: Grid,
                 start: SearchNode = None,
                 goal: SearchNode = None,
                 verbose: bool = False):
        super().__init__(env,
                         heuristic=ZeroHeuristic(),
                         start=start,
                         goal=goal,
                         verbose=verbose)
