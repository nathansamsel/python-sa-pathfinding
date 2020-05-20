from sa_pathfinding.algorithms.astar.generic_astar import GenericAstar
from sa_pathfinding.algorithms.generics.search_node import SearchNode
from sa_pathfinding.environments.generics.env import Environment
from sa_pathfinding.heuristics.heuristic import ZeroHeuristic


class GenericDijkstra(GenericAstar):

    def __init__(self,
                 env: Environment,
                 start: SearchNode = None,
                 goal: SearchNode = None,
                 verbose: bool = False):
        super().__init__(env,
                         heuristic=ZeroHeuristic(),
                         start=start, goal=goal,
                         verbose=verbose)
