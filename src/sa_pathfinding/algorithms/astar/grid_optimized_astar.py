import heapq

from sa_pathfinding.algorithms.astar.generic_astar import GenericAstar
from sa_pathfinding.algorithms.generics.search_node import SearchNode
from sa_pathfinding.algorithms.generics.search_node import Status
from sa_pathfinding.environments.grids.generics.grid import Grid
from sa_pathfinding.heuristics.heuristic import Heuristic


class GridOptimizedAstar(GenericAstar):
    """ A class to represent the A* search algorithm optimized for grids.

    A class that represents the A* search algorithm. The status attribute
    was added to make running this algorithm on grids faster at the expense
    of generality.

    Note:

    Attributes:
        env (:obj:'Grid'): A class that represents the environment being
            being searched.
        start (:obj:'Node', optional): A class that represent the node to start
            the search from. The default is a random passable node from the
            provided 'env'.
        goal (:obj:'Node', optional): A class that represents the node to search
            to. The default is a random passable node from the provided 'env'.
        heuristic(:enum:'Heuristic', optional): An enum that represents the
            chosen heuristic to run the search with. The default is the octile
            distance heuristic. Pre-supported heuristics include: octile
            distance, manhattan distance, and euclidean distance.
        verbose(:obj:'bool', optional): A boolean flag that, when true, enables
            the printing of information about the search as it runs.
    """

    __slots__ = '_status'

    def __init__(self,
                 env: Grid,
                 heuristic: Heuristic,
                 start: SearchNode = None,
                 goal: SearchNode = None,
                 verbose: bool = False):
        # status needs to be initialized before super() call because
        # GenericAstar calls _add_to_open(start) at the end of its init
        # and that method is overwritten in this class as part of the
        # optimizations for grids. Don't want to add extra initialize() method
        # to call before running a search
        self._status = []
        for _ in range(env.height):
            self._status.append(
                [Status.UNDISCOVERED for _ in range(env.width)])
        super().__init__(env,
                         heuristic,
                         start=start,
                         goal=goal,
                         verbose=verbose)
    
    def __repr__(self) -> str:
        rep = repr(super())
        return rep

    def _add_to_open(self, node: SearchNode) -> None:
        heapq.heappush(self._open, node)
        self._status[node.state.y][node.state.x] = Status.ON_OPEN

    def _add_to_closed(self, node: SearchNode) -> None:
        self._status[node.state.y][node.state.x] = Status.ON_CLOSED

    def _is_status(self, node: SearchNode, status: Status) -> bool:
        return self._status[node.state.y][node.state.x] == status

    def _is_on_open(self, node: SearchNode) -> bool:
        return self._is_status(node, Status.ON_OPEN)

    def _is_on_closed(self, node: SearchNode):
        return self._is_status(node, Status.ON_CLOSED)
