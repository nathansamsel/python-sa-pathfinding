import heapq

from sa_pathfinding.algorithms.astar.generic_astar import GenericAstar
from sa_pathfinding.algorithms.generics.search_node import SearchNode
from sa_pathfinding.algorithms.generics.search_node import Status
from sa_pathfinding.environments.grids.generics.grid import Grid
from sa_pathfinding.environments.generics.state import State
from sa_pathfinding.heuristics.heuristic import Heuristic

"""grid_optimized_astar Module

This module contains an implementation for the A* algorithm, optimized for grids.

The below example has not been implemented yet. coming soon...

Example:
    The module can be run from the command line to execute an A* search.
    The command line args are the same as the parameters for the
    GenericAstar object. The result is a string representation of the
    path that was found - a comma separated list of state __repr__() calls::

        $ python grid_optimized_astar.py OctileGrid('path/to/file')
        $ python grid_optimized_astar.py OctileGrid('path/to/file') start=GridState(43, 65)
        $ python grid_optimized_astar.py OctileGrid('path/to/file') goal=GridState(43, 65)
        $ python grid_optimized_astar.py OctileGrid('path/to/file') start=GridState(43, 65) goal=GridState(132, 94)
        $ python grid_optimized_astar.py OctileGrid('path/to/file') start=GridState(43, 65) goal=GridState(132, 94) heuristic=OctileGridHeuristic

Todo:
    * implement logging solutions for debug printing/to file and history tracking to console / to file
    * implement the module level command line interface
    * redo GenericAstar and Search __repr__
"""


class GridOptimizedAstar(GenericAstar):
    """ This class implements the A* search algorithm, optimized for grids.

    GridOptimizedAstar inherits from GenericAstar. GenericAstar implements
    the core A* algorithm, but GridOptimizedAstar adds a status data 
    structure to help speed up open and closed list checks. The status structure
    is a 2D array of Status enums that overlays the grid such that the
    grid position indexes into the status structure. As long as the status structure
    is maintained as the open and closed lists are manipulated, we get constant time
    membership checks for the open and closed lists. This is at the cost of some additional
    memory to store potentially invalid states. GridOptimizedAstar overrides the _is_on_open()
    and _is_on_closed() with a check against the status data structure. To maintain the status
    as the search progresses, the _add_to_open() and _add_to_closed methods are overriden
    to set the appropriate status.

    Note: This implementation eliminates the closed list. While closed still exists as a
            member, it is not used by the algorithm. Since the status structure keeps
            a reference to which grid states are on closed, theres no need to keep the list
            of SearchNodes around with their costs wasting space. If these details need
            to be reviewed, its better to use the history dict or logging.

    All attributes are read-only properties.

    Attributes:
        closed (:obj:'list' of :obj:'State'): The closed list for the A* search algorithm.
        env (:obj:'Environment'): A class that represents the environment being
            being searched.
        goal (:obj:'Node'): A class that represents the node to search
            to. The default is a random passable node from the provided 'env'.
        heuristic (:obj:'Heuristic'): A class that represents the
            chosen heuristic to run the search with. The default is the octile
            distance heuristic. Pre-supported heuristics include: octile
            distance, manhattan distance, and euclidean distance.
        history (:obj:'dict'): A dictionary of documentary info on the
            execution of the search.
        nodes_expanded (:obj:'int'): Number of nodes expanded in the search.
        open (:obj:'list' of :obj:'State'): The open list for the A* search algorithm.
        path (:obj:'list' of :obj:'State'): The path returned by the execution of the search. It
            is empty by default and is empty if the search fails.
        start (:obj:'Node'): A class that represent the node to start
            the search from. The default is a random passable node from the
            provided 'env'.
        success (:obj:'bool'): A boolean flag set at the end of search execution,
            where true indicates search success and false indicates failure
        verbose (:obj:'bool'): A boolean flag that, when true, enables 
            the printing of information about the search as it runs. 
    """

    __slots__ = '_status'

    def __init__(self,
                 env: Grid,
                 heuristic: Heuristic,
                 start: State = None,
                 goal: State = None,
                 verbose: bool = False):
        """GridOptimizedAstar __init__ method.

        Attributes env, start, goal, nodes_expanded, path, success, verbose
        are instantiated in parent's parent class Search __init__.

        Attributes heuristic, history, open and closed are
        instantiated in parent's class GenericAstar __init__.

        When start and goal are left as None, the default behavior is
        to pick a random, valid state using env.get_random().  This
        occurs in Search __init__.

        The start node is assigned initial costs and added to open in
        the GenericAstar __init__.

        Note:
            _status needs to be initialized before the super() call
            because, at the end of GenericAstar's __init__, the
            start is added to the open list and this class overrides
            the _add_to_open() method with a reference to _status.

        Args:
            env (:obj:'Environment'): Environment being being searched.
            start (:obj:`State`, optional): State to start search from.
            goal (:obj:`State`): State to search to.
            verbose (:obj:'bool'): Flag for verbose printing.
        """
        self._status = []
        for _ in range(env.height):
            self._status.append([Status.UNDISCOVERED for _ in range(env.width)])
        super().__init__(env,
                         heuristic,
                         start=start,
                         goal=goal,
                         verbose=verbose)
    
    def __repr__(self) -> str:
        return repr(super())

    def _add_to_open(self, node: SearchNode) -> None:
        super()._add_to_open(node)
        self._status[node.state.y][node.state.x] = Status.ON_OPEN

    def _add_to_closed(self, node: SearchNode) -> None:
        self._status[node.state.y][node.state.x] = Status.ON_CLOSED

    def _is_status(self, node: SearchNode, status: Status) -> bool:
        return self._status[node.state.y][node.state.x] == status

    def _is_on_open(self, node: SearchNode) -> bool:
        return self._is_status(node, Status.ON_OPEN)

    def _is_on_closed(self, node: SearchNode):
        return self._is_status(node, Status.ON_CLOSED)
