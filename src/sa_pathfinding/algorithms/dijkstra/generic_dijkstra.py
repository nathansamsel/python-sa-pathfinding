from sa_pathfinding.algorithms.astar.generic_astar import GenericAstar
from sa_pathfinding.environments.generics.env import Environment
from sa_pathfinding.heuristics.heuristic import ZeroHeuristic
from sa_pathfinding.environments.generics.state import State

"""generic_dijkstra Module

This module contains the implementation for the generic Dijkstra search algorithm.

The below example has not been implemented yet. coming soon...

Example:
    The module can be run from the command line to execute a Dijkstra search.
    The command line args are the same as the parameters for the
    GenericDijkstra object. The result is a string representation of the
    path that was found - a comma separated list of state __repr__() calls::

        $ python generic_dijkstra.py OctileGrid('path/to/file')
        $ python generic_dijkstra.py OctileGrid('path/to/file') start=GridState(43, 65)
        $ python generic_dijkstra.py OctileGrid('path/to/file') goal=GridState(43, 65)
        $ python generic_dijkstra.py OctileGrid('path/to/file') start=GridState(43, 65) goal=GridState(132, 94)
        $ python generic_dijkstra.py OctileGrid('path/to/file') start=GridState(43, 65) goal=GridState(132, 94) heuristic=OctileGridHeuristic

Todo:
    * implement logging solutions for debug printing/to file and history tracking to console / to file
    * implement the module level command line interface
"""


class GenericDijkstra(GenericAstar):
    """ This class implements the Dijkstra search algorithm.

    GenericDijkstra inherits from GenericAstar. See GenericAstar for
    implementation details.

    Given that Dijkstra and A* are nearly the same - aside from the heuristic, we can
    run an A* search with a heuristic that always returns 0. This adds the overhead of 
    having the heuristic to deal with, but makes the code nice and DRY.

    All attributes are read-only properties.

    Attributes:
        closed (:obj:'list' of :obj:'State'): The closed list for the Dijkstra search algorithm.
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
        open (:obj:'list' of :obj:'State'): The open list for the Dijkstra search algorithm.
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
    def __init__(self,
                 env: Environment,
                 start: State = None,
                 goal: State = None,
                 verbose: bool = False):
        """GenericDijkstra __init__ method.

        Attributes env, start, goal, nodes_expanded, path, success, verbose
        are instantiated in parent's parent's parent class Search __init__.

        Args:
            env (:obj:'Environment'): Environment being being searched.
            start (:obj:`State`, optional): State to start search from.
            goal (:obj:`State`): State to search to.
            verbose (:obj:'bool'): Flag for verbose printing.
        """
        super().__init__(env,
                         heuristic=ZeroHeuristic(),
                         start=start, goal=goal,
                         verbose=verbose)
