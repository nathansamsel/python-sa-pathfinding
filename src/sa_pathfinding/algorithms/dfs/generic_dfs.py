import queue

from sa_pathfinding.algorithms.bfs.generic_bfs import GenericBFS
from sa_pathfinding.environments.generics.env import Environment
from sa_pathfinding.environments.generics.state import State

"""generic_dfs Module

This module contains the implementation for the Depth-First Search (DFS) algorithm.

The below example has not been implemented yet. coming soon...

Example:
    The module can be run from the command line to execute a DFS search.
    The command line args are the same as the parameters for the
    GenericDFS object. The result is a string representation of the
    path that was found - a comma separated list of state __repr__() calls::

        $ python generic_dfs.py OctileGrid('path/to/file')
        $ python generic_dfs.py OctileGrid('path/to/file') start=GridState(43, 65)
        $ python generic_dfs.py OctileGrid('path/to/file') goal=GridState(43, 65)
        $ python generic_dfs.py OctileGrid('path/to/file') start=GridState(43, 65) goal=GridState(132, 94)
        $ python generic_dfs.py OctileGrid('path/to/file') start=GridState(43, 65) goal=GridState(132, 94) heuristic=OctileGridHeuristic

Todo:
    * implement logging solutions for debug printing/to file and history tracking to console / to file
    * implement the module level command line interface
    * redo __repr__
"""


class GenericDFS(GenericBFS):
    """ This class implements the DFS algorithm.

    GenericDFS inherits from GenericBFS becuase DFS and BFS are basically the same
    algorithm with the open list as a FIFO queue swapped out for a Stack. This was
    accomplished by overriding the _remove_from_open() method to pop off the back
    of the open list instead of the front.

    All attributes are read-only properties.

    Attributes:
        env (:obj:'Environment'): A class that represents the environment being
            being searched.
        goal (:obj:'Node'): A class that represents the node to search
            to. The default is a random passable node from the provided 'env'.
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
    def __init__(self,  
                env: Environment, 
                start: State=None, 
                goal: State=None, 
                verbose: bool=False):
        """GenericDFS __init__ method.

        Attributes env, start, goal, nodes_expanded, path, success, verbose
        are instantiated in parent's parent class Search __init__.

        When start and goal are left as None, the default behavior is
        to pick a random, valid state using env.get_random().  This
        occurs in Search __init__.

        The start node is added to open in parent's class
        GenericBFS __init__.
 
        Args:
            env (:obj:'Environment'): Environment being being searched.
            start (:obj:`State`, optional): State to start search from.
            goal (:obj:`State`): State to search to.
            verbose (:obj:'bool'): Flag for verbose printing.
        """
        super().__init__(env=env, start=start, goal=goal)
    
    def _remove_from_open(self):
        return self._open.pop(len(self._open) - 1)