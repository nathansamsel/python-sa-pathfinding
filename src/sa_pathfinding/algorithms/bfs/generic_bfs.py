from typing import List
import queue

from sa_pathfinding.algorithms.generics.search_node import SearchNode
from sa_pathfinding.environments.generics.env import Environment
from sa_pathfinding.algorithms.generics.search import Search
from sa_pathfinding.environments.generics.state import State

"""generic_bfs Module

This module contains the implementation for the Breadth-First Search (BFS) algorithm.

The below example has not been implemented yet. coming soon...

Example:
    The module can be run from the command line to execute a BFS search.
    The command line args are the same as the parameters for the
    GenericBFS object. The result is a string representation of the
    path that was found - a comma separated list of state __repr__() calls::

        $ python generic_bfs.py OctileGrid('path/to/file')
        $ python generic_bfs.py OctileGrid('path/to/file') start=GridState(43, 65)
        $ python generic_bfs.py OctileGrid('path/to/file') goal=GridState(43, 65)
        $ python generic_bfs.py OctileGrid('path/to/file') start=GridState(43, 65) goal=GridState(132, 94)
        $ python generic_bfs.py OctileGrid('path/to/file') start=GridState(43, 65) goal=GridState(132, 94) heuristic=OctileGridHeuristic

Todo:
    * implement logging solutions for debug printing/to file and history tracking to console / to file
    * implement the module level command line interface
    * redo __repr__
"""

class GenericBFS(Search):
    """ This class implements the BFS algorithm.

    The open list is a standard list that is treated like a First-in First-out (FIFO)
    queue. 

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
                start: State,
                goal: State,
                verbose: bool=False):
        """GenericBFS __init__ method.

        Attributes env, start, goal, nodes_expanded, path, success, verbose
        are instantiated in parent's class Search __init__.

        When start and goal are left as None, the default behavior is
        to pick a random, valid state using env.get_random().  This
        occurs in Search __init__.

        The start node is added to open.

        Args:
            env (:obj:'Environment'): Environment being being searched.
            start (:obj:`State`, optional): State to start search from.
            goal (:obj:`State`): State to search to.
            verbose (:obj:'bool'): Flag for verbose printing.
        """
        super().__init__(env, 
                        start=start, 
                        goal=goal, 
                        verbose=verbose)
        self._open = []
        self._add_to_open(SearchNode(self._start))
    
    def _add_to_open(self, new_node: SearchNode) -> None:
        self._open.append(new_node)

    def _remove_from_open(self) -> SearchNode:
        return self._open.pop(0)
    
    def step(self):
        """step generator

        Yields:
            Tuple[SearchNode, List[SearchNode]]: A tuple of the node expanded and its
                children that were added to the open list.

        Example:

            >>> print([repr(node) + ' ' + repr(to_open)) for node, to_open in astar.step()])
            [SearchNode<...> [SearchNode<...>, SearchNode<...>, ...], ...]
        """
        while len(self._open) > 0:
            
            node = self._remove_from_open()

            self._nodes_expanded += 1
            self.history['nodes_expanded'] = self._nodes_expanded

            if node.state == self.goal:
                self._success = True
                # re-create path by following parents from goal to start
                self._path.append(node.state)
                # start has None as parent, so walk back until that None parent is hit
                while node.parent is not None:
                    node = node.parent
                    self._path.append(node.state)
                self._path = list(reversed(self._path))
                self._history['path'] = self._path
                if self._verbose:
                    print("---------------------------------------------")
                    print("Search terminated successfully")
                    print(f"Path of length {len(self._path)} from {self._start} "
                        f"to {self._goal} found.")
                    print(f"Nodes Expanded: {self._nodes_expanded}")
                    print(f"Path: {self._path}")
                    print("---------------------------------------------\n\n")
                return
            
            parent = node.parent.state if node.parent is not None else None
            action_cost_tuples = self._env.get_actions(node.state, parent)
            to_open = list()

            # generate children nodes based on available actions of our 'state'
            # ignoring cost becuase BFS doesn't have a concept of cost, just order
            for action, _ in action_cost_tuples:

                # apply the action to generate new state
                new_state = self._env.apply_action(node.state, action)
                new_node = SearchNode(new_state, parent=node)
                self._add_to_open(new_node)
                to_open.append(new_node)
            
            self._history['steps'][f"step-{self._nodes_expanded}"] = {}
            self._history['steps'][f"step-{self._nodes_expanded}"]['expanded'] = repr(node)
            self._history['steps'][f"step-{self._nodes_expanded}"]['to_open'] = repr(to_open)
            yield node, to_open
        return

    def get_path(self) -> List[State]:
        """get_path() executes the search from beginning to end. No other
        method needs to be called after instantiation to complete a
        successful search.

        Returns:
            List[State] where list is empty if search does not return
                a path and full of connected nodes if a path was found.
        """ 
        if self._verbose:
            print("Starting search...")
        for node, to_open in self.step():
            if self._verbose:
                print(f"Step: {self._nodes_expanded}, "
                      f"Chosen for expansion: {node}, "
                      f"Nodes generated: {to_open}")
        return self._path
    
    @property
    def open(self):
        return self._open
