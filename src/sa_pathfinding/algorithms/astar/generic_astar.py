from typing import List
import heapq

from sa_pathfinding.algorithms.generics.search_node import SearchNode
from sa_pathfinding.environments.generics.env import Environment
from sa_pathfinding.algorithms.generics.search import Search
from sa_pathfinding.environments.generics.state import State
from sa_pathfinding.heuristics.heuristic import Heuristic

"""generic_astar Module

This module contains the implementation for the A* algorithm.

The below example has not been implemented yet. coming soon...

Example:
    The module can be run from the command line to execute an A* search.
    The command line args are the same as the parameters for the
    GenericAstar object. The result is a string representation of the
    path that was found - a comma separated list of state __repr__() calls::

        $ python generic_astar.py OctileGrid('path/to/file')
        $ python generic_astar.py OctileGrid('path/to/file') start=GridState(43, 65)
        $ python generic_astar.py OctileGrid('path/to/file') goal=GridState(43, 65)
        $ python generic_astar.py OctileGrid('path/to/file') start=GridState(43, 65) goal=GridState(132, 94)
        $ python generic_astar.py OctileGrid('path/to/file') start=GridState(43, 65) goal=GridState(132, 94) heuristic=OctileGridHeuristic

Todo:
    * implement logging solutions for debug printing/to file and history tracking to console / to file
    * implement the module level command line interface
    * redo GenericAstar and Search __repr__
"""


class GenericAstar(Search):
    """ This class implements the A* search algorithm.

    By default, the open and closed lists are both simple lists. The open list
    is maintained as a heap using heapq, sorted on f-cost with ties broken to
    high g-cost. This means that cost of removing the best node from the open
    list on each step is O(lg(n)). Both lists are checked for membership in 
    O(n) time using the env state __eq__() in a loop.
    
    The open and closed lists are checked for membership
    via the _is_on_open() and _is_on_closed() methods. If you define a 
    custom environment and can structure the open and closed list checks
    in a way that beats O(n) checks, then extend GenericAstar by inheriting
    from it and overriding the above methods to optimize an A* 
    implementation for your environment. An example of this is
    GridOptimizedAstar where a 2d status structure was added as
    an effective overlay on the grids such that indexes align with grid
    positions, creating constant time checks for both lists.

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

    __slots__ = '_open _closed _heuristic'.split()

    def __init__(self,
                 env: Environment,
                 heuristic: Heuristic,
                 start: State = None,
                 goal: State = None,
                 verbose: bool = False):
        """GenericAstar __init__ method.

        Attributes env, start, goal, nodes_expanded, path, success, verbose
        are instantiated in parent class Search __init__.

        The start node is assigned initial costs and added to open.

        Args:
            env (:obj:'Environment'): Environment being being searched.
            start (:obj:`State`, optional): State to start search from.
            goal (:obj:`State`): State to search to.
            verbose (:obj:'bool'): Flag for verbose printing.
        """
        super().__init__(env, start=start, goal=goal, verbose=verbose)
        self._heuristic = heuristic
        self._history['heuristic'] = str(self._heuristic.name)

        self._open = []
        self._closed = []

        # setup beginning of search by assigning costs to start
        # and adding it to the open list
        hcost = self._heuristic.get_cost(self._start, self._goal)
        start_node = SearchNode(self._start, 
                                gcost=0, 
                                hcost=hcost, 
                                fcost=hcost, 
                                parent=None)
        self._add_to_open(start_node)

    def __repr__(self) -> str:
        rep = repr(super())
        if not self._heuristic.name == 'NONE':
            rep += 'Heuristic: ' + self._heuristic.name
        return rep

    @property
    def open(self):
        """list of SearchNode: open list"""
        return self._open
    
    @property
    def closed(self):
         """list of SearchNode: closed list"""
         return self._closed

    @property
    def heuristic(self):
         """str: heuristic name."""
         return self._heuristic.name

    def _add_to_open(self, node: SearchNode) -> None:
        heapq.heappush(self._open, node)

    def _add_to_closed(self, node: SearchNode) -> None:
        self._closed.append(node)

    def _remove_best(self) -> SearchNode:
        return heapq.heappop(self._open)

    def _is_on_open(self, node: SearchNode) -> bool:
        return self._is_on_list(node, self._open)

    def _is_on_closed(self, node: SearchNode) -> bool:
        return self._is_on_list(node, self._closed)

    @staticmethod
    def _is_on_list(node: SearchNode,
                    some_list: List) -> bool:
        for some_node in some_list:
            if some_node == node:
                return True

    @staticmethod
    def _is_on_list_w_index(some_node: SearchNode,
                            some_list: List) -> (bool, int):
        in_list = -1
        for i, node in enumerate(some_list):
            if node == some_node:
                in_list = i
        if in_list == -1:
            return False, None
        else:
            return True, in_list

    def _is_on_open_w_index(self, some_node: SearchNode) -> (bool, int):
        return self._is_on_list_w_index(some_node, self._open)

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
            # if the open list is empty, then we have explored every state in
            # the env state space and have not found the goal.
            # Since A* is a "complete" search algorithm, which means that it will
            # find the goal if it exists and is 'reachable', so we can conclude a
            # path to the requested goal does not exist
            # this forces a return -> StopIteration for this generator function 

            # remove the lowest f-cost (ties to high g-cost) node from
            # the open list, add to closed
            node = self._remove_best()
            self._add_to_closed(node)
            self._nodes_expanded += 1
            self.history['nodes_expanded'] = self._nodes_expanded

            # Goal Check
            # This needs to happen after the node has been selected as the lowest
            # f-cost node on the open list in order to prove its an optimal path.
            # Common mistake is to goal check when the node is generated by the
            # expansion of its parent.
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
                # once path and successful path info are recorded - we are done
                # can return to force a StopIteration for this generator function
                return

            # get possible actions for selected node
            # querying the environment for the actions available to the given state
            # parent.state check because start has None parent and wont have state
            parent = node.parent.state if node.parent is not None else None
            action_cost_tuples = self._env.get_actions(node.state, parent)
            to_open = list()

            # generate children nodes based on available actions of our 'state'
            for action, cost in action_cost_tuples:

                # apply the action to generate new state
                new_state = self._env.apply_action(node.state, action)
                new_node = SearchNode(new_state,
                                    gcost=node.gcost,
                                    hcost=node.hcost,
                                    fcost=node.fcost,
                                    parent=node)

                # set costs according to A* algorithm
                new_node.gcost += cost
                new_node.hcost = self._heuristic.get_cost(new_node.state, self.goal)
                new_node.fcost = new_node.gcost + new_node.hcost

                is_on_open, index = self._is_on_open_w_index(new_node)
                if not is_on_open:
                    # if node is not on open, its either
                    # undiscovered or expanded already and on closed
                    # safe to skip if on closed because it was chosen
                    # for expansion and added to closed with the lowest f-cost
                    # so a shorter path to that state does not exist
                    if not self._is_on_closed(new_node):
                        self._add_to_open(new_node)
                        to_open.append(new_node)
                else:
                    # if found on open, means different path to same state was found
                    # check cost to see if found a shorter path to that state
                    if new_node.fcost < self._open[index].fcost:
                        # if we need to update cost
                        # then it needs to be removed from heap
                        # the heap property needs to be restored (heapify)
                        # then it needs to be added again
                        # due to the heap being sorted on f-cost
                        # and this forces a change to f-cost
                        self._open[index] = self._open[-1]
                        self._open.pop()
                        heapq.heapify(self._open)
                        self._add_to_open(new_node)
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
