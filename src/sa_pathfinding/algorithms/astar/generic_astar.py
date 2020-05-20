from typing import List
import heapq

from sa_pathfinding.algorithms.generics.search_node import SearchNode
from sa_pathfinding.environments.generics.env import Environment
from sa_pathfinding.algorithms.generics.search import Search
from sa_pathfinding.environments.generics.state import State
from sa_pathfinding.heuristics.heuristic import Heuristic



class GenericAstar(Search):
    """ A class to represent the A* search algorithm.

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

    __slots__ = '_open _closed _heuristic'.split()

    def __init__(self,
                 env: Environment,
                 heuristic: Heuristic,
                 start: SearchNode = None,
                 goal: SearchNode = None,
                 verbose: bool = False):
        super().__init__(env, start=start, goal=goal, verbose=verbose)
        self._heuristic = heuristic
        self._history['heuristic'] = str(self._heuristic.name)

        self._open = []  # uses heapq
        self._closed = []  # just a list

        # setup beginning of search by assigning costs to start
        # and adding it to the open list
        self._start.gcost = 0
        self._start.hcost = self._heuristic.get_cost(self._start.state,
                                                     self._goal.state)
        self._start.fcost = self._start.gcost + self._start.hcost
        self._start.parent = None
        self._add_to_open(self._start)

    def __repr__(self) -> str:
        rep = repr(super())
        if not self._heuristic.name == 'NONE':
            rep += 'Heuristic: ' + self._heuristic.name
        return rep

    @property
    def open(self):
        return self._open

    @property
    def heuristic(self):
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

    def step(self) -> (SearchNode, List[SearchNode]):

        # if the open list is empty, then we have explored every state in
        # the env state space and have not found the goal.
        # Since A* is a "complete" search algorithm, which means that it will
        # find the goal if it exists and is 'reachable', so we can conclude a
        # path to the requested goal does not exist
        if len(self._open) == 0:
            self._success = False
            if self._verbose:
                print("path does not exist.")
            # None in the first element of the tuple means that the search
            # has terminated while a Node in that position means that the search
            # executed a step and is returning the node that was added to closed
            # during that step. Second tuple element is either a list of nodes
            # to be interpreted as the path being returned from a successful
            # search or the nodes added to open on that step, depending on the
            # first param of the tuple
            return None, None

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
        if node == self.goal:
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
            return None, self._path

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
            new_node.hcost = self._heuristic.get_cost(new_node.state,
                                                      self.goal.state)
            new_node.fcost = new_node.gcost + new_node.hcost

            # if its already been expanded, then it was expanded with lowest
            # f-cost, so a shortest path cannot be found by visiting
            # the node through a higher f-cost parent - a.k.a safe to skip
            if self._is_on_closed(new_node):
                continue

            # if its not on open (Status: UNDISCOVRED)
            # add it to both open and the list to be returned
            if not self._is_on_open(new_node):
                self._add_to_open(new_node)
                to_open.append(new_node)
            else:
                # otherwise it is on open, so find the index
                is_on_open, index = self._is_on_open_w_index(new_node)
                if not is_on_open:
                    raise Exception('Status = ON_OPEN, but not found on open.')

                # update cost if found on open with a higher cost
                # this means we found a path through this node with a better
                # cost than we had previously found
                elif new_node.fcost < self._open[index].fcost:
                    # if we need to update cost
                    # then it needs to be removed from heap
                    # the heap property needs to be restored (heapify)
                    # then it needs to be added again
                    self._open[index] = self._open[-1]
                    self._open.pop()
                    heapq.heapify(self._open)
                    self._add_to_open(new_node)
        self._history['steps'][f"step-{self._nodes_expanded}"] = {}
        self._history['steps'][f"step-{self._nodes_expanded}"]['expanded'] = repr(node)
        self._history['steps'][f"step-{self._nodes_expanded}"]['to_open'] = repr(to_open)
        return node, to_open

    def get_path(self) -> List[SearchNode]:
        if self._verbose:
            print("Starting search...")
        while True:
            node, to_open = self.step()
            if node is None:
                if self._success:
                    return self._path
                elif to_open is None:
                    return []
            if self._verbose:
                print(f"Step: {self._nodes_expanded}, "
                      f"Chosen for expansion: {node}, "
                      f"Nodes generated: {to_open}")
