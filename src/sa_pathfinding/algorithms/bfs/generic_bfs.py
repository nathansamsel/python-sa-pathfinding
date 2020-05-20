from typing import List
import queue

from sa_pathfinding.algorithms.generics.search import Search
from sa_pathfinding.environments.generics.env import Environment
from sa_pathfinding.algorithms.generics.search_node import SearchNode

class GenericBFS(Search):
    def __init__(self, 
                env: Environment,
                start: SearchNode,
                goal: SearchNode,
                verbose: bool=False):
        super().__init__(env, start=start, goal=goal, verbose=verbose)
        self._open = queue.Queue()
        self._open.put(self._start)
    
    def _add_to_open(self, new_node: SearchNode) -> None:
        self._open.put(new_node)

    def _pull_from_open(self) -> SearchNode:
        return self._open.get()
    
    def step(self) -> (SearchNode, List[SearchNode]):
        if self._open.empty():
            self._success = False
            if self._verbose:
                print("path does not exist.")
            return None, None
        
        node = self._pull_from_open()

        self._nodes_expanded += 1
        self.history['nodes_expanded'] = self._nodes_expanded

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
        
        parent = node.parent.state if node.parent is not None else None
        action_cost_tuples = self._env.get_actions(node.state, parent)
        to_open = list()

        # generate children nodes based on available actions of our 'state'
        for action, _ in action_cost_tuples:

            # apply the action to generate new state
            new_state = self._env.apply_action(node.state, action)
            new_node = SearchNode(new_state,
                                  parent=node)
            self._open.put(new_node)
            to_open.append(new_node)
        
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
    
    @property
    def open(self):
        return self._open