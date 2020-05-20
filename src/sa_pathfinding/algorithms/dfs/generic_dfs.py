import queue

from sa_pathfinding.algorithms.bfs.generic_bfs import GenericBFS
from sa_pathfinding.environments.generics.env import Environment
from sa_pathfinding.algorithms.generics.search_node import SearchNode


class GenericDFS(GenericBFS):
    def __init__(self, 
                env: Environment, 
                start: SearchNode=None, 
                goal: SearchNode=None, 
                verbose: bool=False):
        super().__init__(env=env, start=start, goal=goal)
        self._open = queue.LifoQueue()
        self._open.put(self._start)