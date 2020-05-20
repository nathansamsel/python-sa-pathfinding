from abc import abstractmethod
from typing import List
from abc import ABC

from sa_pathfinding.environments.generics.state import State
from sa_pathfinding.environments.grids.octile_grid import StateNotValidError
from sa_pathfinding.algorithms.generics.search_node import SearchNode
from sa_pathfinding.environments.generics.env import Environment

# TODO(Nathan): Change start and goal in all searches to State, create the search node within the search


class Search(ABC):
    """ An abstract class used to represent search algorithms generally.

    Search algorithms can subclass this abstract class and implement the
    interface to receive some general search functionality and
    implementation guidance

    Note:

    Attributes:
        env (:obj:'Grid'): A class that represents the environment being
            being searched.
        start (:obj:'Node', optional): A class that represent the node to start
            the search from. The default is a random passable node from the
            provided 'env'.
        goal (:obj:'Node', optional): A class that represents the node to search
            to. The default is a random passable node from the provided 'env'.
        verbose(:obj:'bool', optional): A boolean flag that, when true, enables
            the printing of information about the search as it runs.
    """

    __slots__ = '_env _start _goal _verbose ' \
                '_nodes_expanded _path _success _history'.split()

    def __init__(self,
                 env: Environment,
                 start: SearchNode = None,
                 goal: SearchNode = None,
                 verbose: bool = False) -> None:
        self._env = env
        self._start = start
        self._goal = goal
        self._verbose = verbose
        self._nodes_expanded = 0
        self._path = []
        self._success = None

        if start is None:
            self._start = self._get_random_start()
        elif self._env.is_valid(start.state):
            self._start = start
        else:
            raise StateNotValidError(start.state)

        if goal is None:
            self._goal = self._get_random_goal()
        elif self._env.is_valid(goal.state):
            self._goal = goal
        else:
            raise StateNotValidError(goal.state)

        self._history = {'start': repr(self._start.state),
                        'goal': repr(self._goal.state),
                        'nodes_expanded': self._nodes_expanded,
                        'steps': {}}

        if self._verbose:
            print(f"Search initialized...")
            print(f"Start = {self._start}")
            print(f"Goal = {self._goal}")

    def __repr__(self) -> str:
        pass

    @property
    def start(self):
        return self._start

    @property
    def goal(self):
        return self._goal

    @property
    def nodes_expanded(self):
        return self._nodes_expanded

    @property
    def path(self):
        return self._path

    @property
    def verbose(self):
        return self._verbose
    
    @property
    def history(self):
        return self._history

    @abstractmethod
    def get_path(self) -> List[SearchNode]:
        pass

    def _get_random(self) -> SearchNode:
        return SearchNode(self._env.get_random(valid=True))

    def _get_random_diff(self, diff_state: State) -> SearchNode:
        state = self._env.get_random()
        if diff_state is not None:
            while state == diff_state:
                state = self._env.get_random()
        return SearchNode(state)

    def _get_random_start(self) -> SearchNode:
        return self._get_random_diff(self._goal.state) \
            if self._goal is not None else self._get_random()

    def _get_random_goal(self) -> SearchNode:
        return self._get_random_diff(self._start.state) \
            if self._start is not None else self._get_random()
