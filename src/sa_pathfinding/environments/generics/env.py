from abc import abstractmethod
from typing import Tuple
from typing import List
from enum import Enum
from abc import ABC

from sa_pathfinding.environments.generics.state import State


class StateDoesNotExistError(Exception):
    """Raised when state does not exist"""
    def __init__(self, state: State):
        self.message = f"State {state} does not exist."


class StateNotValidError(Exception):
    """Raised when state is not passable"""
    def __init__(self, state: State):
        self.message = f"State {state} is not valid."


class Action(ABC):
    pass


class Environment(ABC):

    @abstractmethod
    def apply_action(self,
                     state: State,
                     action: Action) -> State:
        pass

    @abstractmethod
    def get_actions(self,
                    state: State,
                    parent: State) -> List[Tuple[Action, float]]:
        pass

    @abstractmethod
    def is_defined(self, state: State) -> bool:
        pass

    @abstractmethod
    def is_valid(self, state: State) -> bool:
        pass

    @abstractmethod
    def get_random(self, valid: bool = True) -> State:
        pass
