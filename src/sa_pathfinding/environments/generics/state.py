from abc import abstractmethod
from abc import ABC


class State(ABC):

    @abstractmethod
    def get_state(self):
        pass
