from __future__ import annotations

from enum import Enum
import math

from sa_pathfinding.environments.generics.state import State


class Status(Enum):
    UNDISCOVERED = 0
    ON_OPEN = 1
    ON_CLOSED = 2


class SearchNode:

    __slots__ = '_state gcost hcost fcost parent'.split()

    def __init__(self,
                 state: State,
                 gcost: float = None,
                 hcost: float = None,
                 fcost: float = None,
                 parent: SearchNode = None):

        self.gcost = gcost
        self.hcost = hcost
        self.fcost = fcost
        self.parent = parent
        self._state = state

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return NotImplemented
        return self._state == other.state

    def __ne__(self, other):
        return not self.__eq__(other)

    def __lt__(self, other):
        if self.fcost < other.fcost:
            return True
        elif math.isclose(self.fcost, other.fcost):
            if self.gcost > other.gcost:
                return True
        return False

    def __repr__(self):
        if self is None:
            return 'Uninitialized node: None'
        else:
            if self.gcost is None or self.hcost is None or self.fcost is None \
                    or self.parent is None:
                return f"<Node({self._state})>"
            else:
                return f"<Node({self._state}) g={self.gcost:.2f} " \
                       f"h={self.hcost:.2f} f={self.fcost:.2f} " \
                       f"parent=({self.parent._state})>"

    @property
    def state(self):
        return self._state
