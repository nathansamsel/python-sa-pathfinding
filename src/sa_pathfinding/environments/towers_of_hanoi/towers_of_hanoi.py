from typing import Tuple
from typing import List
import random
import copy
import math

from sa_pathfinding.environments.generics.env import StateDoesNotExistError
from sa_pathfinding.environments.generics.env import Environment
from sa_pathfinding.environments.generics.state import State
from sa_pathfinding.environments.generics.env import Action

class TOHState(State):
    def __init__(self, env_structure: List[List[int]]) -> None:
        self._pegs = env_structure
    
    def __str__(self) -> str:
        return self.__repr__()

    def __repr__(self) -> str:
        rep = '['
        for peg in self._pegs:
            rep += '['
            if len(peg) > 0:
                for d, disk in enumerate(peg):
                    rep += f'{disk}'
                    if d < len(peg) - 1:
                        rep += ','
            rep += ']'
        rep += ']'
        return rep


    def __eq__(self, other) -> bool:
        if len(other.pegs) != len(self._pegs):
            return False
        for i, peg in enumerate(self._pegs):
            if len(peg) != len(other.pegs[i]):
                return False
            for d, disk in enumerate(self._pegs[i]):
                if disk != other._pegs[i][d]:
                    return False
        return True
            

    def __ne__(self, other) -> bool:
        return not self.__eq__(other)

    def _get_peg_tops(self) -> List[int]:
        # flatten pegs 2d-array by picking off the 'tops' of each peg
        # and use -1 to show empty pegs
        tops = []
        for peg in self.pegs:
            try:
                tops.append(peg[-1])
            except IndexError:
                tops.append(-1)
        return tops

    def get_state(self) -> List[List[int]]:
        return self._pegs
    
    @property
    def pegs(self) -> List[List[int]]:
        return self._pegs
    
    @property
    def tops(self) -> List[int]:
        return self._get_peg_tops()

class TOHAction(Action):
    def __init__(self, start_peg: int, end_peg: int) -> None:
        self._start_peg = start_peg
        self._end_peg = end_peg
    
    def __str__(self):
        return self.__repr__()
    
    def __repr__(self):
        return f'<TOHAction({self._start_peg},{self._end_peg})>'
    
    def __eq__(self, other):
        return self._start_peg == other._start_peg and self._end_peg == other._end_peg
    
    def __ne__(self, other):
        return not self.__eq__(other)
    
    @property
    def start_peg(self) -> int:
        return self._start_peg
    
    @property
    def end_peg(self) -> int:
        return self._end_peg

class TowersOfHanoi(Environment):
    def __init__(self, 
                    pegs: int, 
                    disks: int, 
                    start_peg: int = -1, 
                    end_peg: int = -1):
        self._num_pegs = pegs
        self._num_disks = disks
        if self._num_pegs < 3:
            raise StateDoesNotExistError('Too few pegs.')
        if start_peg < -1 or start_peg > self._num_pegs:
            raise StateDoesNotExistError('Invalid start peg')
        if end_peg < -1 or end_peg > self._num_pegs:
            raise StateDoesNotExistError('Invalid goal peg')
        if start_peg == -1:
            start_peg = random.randint(0, self._num_pegs)
        if end_peg == -1:
            while end_peg == start_peg:
                end_peg = random.randint(0, self._num_pegs)
        self._start_peg = start_peg
        self._end_peg = end_peg

    def apply_action(self,
                     state: TOHState,
                     action: TOHAction) -> TOHState:
        new_pegs = copy.deepcopy(state.pegs)
        disk = new_pegs[action.start_peg].pop()
        new_pegs[action.end_peg].append(disk)
        new_state = TOHState(new_pegs)
        return new_state
    
    def create_start_state(self) -> TOHState:
        start = list()
        for p in range(self._num_pegs):
            start.append(list())
            if p == self._start_peg:
                for d in range(self.num_disks, 0, -1):
                    start[p].append(d)
        return TOHState(start)
    

    def create_goal_state(self) -> TOHState:
        goal = list()
        for p in range(self._num_pegs):
            goal.append(list())
            if p == self._end_peg:
                for d in range(self.num_disks, 0, -1):
                    goal[p].append(d)
        return TOHState(goal)


    def get_actions(self,
                    state: TOHState,
                    parent: TOHState) -> List[Tuple[TOHAction, float]]:
        action_cost_tuples = []
        # n-squared comparisons of top disks to eachother
        # to check if valid actions exist for each top disk
        for td, top_disk in enumerate(state.tops):
            for cd, comp_disk in enumerate(state.tops):
                # skip same disk comparisons and empty pegs
                if td == cd or top_disk == -1:
                    continue
                potential_action = TOHAction(td, cd)
                #if comp_disk == -1 or top_disk < comp_disk:
                if self.is_action_valid(state, potential_action):
                    action_cost_tuples.append((potential_action, 1))
        return action_cost_tuples

    def is_action_defined(self, state: TOHState, action: TOHAction) -> bool:
        if not (action.start_peg in range(self._num_pegs) and action.end_peg in range(self._num_pegs)):
            return False
        return True

    
    def is_action_valid(self, state: TOHState, action: TOHAction) -> bool:
        # is the action in-bounds?
        if not self.is_action_defined(state, action):
            return False
        # is there a disk to move?
        if len(state.pegs[action.start_peg]) == 0:
            return False
        # is it moving to an empty peg?
        if len(state.pegs[action.end_peg]) == 0:
            return True
        # is it too big to move to end_peg?
        # else if check to make sure list isn't empty so indexing doesn't throw and error
        elif state.pegs[action.start_peg][-1] >= state.pegs[action.end_peg][-1]:
            return False
        return True

    def is_defined(self, state: TOHState) -> bool:
        return True

    def is_valid(self, state: TOHState) -> bool:
        last_disk = None
        for peg in state.pegs:
            for disk in peg:
                if last_disk is not None and disk > last_disk:
                    return False
                last_disk = disk
            last_disk = None
        return True

    def get_random(self, valid: bool = True) -> TOHState:
        pass

    @property
    def num_pegs(self):
        return self._num_pegs
    
    @property
    def num_disks(self):
        return self._num_disks