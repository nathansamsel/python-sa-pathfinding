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
    
    def __str__(self) -> str:
        return self.__repr__()

    def __repr__(self) -> str:
        rep = 'TOHState<['
        for peg in self._pegs:
            rep += '['
            if len(peg) > 0:
                for d, disk in enumerate(peg):
                    rep += f'{disk}'
                    if d < len(peg) - 1:
                        rep += ','
            rep += ']'
        rep += ']>'
        return rep

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
    def __init__(self, start_peg: int, goal_peg: int) -> None:
        self._start_peg = start_peg
        self._goal_peg = goal_peg

    def __str__(self):
        return self.__repr__()
    
    def __repr__(self):
        return f'<TOHAction({self._start_peg},{self._goal_peg})>'
    
    def __eq__(self, other):
        return self._start_peg == other._start_peg and self._goal_peg == other._goal_peg
    
    def __ne__(self, other):
        return not self.__eq__(other)
    
    @property
    def start_peg(self) -> int:
        return self._start_peg
    
    @property
    def goal_peg(self) -> int:
        return self._goal_peg


class TowersOfHanoi(Environment):
    def __init__(self, 
                    pegs: int, 
                    disks: int, 
                    start_peg: int = -1, 
                    goal_peg: int = -1):
        self._num_pegs = pegs
        self._num_disks = disks
        if self._num_pegs < 3:
            raise StateDoesNotExistError('Too few pegs.')
        if start_peg < -1 or start_peg > self._num_pegs:
            raise StateDoesNotExistError('Invalid start peg.')
        if goal_peg < -1 or goal_peg > self._num_pegs:
            raise StateDoesNotExistError('Invalid goal peg.')
        # default behaviour is to pick start and end pegs randomly
        if start_peg == -1:
            while start_peg == goal_peg or start_peg == -1:
                start_peg = random.randint(0, self._num_pegs)
        if goal_peg == -1:
            while goal_peg == start_peg or goal_peg == -1:
                goal_peg = random.randint(0, self._num_pegs)
        self._start_peg = start_peg
        self._goal_peg = goal_peg


    def apply_action(self,
                     state: TOHState,
                     action: TOHAction) -> TOHState:
        new_pegs = copy.deepcopy(state.pegs)
        disk = new_pegs[action.start_peg].pop()
        new_pegs[action.goal_peg].append(disk)
        return TOHState(new_pegs)

    def get_actions(self,
                    state: TOHState,
                    parent: TOHState) -> List[Tuple[TOHAction, float]]:
        action_cost_tuples = []
        # n-squared comparisons of top disks to eachother
        # to check if valid actions exist for each top disk
        for td, top_disk in enumerate(state.tops):
            for cd, _ in enumerate(state.tops):
                # skip same disk comparisons and empty pegs
                if td == cd or top_disk == -1:
                    continue
                potential_action = TOHAction(td, cd)
                #if comp_disk == -1 or top_disk < comp_disk:
                if self.is_action_valid(state, potential_action):
                    action_cost_tuples.append((potential_action, 1))
        return action_cost_tuples

    def get_stacked_state(self, peg: int) -> TOHState:
        state = list()
        for p in range(self._num_pegs):
            state.append(list())
            if p == peg:
                for d in range(self.num_disks, 0, -1):
                    state[p].append(d)
        return TOHState(state)
    
    def get_random(self, valid: bool = True) -> TOHState:
        disks = [i for i in range(1, self._num_disks + 1)]
        state = []
        lengths = []
        disk_count = self._num_disks
        pegs = self._num_pegs
        while pegs > 0:
            if disk_count > 0:
                lengths.append(random.randint(0, disk_count))
                disk_count -= lengths[-1]
            else:
                lengths.append(0)
            pegs -= 1
        for length in lengths:
            state.append([])
            for _ in range(length):
                disk_index = random.randint(0, len(disks) - 1)
                state[-1].append(disks.pop(disk_index))
            state[-1].sort(reverse=True)
        return TOHState(state)
    
    def get_random_stacked_state(self) -> TOHState:
        return self.get_stacked_state(random.randint(0, self._num_pegs))

    def is_action_defined(self, state: TOHState, action: TOHAction) -> bool:
        if not (action.start_peg in range(self._num_pegs) and action.goal_peg in range(self._num_pegs)):
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
        if len(state.pegs[action.goal_peg]) == 0:
            return True
        # is it too big to move to goal_peg?
        # made as else if check to make sure list isn't empty so indexing doesn't throw and error
        elif state.pegs[action.start_peg][-1] >= state.pegs[action.goal_peg][-1]:
            return False
        return True

    def is_defined(self, state: TOHState) -> bool:
        # only way for a List[List[int]] to be undefined here
        # is with negative numbers - its doesn't really make sense
        # since disk value is its size
        for peg in state.pegs:
            for disk in peg:
                if disk < 0:
                    return False
        return True

    def is_valid(self, state: TOHState) -> bool:
        # valid = d1, d2 for all disk on peg, d1 > d2
        # pegs = list of descending-ordered lists of ints
        last_disk = None
        for peg in state.pegs:
            for disk in peg:
                if last_disk is not None and disk > last_disk:
                    return False
                last_disk = disk
            last_disk = None
        return True

    @property
    def num_pegs(self) -> int:
        return self._num_pegs
    
    @property
    def num_disks(self) -> int:
        return self._num_disks
    
    @property
    def start_peg(self) -> int:
        return self._start_peg

    @property
    def goal_peg(self) -> int:
        return self._goal_peg

    @property
    def start(self) -> TOHState:
        return self.get_stacked_state(self._start_peg)
    
    @property
    def goal(self) -> TOHState:
        return self.get_stacked_state(self._goal_peg)
