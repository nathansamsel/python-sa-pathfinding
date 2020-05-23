from sa_pathfinding.environments.towers_of_hanoi.towers_of_hanoi import TOHState
from sa_pathfinding.environments.towers_of_hanoi.towers_of_hanoi import TOHAction
from sa_pathfinding.environments.towers_of_hanoi.towers_of_hanoi import TowersOfHanoi

def test_TOHState_eq():
    assert TOHState([[3, 2],[1],[]]) == TOHState([[3, 2], [1], []])
    assert TOHState([[3, 2, 1],[],[]]) == TOHState([[3, 2, 1], [], []])
    assert TOHState([[3],[2],[1]]) == TOHState([[3], [2], [1]])
    assert TOHState([[3, 2, 1],[],[]]) != TOHState([[], [], [3, 2, 1]])
    assert TOHState([[3],[1],[2]]) != TOHState([[3], [2], [1]])
