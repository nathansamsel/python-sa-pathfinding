from sa_pathfinding.environments.towers_of_hanoi.towers_of_hanoi import TowersOfHanoi
from sa_pathfinding.environments.towers_of_hanoi.towers_of_hanoi import TOHAction
from sa_pathfinding.environments.towers_of_hanoi.towers_of_hanoi import TOHState
from sa_pathfinding.algorithms.dijkstra.generic_dijkstra import GenericDijkstra
from sa_pathfinding.algorithms.generics.search_node import SearchNode
from sa_pathfinding.algorithms.bfs.generic_bfs import GenericBFS

def test_TOHState_eq():
    assert TOHState([[3, 2],[1],[]]) == TOHState([[3, 2], [1], []])
    assert TOHState([[3, 2, 1],[],[]]) == TOHState([[3, 2, 1], [], []])
    assert TOHState([[3],[2],[1]]) == TOHState([[3], [2], [1]])
    assert TOHState([[3, 2, 1],[],[]]) != TOHState([[], [], [3, 2, 1]])
    assert TOHState([[3],[1],[2]]) != TOHState([[3], [2], [1]])

def test_repr():
    assert repr(TOHState([[3, 2],[1],[]])) == 'TOHState<[[3,2][1][]]>'

def test_str():
    assert str(TOHState([[3, 2],[1],[]])) == repr(TOHState([[3, 2],[1],[]]))

def test_undefined_state():
    toh = TowersOfHanoi(3, 4)
    assert not toh.is_defined(TOHState([[4, -2, 3],[1],[2]]))
    assert toh.is_defined(TOHState([[4, 3],[1],[2]]))

def test_invalid_state():
    toh = TowersOfHanoi(3, 4)
    assert toh.is_valid(TOHState([[4, 3],[1],[2]]))
    assert not toh.is_valid(TOHState([[3, 4],[1],[2]]))
