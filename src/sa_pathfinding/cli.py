"""
Module that contains the command line app.

Why does this file exist, and why not put this in __main__?

  You might be tempted to import things from __main__ later, but that will cause
  problems: the code will get executed twice:

  - When you run `python -m sa_pathfinding` python will execute
    ``__main__.py`` as a script. That means there won't be any
    ``sa-pathfinding.__main__`` in ``sys.modules``.
  - When you import __main__ it will get executed again (as a module) because
    there's no ``sa-pathfinding.__main__`` in ``sys.modules``.

  Also see (1) from http://click.pocoo.org/5/setuptools/#setuptools-integration
"""
import inspect
from typing import Tuple
from pydoc import locate
import argparse
import time
import sys
import os

from sa_pathfinding.environments.towers_of_hanoi.towers_of_hanoi import TowersOfHanoi
from sa_pathfinding.algorithms.astar.grid_optimized_astar import GridOptimizedAstar
from sa_pathfinding.environments.towers_of_hanoi.towers_of_hanoi import TOHAction
from sa_pathfinding.environments.towers_of_hanoi.towers_of_hanoi import TOHState
from sa_pathfinding.algorithms.dijkstra.generic_dijkstra import GenericDijkstra
from sa_pathfinding.heuristics.grid_heuristic import OctileGridHeuristic
from sa_pathfinding.algorithms.astar.generic_astar import GenericAstar
from sa_pathfinding.algorithms.generics.search_node import SearchNode
from sa_pathfinding.environments.grids.generics.grid import GridState
from sa_pathfinding.environments.grids.octile_grid import OctileGrid
from sa_pathfinding.environments.generics.env import Environment
from sa_pathfinding.algorithms.bfs.generic_bfs import GenericBFS
from sa_pathfinding.algorithms.dfs.generic_dfs import GenericDFS
from sa_pathfinding.environments.generics.state import State
from sa_pathfinding.algorithms.generics.search import Search

verbose: bool = False
filepath = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))) + '/data/maps/large/brc202d.map')

def main(argv=sys.argv):
  print('running...')

  toh = TowersOfHanoi(4, 3)

  dij = GenericBFS(toh, start=SearchNode(toh.start), goal=SearchNode(toh.goal))
  dij.get_path()
  print(dij.history)

  args = _parse_args()
  _execute_args(args)
  return 0

def _parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser()
  parser.add_argument("-v", 
                      "--verbose", 
                      action='store_true', 
                      help="print supplementary info vs get results")

  # environment start_x start_y goal_x goal_y SearchClassName(HeuristicClassName) SearchClassName
  parser.add_argument("-sc", 
                    metavar='-',
                    nargs=7,
                    type=str,
                    help="""Compare wall clock running time of 2 search algorithms on a given environment. 
                          define string args as: 
                          env='EnvironmentClassName:filepath' 
                          start='EnvironmentStateClassName:param1, param2, ...' 
                          goal='EnvironmentStateClassName:param1, param2, ...'
                          search_1='SearchClassName:param1, param2, ...'
                          search_2='SearchClassName:param1, param2, ...'
                          usage: __main__.py -sc env start goal search_1 search_2""")
             
  return parser.parse_args()

def _execute_args(args) -> None:
  verbose = args.verbose

def speed_test(env: Environment, 
              search_1: Search.__class__, 
              search_2: Search.__class__, 
              start: State=None, 
              goal: State=None, 
              heuristic=None) -> Tuple[float, float]:

  if start is None:
    start = env.get_random(valid=True)
  
  if goal is None:
    goal = env.get_random(valid=True)
  
  

  generic_astar = GenericAstar(env, 
                              heuristic=OctileGridHeuristic(), 
                              start=SearchNode(start), 
                              goal=SearchNode(goal))
  gridopt_astar = GridOptimizedAstar(env, 
                                      heuristic=OctileGridHeuristic(), 
                                      start=SearchNode(start), 
                                      goal=SearchNode(goal))
  if verbose:
    print('\n-----Beginning Speeedtest-----\n')
    print(f'environment: {str(env)}')
    print(f'start: {str(start)}')
    print(f'goal: {str(goal)}')
    print(f'heuristic: {str(OctileGridHeuristic())}')
    print('\nrunning GenericAstar...')
  generic_t1 = time.time()
  generic_astar.get_path()
  generic_t2 = time.time()
  generic_time = generic_t2 - generic_t1
  if verbose:
    print(f'completed in: ' + str(round(generic_time, 2)) + 's')
    print('\nrunning GridOptimizedAstar...')
  gridopt_t1 = time.time()
  gridopt_astar.get_path()
  gridopt_t2 = time.time()
  gridopt_time = gridopt_t2 - gridopt_t1
  if verbose:
    print(f'completed in: ' + str(round(gridopt_time, 2)) + 's')
    print(f"\n{'GenericAstar' if generic_time < gridopt_time else 'GridOptimizedAstar'} was faster!")
  if not verbose:
    print(generic_time + ' ' + gridopt_time)