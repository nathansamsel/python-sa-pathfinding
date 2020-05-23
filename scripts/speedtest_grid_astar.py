import time
import os

from sa_pathfinding.environments.grids.octile_grid import OctileGrid
from sa_pathfinding.algorithms.generics.search_node import SearchNode
from sa_pathfinding.heuristics.grid_heuristic import OctileGridHeuristic
from sa_pathfinding.environments.grids.generics.grid_state import GridState
from sa_pathfinding.algorithms.astar.generic_astar import GenericAstar
from sa_pathfinding.algorithms.astar.grid_optimized_astar import GridOptimizedAstar

def speed_diff():
    filepath = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))) + '/data/maps/large/brc202d.map')
    environment = OctileGrid(filepath)
    start = environment.get_random(valid=True)
    goal = GridState(474, 426)
    generic_astar = GenericAstar(environment, 
                                heuristic=OctileGridHeuristic(), 
                                start=SearchNode(start), 
                                goal=SearchNode(goal),
                                verbose=True)
    gridopt_astar = GridOptimizedAstar(environment, 
                                        heuristic=OctileGridHeuristic(), 
                                        start=SearchNode(start), 
                                        goal=SearchNode(goal))
    print('\n-----Beginning Speeedtest-----\n')
    print(f'environment: {str(environment)}')
    print(f'start: {str(start)}')
    print(f'goal: {str(goal)}')
    print(f'heuristic: {str(OctileGridHeuristic())}')
    print('\nrunning GenericAstar...')
    generic_t1 = time.time()
    generic_astar.get_path()
    generic_t2 = time.time()
    generic_time = generic_t2 - generic_t1
    print(f'completed in: ' + str(round(generic_time, 2)) + 's')
    print('\nrunning GridOptimizedAstar...')
    gridopt_t1 = time.time()
    gridopt_astar.get_path()
    gridopt_t2 = time.time()
    gridopt_time = gridopt_t2 - gridopt_t1
    print(f'completed in: ' + str(round(gridopt_time, 2)) + 's')

    print(f"\n{'GenericAstar' if generic_time < gridopt_time else 'GridOptimizedAstar'} was faster!")

if __name__ == '__main__':
    speed_diff()