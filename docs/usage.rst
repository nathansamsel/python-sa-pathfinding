=====
Usage
=====

This package has a number of search algorithms and environments implemented already. In addition to these existing implementations, there are generic
definitions that provide a guide for how to define a custom algorithm or environment.

To use in a basic grid search::

	from sa_pathfinding.heuristics.grid_heuristic import OctileGridHeuristic
	from sa_pathfinding.algorithms.astar.generic_astar import GenericAstar
	from sa_pathfinding.environments.grids.generics.grid import GridState
	from sa_pathfinding.environments.grids.octile_grid import OctileGrid

	env = OctileGrid('path/to/file.map')

	# x and y location on grid
	start = GridState(25, 46)
	goal = GridState(123, 56)

	astar = GenericAstar(env, start=start, goal=goal, heuristic=OctileGridHeuristic())
	print(astar.get_path())

