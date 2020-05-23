from tkinter.filedialog import askopenfilename
from tkinter import ttk
import tkinter as tk
import math
import os

from sa_pathfinding.algorithms.dijkstra.grid_optimized_dijkstra import GridOptimizedDijkstra
from sa_pathfinding.algorithms.astar.grid_optimized_astar import GridOptimizedAstar
from sa_pathfinding.algorithms.dijkstra.generic_dijkstra import GenericDijkstra
from sa_pathfinding.environments.grids.octile_grid import StateNotValidError
from sa_pathfinding.heuristics.grid_heuristic import ManhattanGridHeuristic
from sa_pathfinding.heuristics.grid_heuristic import EuclideanGridHeuristic
from sa_pathfinding.heuristics.grid_heuristic import OctileGridHeuristic
from sa_pathfinding.algorithms.astar.generic_astar import GenericAstar
from sa_pathfinding.environments.grids.generics.grid import GridState
from sa_pathfinding.algorithms.generics.search_node import SearchNode
from sa_pathfinding.environments.grids.octile_grid import OctileGrid
from sa_pathfinding.algorithms.bfs.generic_bfs import GenericBFS
from sa_pathfinding.algorithms.dfs.generic_dfs import GenericDFS


class AppFrame(ttk.Frame):
    x_axis_padding: int = 35
    y_axis_padding: int = 35
    canvas_xmax: int = 800
    canvas_ymax: int = 800
    grid_col_width: int = 0
    grid_row_height: int = 0

    def __init__(self, app):
        # setting up the AppFrame frame

        ttk.Frame.__init__(self, app)
        self.pack(fill=None, expand=False)
        self.grid_rowconfigure(17)
        self.grid_columnconfigure(3)
        app.resizable(False, False)

        # Create UI elements

        self.drawing_frame = ttk.Frame(self, relief="sunken",
                                       width=self.canvas_xmax,
                                       height=self.canvas_xmax)
        self.canvas = tk.Canvas(self.drawing_frame, width=self.canvas_xmax,
                                height=self.canvas_ymax)
        self.canvas.bind("<Button-1>", app.canvas_click)

        self.label1 = ttk.Label(self, text="Map: den403d")
        self.button1 = ttk.Button(self, text="Choose Map",
                                  command=app.pressed_load_map)

        self.sep1 = ttk.Separator(self, orient=tk.HORIZONTAL)

        self.label2 = ttk.Label(self, text="Search Algorithm:")
        self.label3 = ttk.Label(self, text='Heuristic:')

        search_list = ['Generic A*', 'Grid Optimized A*', 'Generic Dijkstra', 'Grid Optomized Dijkstra', 'BFS', 'DFS']
        self.combo1 = ttk.Combobox(self, state='readonly', values=search_list)
        self.combo1.set(search_list[0])

        self.heuristic_list = ['Octile Distance', 'Manhattan Distance',
                               'Euclidean Distance']
        self.combo2 = ttk.Combobox(self, state='readonly',
                                   values=self.heuristic_list)
        self.combo2.set(self.heuristic_list[0])

        self.label4 = ttk.Label(self, text='Nodes Expanded: 0')
        self.label5 = ttk.Label(self, text='Open List Size: 0')

        self.sep2 = ttk.Separator(self, orient=tk.HORIZONTAL)

        start_x = tk.StringVar()
        start_y = tk.StringVar()
        self.label6 = ttk.Label(self, text='Start Cell: ')
        self.label7 = ttk.Label(self, text='Start X: ')
        self.label8 = ttk.Label(self, text='Start Y: ')
        self.entry1 = ttk.Entry(self, textvariable=start_x)
        self.entry2 = ttk.Entry(self, textvariable=start_y)
        self.button2 = ttk.Button(self, text="Choose Start Cell",
                                  command=app.pressed_choose_start)

        self.sep3 = ttk.Separator(self, orient=tk.HORIZONTAL)

        goal_x = tk.StringVar()
        goal_y = tk.StringVar()
        self.label9 = ttk.Label(self, text='Goal Cell: ')
        self.label10 = ttk.Label(self, text='Goal X: ')
        self.label11 = ttk.Label(self, text='Goal Y: ')
        self.entry3 = ttk.Entry(self, textvariable=goal_x)
        self.entry4 = ttk.Entry(self, textvariable=goal_y)
        self.button3 = ttk.Button(self, text="Choose Goal Cell",
                                  command=app.pressed_choose_goal)

        self.sep4 = ttk.Separator(self, orient=tk.HORIZONTAL)

        self.button4 = ttk.Button(self, text="Start", command=app.pressed_start)
        self.button5 = ttk.Button(self, text="Quit", command=app.pressed_quit)

        # Place UI elements into AppFrame Frame

        self.drawing_frame.grid(column=0, row=0, columnspan=3, rowspan=17)
        self.canvas.grid(column=0, row=0, columnspan=3, rowspan=17)
        self.label1.grid(column=3, row=0)
        self.button1.grid(column=4, row=0, sticky='w', padx=20)

        self.sep1.grid(column=3, row=1, columnspan=2, sticky='ew')

        self.label2.grid(column=3, row=2, sticky='w', padx=20)
        self.label3.grid(column=4, row=2, sticky='w', padx=20)
        self.combo1.grid(column=3, row=3, sticky='w', padx=20)
        self.combo2.grid(column=4, row=3, sticky='w', padx=20)
        self.label4.grid(column=3, row=4, sticky='w', padx=20)
        self.label5.grid(column=4, row=4, stick='w', padx=20)

        self.sep2.grid(column=3, row=5, columnspan=2, sticky='ew')

        self.label6.grid(column=3, row=6, sticky='w', padx=20)
        self.label7.grid(column=3, row=7, sticky='w', padx=20)
        self.entry1.grid(column=4, row=7, padx=20)
        self.label8.grid(column=3, row=8, sticky='w', padx=20)
        self.entry2.grid(column=4, row=8, padx=20)
        self.button2.grid(column=4, row=9, sticky='w', padx=20)

        self.sep3.grid(column=3, row=10, columnspan=2, sticky='ew')

        self.label9.grid(column=3, row=11, sticky='w', padx=20)
        self.label10.grid(column=3, row=12, sticky='w', padx=20)
        self.entry3.grid(column=4, row=12, padx=20)
        self.label11.grid(column=3, row=13, sticky='w', padx=20)
        self.entry4.grid(column=4, row=13, padx=20)
        self.button3.grid(column=4, row=14, sticky='w', padx=20)

        self.sep4.grid(column=3, row=15, columnspan=2, sticky='ew')

        self.button4.grid(column=3, row=16, sticky='ew', padx=20, pady=10)
        self.button5.grid(column=4, row=16, stick='ew', padx=20, pady=10)


class SearchVizApp(tk.Tk):
    # I don't want to deal with window sizing stuff for now
    canvas_width: int = 900
    canvas_height: int = 900

    def __init__(self, verbose: bool = False) -> None:
        tk.Tk.__init__(self)
        tk.Tk.wm_title(self, "Grid Search Visualization Application")
        self.verbose = verbose
        self.app_frame = AppFrame(self)
        self.map_file = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(__file__)))) + '/data/maps/small/den403d.map'
        self.env = OctileGrid(self.map_file)
        self.app_frame.canvas.config(width=self.canvas_width,
                                     height=self.canvas_height)
        self.choosing_start = False
        self.chose_start = False
        self.choosing_goal = False
        self.chose_goal = False
        self.start = None
        self.goal = None
        self.axis_labels = []
        self.search = None
        self._rects = []
        self.draw_initial()

    def canvas_click(self, event: tk.EventType) -> None:
        x = math.floor((event.x - self.app_frame.x_axis_padding) /
                       self.app_frame.grid_col_width)
        y = math.floor((event.y - self.app_frame.y_axis_padding) /
                       self.app_frame.grid_row_height)
        if self.choosing_start:
            self.start = SearchNode(GridState(x, y))
            self.chose_start = True
            self.choosing_start = False
            self.app_frame.entry1.delete(0, tk.END)
            self.app_frame.entry1.insert(0, str(self.start.state.x))
            self.app_frame.entry2.delete(0, tk.END)
            self.app_frame.entry2.insert(0, str(self.start.state.y))
        elif self.choosing_goal:
            self.goal = SearchNode(GridState(x, y))
            self.chose_goal = True
            self.choosing_goal = False
            self.app_frame.entry3.delete(0, tk.END)
            self.app_frame.entry3.insert(0, str(self.goal.state.x))
            self.app_frame.entry4.delete(0, tk.END)
            self.app_frame.entry4.insert(0, str(self.goal.state.y))
        else:
            print(f"clicked ({x}, {y})")
            return
        self.draw_initial()

    def draw_initial(self) -> None:
        self.app_frame.canvas.delete("all")
        self._rects = []
        self.app_frame.grid_col_width = (
            self.canvas_width - self.app_frame.x_axis_padding) / self.env.width
        self.app_frame.grid_row_height = (
            self.canvas_height - self.app_frame.y_axis_padding) / self.env.height
        for y in range(self.env.height):
            self._rects.append([])
            for x in range(self.env.width):
                color = ""
                if self.env.env[y][x].valid:
                    color = 'white'
                else:
                    color = 'black'
                self._rects[y].append(self.app_frame.canvas.create_rectangle(
                    x * self.app_frame.grid_col_width + self.app_frame.x_axis_padding,
                    y * self.app_frame.grid_row_height + self.app_frame.y_axis_padding,
                    x * self.app_frame.grid_col_width + self.app_frame.grid_col_width + self.app_frame.x_axis_padding,
                    y * self.app_frame.grid_row_height + self.app_frame.grid_row_height + self.app_frame.y_axis_padding,
                    fill=color))
        b = self.app_frame.canvas.create_rectangle(0,
                                                   self.app_frame.y_axis_padding - 20,
                                                   self.canvas_width,
                                                   self.app_frame.y_axis_padding,
                                                   fill='white', outline='')
        b = self.app_frame.canvas.create_rectangle(0,
                                                   0,
                                                   self.app_frame.x_axis_padding,
                                                   self.canvas_height,
                                                   fill='white', outline='')
        i = 0

        for x in range(0, self.env.width, 10):
            a = self.app_frame.canvas.create_text(
                x * self.app_frame.grid_col_width + self.app_frame.grid_col_width / 2 + self.app_frame.x_axis_padding,
                self.app_frame.y_axis_padding - 10,
                text=str(x))

        for y in range(0, self.env.height, 10):
            a = self.app_frame.canvas.create_text(
                self.app_frame.x_axis_padding / 2,
                i * self.app_frame.grid_row_height * 10 + self.app_frame.grid_row_height / 2 + self.app_frame.y_axis_padding,
                text=str(y))
            i += 1
        if self.start is not None:
            a = self.app_frame.canvas.create_rectangle(
                self.start.state.x * self.app_frame.grid_col_width + self.app_frame.x_axis_padding,
                self.start.state.y * self.app_frame.grid_row_height + self.app_frame.y_axis_padding,
                self.start.state.x * self.app_frame.grid_col_width + self.app_frame.grid_col_width + self.app_frame.x_axis_padding,
                self.start.state.y * self.app_frame.grid_row_height + self.app_frame.grid_row_height + self.app_frame.y_axis_padding,
                fill="yellow")
            a = self.app_frame.canvas.create_text((
                self.start.state.x * self.app_frame.grid_col_width + self.app_frame.grid_col_width / 2 + self.app_frame.x_axis_padding,
                self.start.state.y * self.app_frame.grid_row_height + self.app_frame.grid_row_height / 2 + self.app_frame.y_axis_padding),
                text="S")
        if self.goal is not None:
            a = self.app_frame.canvas.create_rectangle(
                self.goal.state.x * self.app_frame.grid_col_width + self.app_frame.x_axis_padding,
                self.goal.state.y * self.app_frame.grid_row_height + self.app_frame.y_axis_padding,
                self.goal.state.x * self.app_frame.grid_col_width + self.app_frame.grid_col_width + self.app_frame.x_axis_padding,
                self.goal.state.y * self.app_frame.grid_row_height + self.app_frame.grid_row_height + self.app_frame.y_axis_padding,
                fill="yellow")
            a = self.app_frame.canvas.create_text((
                self.goal.state.x * self.app_frame.grid_col_width + self.app_frame.grid_col_width / 2 + self.app_frame.x_axis_padding,
                self.goal.state.y * self.app_frame.grid_row_height + self.app_frame.grid_row_height / 2 + self.app_frame.y_axis_padding),
                text="G")

    def draw_step(self) -> None:
        try:
            node, open_list = next(self.search.step())
            self.app_frame.label4['text'] = 'Nodes Expanded: ' + str(self.search.nodes_expanded)
            self.app_frame.label5['text'] = 'Open List Size: ' + str(len(self.search.open))

            if node != self.search.start:
                self.app_frame.canvas.itemconfigure(self._rects[node.state.y][node.state.x], fill='red')
            for node in open_list:
                if node != self.search.goal:
                    self.app_frame.canvas.itemconfigure(self._rects[node.state.y][node.state.x], fill='green')
            self.after(1, self.draw_step)
        except StopIteration:
            if len(self.search.path) > 0:
                path = list(reversed(self.search.path))
                self.draw_path(path)
            else:
                if self.verbose:
                    print("Search Failed to return a path.")

    def draw_path(self, path: list) -> None:
        if len(path) == 0:
            return
        state = path.pop(0)
        if state != self.search.start.state and state != self.search.goal.state:
            self.app_frame.canvas.itemconfigure(self._rects[state.y][state.x], fill='blue')
        self.after(1, self.draw_path, path)

    def pressed_choose_goal(self) -> None:
        self.choosing_start = False
        self.choosing_goal = True

    def pressed_choose_start(self) -> None:
        self.choosing_goal = False
        self.choosing_start = True

    def pressed_load_map(self) -> None:
        self.map_file = askopenfilename(initialdir=os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(__file__)))) + '/data/maps')
        self._root().update()
        self.app_frame.label1['text'] = "Map: " + str(self.map_file).split("/")[-1]
        self.env = OctileGrid(self.map_file)
        for label in self.axis_labels:
            label.forget()
        self.draw_initial()

    def pressed_quit(self) -> None:
        if self.verbose:
            print("Quit button pressed. Exiting application.")
        exit(1)

    def pressed_start(self) -> None:
        if self.start is None or self.goal is None:
            raise Exception('Search started without start / goal')
        search = 'gas'
        if self.app_frame.combo1.get() == 'Generic Dijkstra':
            search = 'gd'
        elif self.app_frame.combo1.get() == 'Grid Optimized Dijkstra':
            search = 'god'
        elif self.app_frame.combo1.get() == 'Grid Optimized A*':
            search = 'goas'
        elif self.app_frame.combo1.get() == 'BFS':
            search = 'bfs'
        elif self.app_frame.combo1.get() == 'DFS':
            search = 'dfs'
        
        heuristic = OctileGridHeuristic()
        if self.app_frame.combo2.get() == 'Manhattan Distance':
            heuristic = ManhattanGridHeuristic()
        elif self.app_frame.combo2.get() == 'Euclidean Distance':
            heuristic = EuclideanGridHeuristic()
        if not self.env.is_valid(self.start.state):
            raise StateNotValidError(self.start.state)
        else:
            self.start.state._valid = True
        if not self.env.is_valid(self.goal.state):
            raise StateNotValidError(self.goal.state)
        else:
            self.goal.state._valid = True
        if search == 'gas':
            self.search = GenericAstar(self.env,
                                             start=self.start,
                                             goal=self.goal,
                                             heuristic=heuristic,
                                             verbose=self.verbose)
        elif search == 'gd':
            self.search = GenericDijkstra(self.env,
                                   start=self.start,
                                   goal=self.goal,
                                   verbose=self.verbose)
        elif search == 'god':
            self.search = GridOptimizedDijkstra(self.env,
                                   start=self.start,
                                   goal=self.goal,
                                   verbose=self.verbose)
        elif search == 'goas':
            self.search = GridOptimizedAstar(self.env,
                                   start=self.start,
                                   goal=self.goal,
                                   heuristic=heuristic,
                                   verbose=self.verbose)
        elif search == 'bfs':
            self.search = GenericBFS(self.env,
                                   start=self.start,
                                   goal=self.goal,
                                   verbose=self.verbose)
        elif search == 'dfs':
            self.search = GenericDFS(self.env,
                                   start=self.start,
                                   goal=self.goal,
                                   verbose=self.verbose)
        self.draw_initial()
        self.after(1000, self.draw_step)

    def run(self) -> None:
        self.mainloop()


if __name__ == '__main__':
    app = SearchVizApp(verbose=False)
    app.run()
