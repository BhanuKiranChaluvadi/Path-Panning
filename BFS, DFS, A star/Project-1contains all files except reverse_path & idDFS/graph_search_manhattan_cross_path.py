#!/usr/bin/env python
'''
Package providing helper classes and functions for performing graph search operations for planning.
'''
import numpy as np
import heapq
import matplotlib.pyplot as plotter
from collections import deque
from math import hypot

_DEBUG = False
_DEBUG_END = True
_ACTIONS = ['u','d','l','r']
_ACTIONS_2 = ['u','d','l','r','ne','nw','sw','se']
_X = 0
_Y = 1
_GOAL_COLOR = 0.75
_INIT_COLOR = 0.25
_PATH_COLOR_RANGE = _GOAL_COLOR-_INIT_COLOR
_VISITED_COLOR = 0.9


class GridMap:
    '''
    Class to hold a grid map for navigation. Reads in a map.txt file of the format
    0 - free cell, x - occupied cell, g - goal location, i - initial location.
    Additionally provides a simple transition model for grid maps and a convience function
    for displaying maps.
    '''
    def __init__(self, map_path=None):
        '''
        Constructor. Makes the necessary class variables. Optionally reads in a provided map
        file given by map_path.

        map_path (optional) - a string of the path to the file on disk
        '''
        self.rows = None
        self.cols = None
        self.goal = None
        self.init_pos = None
        self.occupancy_grid = None
        if map_path is not None:
            self.read_map(map_path)

    def read_map(self, map_path):
        '''
        Read in a specified map file of the format described in the class doc string.

        map_path - a string of the path to the file on disk
        '''
        map_file = file(map_path,'r')
        lines = [l.rstrip().lower() for l in map_file.readlines()]
        map_file.close()
        self.rows = len(lines)
        self.cols = max([len(l) for l in lines])
        if _DEBUG:
            print 'rows', self.rows
            print 'cols', self.cols
            print lines
        self.occupancy_grid = np.zeros((self.rows, self.cols), dtype=np.bool)
        for r in xrange(self.rows):
            for c in xrange(self.cols):
                if lines[r][c] == 'x':
                    self.occupancy_grid[r][c] = True
                if lines[r][c] == 'g':
                    self.goal = (r,c)
                elif lines[r][c] == 'i':
                    self.init_pos = (r,c)

    def is_goal(self,s):
        '''
        Test if a specifid state is the goal state

        s - tuple describing the state as (row, col) position on the grid.

        Returns - True if s is the goal. False otherwise.
        '''
        return (s[_X] == self.goal[_X] and
                s[_Y] == self.goal[_Y])

    def transition(self, s, a):
        '''
        Transition function for the current grid map.

        s - tuple describing the state as (row, col) position on the grid.
        a - the action to be performed from state s

        returns - s_prime, the state transitioned to by taking action a in state s.
        If the action is not valid (e.g. moves off the grid or into an obstacle)
        returns the current state.
        '''
        new_pos = list(s[:])
        # Ensure action stays on the board
        if a == 'u':
            if s[_X] > 0:
                new_pos[_X] -= 1
        elif a == 'd':
            if s[_X] < self.rows - 1:
                new_pos[_X] += 1
        elif a == 'l':
            if s[_Y] > 0:
                new_pos[_Y] -= 1
        elif a == 'r':
            if s[_Y] < self.cols - 1:
                new_pos[_Y] += 1
        elif a == 'ne':
            if ( (s[_X] > 0) and (s[_Y] < self.cols - 1)):
                new_pos[_X] -= 1
                new_pos[_Y] += 1
        elif a == 'nw':
            if ( (s[_X] > 0) and (s[_Y] > 0)):
                new_pos[_X] -= 1
                new_pos[_Y] -= 1
        elif a == 'sw':
            if ( (s[_X] < self.rows - 1) and (s[_Y] > 0)):
                new_pos[_X] += 1
                new_pos[_Y] -= 1
        elif a == 'se':
            if ( (s[_X] < self.rows - 1) and (s[_Y] < self.cols -1 )):
                new_pos[_X] += 1
                new_pos[_Y] += 1
        else:
            print 'Unknown action:', str(a)

        # Test if new position is clear
        if self.occupancy_grid[new_pos[0], new_pos[1]]:
            s_prime = tuple(s)
        else:
            s_prime = tuple(new_pos)
        return s_prime

    def display_map(self, path=[], visited={},name ="default"):
        '''
        Visualize the map read in. Optionally display the resulting plan and visisted nodes

        path - a list of tuples describing the path take from init to goal
        visited - a set of tuples describing the states visited during a search
        '''
        display_grid = np.array(self.occupancy_grid, dtype=np.float32)

        # Color all visited nodes if requested
        for v in visited:
            display_grid[v] = _VISITED_COLOR
        # Color path in increasing color from init to goal
        for i, p in enumerate(path):
            disp_col = _INIT_COLOR + _PATH_COLOR_RANGE*(i+1)/len(path)
            display_grid[p] = disp_col

        display_grid[self.init_pos] = _INIT_COLOR
        display_grid[self.goal] = _GOAL_COLOR

        # Plot display grid for visualization
        imgplot = plotter.imshow(display_grid)
        # Set interpolation to nearest to create sharp boundaries
        imgplot.set_interpolation('nearest')
        # Set color map to diverging style for contrast
        imgplot.set_cmap('spectral')
        plotter.savefig(name, ext='png')
        #plotter.show()

    def uninformed_heuristic(self, s):
        '''
        Example of how a heuristic may be provided. This one is admissable, but dumb.

        s - tuple describing the state as (row, col) position on the grid.

        returns - floating point estimate of the cost to the goal from state s
        '''
        return 0.0


class SearchNode:
    def __init__(self, s, actions, parent=None, parent_action=None, cost=0,g_value=0):
        '''
        s - the state defining the search node
        A - list of actions
        parent - the parent search node
        parent_action - the action taken from parent to get to s
        '''
        self.parent = parent
        self.cost = cost
        self.parent_action = parent_action
        self.state = s[:]
        self.actions = actions[:]
        self.g_cost =  g_value

    def __str__(self):
        '''
        Return a human readable description of the node
        '''
        return str(self.state) + ' ' + str(self.actions)+' '+str(self.parent)+' '+str(self.parent_action)

class PriorityQ:
    '''
    Priority queue implementation with quick access for membership testing
    Setup currently to only with the SearchNode class
    '''
    def __init__(self):
        '''
        Initialize an empty priority queue
        '''
        self.l = [] # list storing the priority q
        self.s = set() # set for fast membership testing

    def __contains__(self, x):
        '''
        Test if x is in the queue
        '''
        return x in self.s

    def push(self, x, cost):
        '''
        Adds an element to the priority queue.
        If the state already exists, we update the cost
        '''
        if x.state in self.s:
            for y in self.l:
                if ((x.state == y[1].state) and (x.cost < y[1].cost)):
                    return self.replace(x,y, cost)
        else:
            heapq.heappush(self.l, (cost, x))
            self.s.add(x.state)

    def pop(self):
        '''
        Get the value and remove the lowest cost element from the queue
        '''
        x = heapq.heappop(self.l)
        self.s.remove(x[1].state)
        return x[1]

    def peak(self):
        '''
        Get the value of the lowest cost element in the priority queue
        '''
        x = self.l[0]
        return x[1]

    def __len__(self):
        '''
        Return the number of elements in the queue
        '''
        return len(self.l)

    def replace(self, x,y, new_cost):
        '''
        Removes element x from the q and replaces it with x with the new_cost
        '''
        self.l.remove(y)
        self.s.remove(y[1].state)
        heapq.heapify(self.l)
        self.push(x, new_cost)

    def get_cost(self, x):
        '''
        Return the cost for the search node with state x.state
        '''
        for y in self.l:
            if x.state == y[1].state:
                return y[0]

    def __str__(self):
        '''
        Return a string of the contents of the list
        '''
        return str(self.l)

def dfs(init_state, f, is_goal, actions):
    stack_of_suspended_vertices = []
    n0 = SearchNode (init_state,actions)
    visited = []
    stack_of_suspended_vertices.append(n0)
    loop_count = 0
    while len(stack_of_suspended_vertices)> 0:
        n_i= stack_of_suspended_vertices.pop()
        if n_i.state not in visited:
            visited.append(n_i.state)
            if is_goal(n_i.state):
                return (backpath(n_i,init_state),visited)
            else:
                for a in actions:
                    s_prime =f(n_i.state,a)
                    if(s_prime != n_i.state):
                        n_prime= SearchNode(s_prime,actions,n_i,a)
                        stack_of_suspended_vertices.append(n_prime)
    print("Path from inti_state to goal is not possible")
    return (backpath(n_i,init_state),visited)

def bfs(init_state, f, is_goal, actions):
    stack_of_suspended_vertices = deque([])
    n0 = SearchNode (init_state,actions)
    visited = []
    stack_of_suspended_vertices.append(n0)
    loop_count = 0
    while len(stack_of_suspended_vertices)> 0:
        n_i= stack_of_suspended_vertices.popleft()
        if n_i.state not in visited:
            visited.append(n_i.state)
            if is_goal(n_i.state):
                return (backpath(n_i,init_state),visited)
            else:
                for a in actions:
                    s_prime =f(n_i.state,a)
                    if(s_prime != n_i.state):
                        n_prime= SearchNode(s_prime,actions,n_i,a)
                        stack_of_suspended_vertices.append(n_prime)
    print("Path from inti_state to goal is not possible")
    return (backpath(n_i,init_state),visited)

def uniform_cost_search(init_state, f, is_goal, actions):
    visited = []
    que = PriorityQ() #creates a list l=[] and emptyset s=set()
    cost = 0
    n0 = SearchNode (init_state,actions,[],[],cost)
    que.push(n0,cost)
    loop_count = 0
    while len(que.l)> 0:
        n_i= que.pop()
        if n_i.state not in visited:
            visited.append(n_i.state)
            if is_goal(n_i.state):
                return (backpath(n_i,init_state),visited)
            else:
                for a in actions:
                    s_prime =f(n_i.state,a)
                    if(s_prime != n_i.state):
                        cost = (n_i.cost+1)
                        n_prime= SearchNode(s_prime,actions,n_i,a,cost)
                        que.push(n_prime,cost)
    print("Path from inti_state to goal is not possible")
    return (backpath(n_i,init_state),visited)


def a_star_search(init_state,goal_state, f, is_goal, actions):
    visited = []
    que = PriorityQ() #creates a list l=[] and emptyset s=set()
    cost = 0
    n0 = SearchNode (init_state,actions,[],[],cost,g_value =0)
    que.push(n0,cost)
    loop_count = 0
    while len(que.l)> 0:
        n_i= que.pop()
        if n_i.state not in visited:
            visited.append(n_i.state)
            if is_goal(n_i.state):
                return (backpath(n_i,init_state),visited)
            else:
                for a in actions:
                    s_prime =f(n_i.state,a)
                    if(s_prime != n_i.state):
                        if ( a=='u'or a== 'd'or a=='l'or a=='r'):
                            g_value = n_i.g_cost + 1
                        else:
                            g_value = n_i.g_cost + 1.5
                        dx =  abs(goal_state[0]-s_prime[0])
                        dy= abs(goal_state[1]-s_prime[1])
                        h_value = dx + dy
                        f_cost = g_value + h_value
                        n_prime= SearchNode(s_prime,actions,n_i,a,f_cost ,g_value)
                        que.push(n_prime,f_cost)

    print("Path from inti_state to goal is not possible")
    return (backpath(n_i,init_state),visited)
    '''
    init_state - value of the initial state
    f - transition function takes input state (s), action (a), returns s_prime = f(s, a)
        returns s if action is not valid
    is_goal - takes state as input returns true if it is a goal state
        actions - list of actions available
    h - heuristic function, takes input s and returns estimated cost to goal
        (note h will also need access to the map, so should be a member function of GridMap)
    '''


    return None

def backpath(node,init_pos):
    '''
    Function to determine the path that lead to the specified search node

    node - the SearchNode that is the end of the path

    returns - a tuple containing (path, action_path) which are lists respectively of the states
    visited from init to goal (inclusive) and the actions taken to make those transitions.
    '''
    path = []
    action_path = []
    n_0 =node
    path.append(n_0.state)

    while (n_0.state != init_pos):
        path.append( n_0.parent.state)
        action_path.append((n_0.parent_action))
        n_0 = n_0.parent
    # I filled U!!!!
    path = path[::-1]
    action_path = action_path[::-1]
    return (path, action_path)


