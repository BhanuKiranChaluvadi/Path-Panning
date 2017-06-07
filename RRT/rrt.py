#!/usr/bin/env python
'''
Package providing helper classes and functions for performing graph search operations for planning.
'''
import numpy as np
import matplotlib.pyplot as plotter
from math import pi,sqrt,cos,sin,atan2
from collisions import PolygonEnvironment
import time
import random
from operator import sub

_DEBUG = False

_TRAPPED = 'trapped'
_ADVANCED = 'advanced'
_REACHED = 'reached'

class TreeNode:
    def __init__(self, state, parent=None):
        self.state = state
        self.children = []
        self.parent = parent

    def add_child(self, child):
        self.children.append(child)

class RRTSearchTree:
    def __init__(self, init):
        self.root = TreeNode(init)
        self.nodes = [self.root]
        self.edges = []

    def find_nearest(self, s_query):
        min_d = 1000000
        nn = self.root
        for n_i in self.nodes:
            d = np.linalg.norm(s_query - n_i.state)
            if d < min_d:
                nn = n_i
                min_d = d
        return (nn, min_d)

    def add_node(self, node, parent):
        self.nodes.append(node)
        self.edges.append((parent.state, node.state))
        node.parent = parent
        parent.add_child(node)

    def get_states_and_edges(self):
        states = np.array([n.state for n in self.nodes])
        return (states, self.edges)

    def get_back_path(self, n):
        path = []
        while n.parent is not None:
            path.append(n.state)
            n = n.parent
        path.reverse()
        return path

class RRT:

    def __init__(self, num_samples, num_dimensions=2, step_length = .15, lims = None,
                 connect_prob = 0.1, collision_func=None):
        self.K = num_samples
        self.n = num_dimensions
        self.epsilon = step_length
        self.connect_prob = connect_prob

        self.in_collision = collision_func
        if collision_func is None:
            self.in_collision = self.fake_in_collision

        # Setup range limits
        self.limits = lims
        if self.limits is None:
            self.limits = []
            for n in xrange(num_dimensions):
                self.limits.append([0,100])
            self.limits = np.array(self.limits)

        self.ranges = self.limits[:,1] - self.limits[:,0]
        self.found_path = False

    def build_rrt(self, init, goal):
        '''
        Build the rrt from init to goal
        Returns path to goal or None
        '''
        self.goal = np.array(goal)
        self.init = np.array(init)
        self.found_path = False

        # Build tree and search
        self.T = RRTSearchTree(init)
        for each in range(0,self.K):
            random_config = self.sample()
            self.extend(random_config)
            last_node = self.T.nodes[-1]
            if((np.linalg.norm(last_node.state - self.goal))<= self.epsilon):
                self.extend(self.goal)
                print "Reached"
                break
        Final_Path=self.T.get_back_path(self.T.nodes[-1])
        #print Final_Path
        return Final_Path

    def build_rrt_connect(self, init, goal):
        '''
        Build the rrt connect from init to goal
        Returns path to goal or None
        '''
        self.goal = np.array(goal)
        self.init = np.array(init)
        self.found_path = False

        # Build tree and search
        self.T = RRTSearchTree(init)
        for each in range(0,self.K):
            random_config = self.sample()
            if(self.extend_connect(random_config) == 'reached'):
                print 'Reached'
                break
            else:
                continue
        Final_Path=self.T.get_back_path(self.T.nodes[-1])
        #print Final_Path
        return Final_Path

    def build_rrt_bi_connect(self, init, goal):
        self.goal = np.array(goal)
        self.init = np.array(init)
        self.found_path = False

        self.Ta = RRTSearchTree(init)
        self.Tb = RRTSearchTree(goal)
        self.T = RRTSearchTree(init)
        for each in range(0,self.K):
            random_config = self.sample()
            self.extend_bi_connect(self.Ta, random_config)
            self.extend_bi_connect(self.Tb, self.Ta.nodes[-1].state)
            if np.array_equal(self.Ta.nodes[-1].state, self.Tb.nodes[-1].state):
                self.found_path = True
                break
            else:
                self.Ta, self.Tb = self.Tb, self.Ta
        if(self.found_path):
            Ta_path = self.Ta.get_back_path(self.Ta.nodes[-1])
            Tb_path = self.Tb.get_back_path(self.Tb.nodes[-1])
            Tb_path.pop()
            Tb_path.reverse()
            final_path = Ta_path + Tb_path
            if((final_path[0][0]+final_path[0][1])>10):
                final_path.reverse()
            self.T = self.Ta
            self.T.edges = self.T.edges + self.Tb.edges
            self.T.nodes = self.T.nodes + self.Tb.nodes
            return final_path

        else:
            return 'not connected'

    def sample(self):
        random_number = random.random()
        if(random_number <= self.connect_prob):
            return self.goal
        else:
            new_configuration = []
            for iteam in range (0,self.n,1):
                new_configuration.append(random.uniform(self.limits[iteam][0],self.limits[iteam][1]))
            return np.array(new_configuration)

    def extend(self, q):
        (nearestnode,dist) = self.T.find_nearest(q)
        new_config = self.new_config_generator(nearestnode.state , q, dist)   # new_config is (x_, y_) not a node
        if self.in_collision(new_config) == False:
            new_node = TreeNode(new_config)
            self.T.add_node(new_node,nearestnode)
        return None

    def extend_connect(self, q):
        (nearestnode,dist) = self.T.find_nearest(q)
        while(dist > self.epsilon):
            new_config = self.new_config_generator(nearestnode.state , q, dist)
            if self.in_collision(new_config) == False:
                new_tree_node = TreeNode(new_config)
                self.T.add_node(new_tree_node,nearestnode)
                if((np.linalg.norm(new_tree_node.state - self.goal))<= self.epsilon):
                    new_tree_node = TreeNode(self.goal)
                    self.T.add_node(new_tree_node,nearestnode)
                    return _REACHED
                else:
                    nearestnode = new_tree_node
                    dist = np.linalg.norm(new_tree_node.state - q)
            else:
                return _ADVANCED
        if(dist <= self.epsilon):
            if self.in_collision(q) == False:
                new_tree_node = TreeNode(q)
                self.T.add_node(new_tree_node,nearestnode)
                if((np.linalg.norm(new_tree_node.state - self.goal))<= self.epsilon):
                    new_tree_node = TreeNode(self.goal)
                    self.T.add_node(new_tree_node,nearestnode)
                    return _REACHED
            return  _ADVANCED


    def extend_bi_connect(self, Tree, q):
        (nearestnode,dist) = Tree.find_nearest(q)
        while(dist > self.epsilon):
            new_config = self.new_config_generator(nearestnode.state , q, dist)
            if self.in_collision(new_config) == False:
                new_tree_node = TreeNode(new_config)
                Tree.add_node(new_tree_node,nearestnode)
                nearestnode = new_tree_node
                dist = np.linalg.norm(new_tree_node.state - q)
            else:
                break
        if(dist <= self.epsilon):
            if self.in_collision(q) == False:
                new_tree_node = TreeNode(q)
                Tree.add_node(new_tree_node,nearestnode)
                return _REACHED
        return  _ADVANCED


    def new_config_generator(self, p1,p2,distance):
        if distance < self.epsilon:
            return p2
        else:
            '''
            theta = atan2(p2[1]-p1[1],p2[0]-p1[0])
            return p1[0] + (self.epsilon)*cos(theta), p1[1] + (self.epsilon)*sin(theta)
            '''
            value = p1+ self.epsilon*((p2-p1)/distance)   #vector manipulation.
            return value


    def fake_in_collision(self, q):
        '''
        We never collide with this function!
        '''
        return False

def test_rrt_env(num_samples=20, step_length=.15, env='./env1.txt', connect= 2):
    pe = PolygonEnvironment()
    pe.read_env(env)

    dims = len(pe.start)
    start_time = time.time()

    rrt = RRT(num_samples,
              dims,
              step_length,
              lims = pe.lims,
              connect_prob = 0.05,
              collision_func=pe.test_collisions)
    if connect==1:
        plan = rrt.build_rrt_connect(pe.start, pe.goal)
    elif connect==0:
        plan = rrt.build_rrt(pe.start, pe.goal)
    elif connect ==2:
        plan = rrt.build_rrt_bi_connect(pe.start, pe.goal)

    run_time = time.time() - start_time
    print 'plan:', plan
    print 'run_time =', run_time
    pe.draw_plan(plan, rrt)
    plotter.pause(10000)
    return plan, rrt