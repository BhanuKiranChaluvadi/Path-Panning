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
import copy

_DEBUG = False

_TRAPPED = 'trapped'
_ADVANCED = 'advanced'
_REACHED = 'reached'

class TreeNode:
    def __init__(self, state):
        self.vertex = state
        self.edges = []

class RRTSearchTree:

    def __init__(self,init):
        self.root = TreeNode(init)
        self.nodes = []
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

class PRM:

    def __init__(self, num_samples, num_dimensions=2, step_length = .15, lims = None,
                 connect_prob = 0.1, collision_func=None, no_closest_neighbours = 3):
        self.vertices = []
        self.edges= []
        self.Nq = no_closest_neighbours
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


    def build_prm(self, init, goal):

        self.goal = np.array(goal)
        self.init = np.array(init)
        self.found_path = False

        # Build tree and search


        while len(self.vertices) < self.K:
            rand_config = self.random_configuration()
            if(self.in_collision(rand_config) == False):
                self.vertices.append(rand_config)

        self.T = RRTSearchTree()
        for index in range(0, len(self.vertices),1):
            each =self.vertices[index]
            nearest_neighbours = self.find_nearest(index)
            for nn in nearest_neighbours:
                if(self.is_collision_freepath(each,nn)):
                    #edge_formed =(self.vertices[index],nn)]
                    #if(edge_formed not in self.edges):
                    self.edges.append((each,nn))

        self.T = RRTSearchTree()
        self.T.nodes = self.vertices
        self.T.edges = self.edges
        return None

    def random_configuration(self):
        new_configuration = []
        for iteam in range (0,self.n,1):
            new_configuration.append(random.uniform(self.limits[iteam][0],self.limits[iteam][1]))
        return np.array(new_configuration)


    def find_nearest(self,index1):
        vertex_1 = self.vertices[index1]
        list_nearest_neighbours=[]
        nn = self.vertices[index1]+1000
        dummy_vertices = copy.deepcopy(self.vertices)
        del dummy_vertices[index1]
        for x in range(0,self.Nq,1):
            min_d = 1000000
            removing_index=0
            for index_2 in range(0,len(dummy_vertices),1):
                d = np.linalg.norm(vertex_1 - dummy_vertices[index_2])
                if d < min_d:
                    nn = dummy_vertices[index_2]
                    removing_index = index_2
                    min_d = d
            list_nearest_neighbours.append(nn)
            del dummy_vertices[removing_index]
        return np.array(list_nearest_neighbours)

    def is_collision_freepath(self,q1,q2):
        distance = np.linalg.norm(q1 - q2)
        while(distance > self.epsilon):
            new_config = self.directed_random_configuration(q1 , q2, distance)
            if self.in_collision(new_config):
                return False
            else:
                q1 = new_config
                distance = np.linalg.norm(q1-q2)
        if(distance <= self.epsilon):
            return  True




    def directed_random_configuration(self,q1,q2,distance):
        value = q1- self.epsilon*((q1-q2)/distance)   #vector manipulation.
        return value



def test_rrt_env(num_samples=10, step_length=2, env='./env0.txt', connect= 2):
    pe = PolygonEnvironment()
    pe.read_env(env)

    dims = len(pe.start)
    start_time = time.time()

    prm = PRM(num_samples,
              dims,
              step_length,
              lims = pe.lims,
              connect_prob = 0.1,
              collision_func=pe.test_collisions)

    plan = prm.build_prm(pe.start, pe.goal)
    run_time = time.time() - start_time
    print 'plan:', plan
    print 'run_time =', run_time
    pe.draw_plan(None, prm)
    return plan, prm