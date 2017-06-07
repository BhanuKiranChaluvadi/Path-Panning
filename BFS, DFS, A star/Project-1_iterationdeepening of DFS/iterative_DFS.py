'''
import iterative_graph_search
g = iterative_graph_search.GridMap('./map0.txt')
file = open('output0.txt', 'a')
iterations = 8
[[path, action_path],visited]=iterative_graph_search.dfs(g.init_pos , g.transition , g.is_goal, iterative_graph_search._ACTIONS,iterations)
file.write("BFS on map0: path        = "+str(path) + "\n")
file.write("BFS on map0: action_path = "+str(action_path) + "\n")
file.write("BFS on map0: visited     = "+str(visited) + "\n")
print(visited)
print(path)
print(action_path)
g.display_map(path,visited)
file.close()
'''
import iterative_graph_search
g = iterative_graph_search.GridMap('./map1.txt')
file = open('output1.txt', 'a')
iterations = 15
[[path, action_path],visited]=iterative_graph_search.dfs(g.init_pos , g.transition , g.is_goal, iterative_graph_search._ACTIONS,iterations)
file.write("BFS on map1: path        = "+str(path) + "\n")
file.write("BFS on map1: action_path = "+str(action_path) + "\n")
file.write("BFS on map1: visited     = "+str(visited) + "\n")
print(visited)
print(path)
print(action_path)
g.display_map(path,visited)
file.close()


import iterative_graph_search
g = iterative_graph_search.GridMap('./map2.txt')
file = open('output2.txt', 'a')
iterations = 8
[[path, action_path],visited]=iterative_graph_search.dfs(g.init_pos , g.transition , g.is_goal, iterative_graph_search._ACTIONS,iterations)
file.write("BFS on map2: path        = "+str(path) + "\n")
file.write("BFS on map2: action_path = "+str(action_path) + "\n")
file.write("BFS on map2: visited     = "+str(visited) + "\n")
print(visited)
print(path)
print(action_path)
g.display_map(path,visited)
file.close()
