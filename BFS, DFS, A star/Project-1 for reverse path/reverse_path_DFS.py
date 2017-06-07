import graph_search
g = graph_search.GridMap('./map0.txt')
file = open('output0.txt', 'a')
[[path, action_path],visited]=graph_search.dfs(g.init_pos , g.transition , g.is_goal, graph_search._ACTIONS)
file.write("BFS on map0: path        = "+str(path) + "\n")
file.write("BFS on map0: action_path = "+str(action_path) + "\n")
file.write("BFS on map0: visited     = "+str(visited) + "\n")
g.display_map(path,visited)
file.close()


import graph_search
g = graph_search.GridMap('./map1.txt')
file = open('output1.txt', 'a')
[[path, action_path],visited]=graph_search.dfs(g.init_pos , g.transition , g.is_goal, graph_search._ACTIONS)
file.write("BFS on map1: path        = "+str(path) + "\n")
file.write("BFS on map1: action_path = "+str(action_path) + "\n")
file.write("BFS on map2: visited     = "+str(visited) + "\n")
g.display_map(path,visited)
file.close()


import graph_search
g = graph_search.GridMap('./map2.txt')
file = open('output2.txt', 'a')
[[path, action_path],visited]=graph_search.dfs(g.init_pos , g.transition , g.is_goal, graph_search._ACTIONS)
file.write("BFS on map2: path        = "+str(path) + "\n")
file.write("BFS on map2: action_path = "+str(action_path) + "\n")
file.write("BFS on map2: visited     = "+str(visited) + "\n")
g.display_map(path,visited)
file.close()
