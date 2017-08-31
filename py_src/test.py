import maxflow
g = maxflow.Graph[int](2, 2)
g.add_nodes(2)
g.add_edge(0, 1, 1, 2)
g.add_tedge(0, 2, 5)
g.add_tedge(1, 9, 4)
g.maxflow()

g.get_segments()
array([ True, False], dtype=bool)
