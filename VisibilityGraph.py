import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MplPolygon
from shapely.geometry import LineString, Polygon
import heapq

boundary = [(0, 0),
            (1085, 0),
            (1085, 745), 
            (0, 745)]

# runs Global Navigation
def run_global_nav(start, goal, raw_obstacles):
    obstacles = grow_obstacles(raw_obstacles, 115)
    nodes = np.array([start, goal] + [v for ob in obstacles for v in ob])
    neighbors = plot_path_options(nodes, boundary, obstacles)
    distance = map_distances(nodes)
    heuristic = calc_heuristic(nodes)
    path, cost, explored, ops, coords = path_search(nodes, neighbors, distance)

    plot_route(path, obstacles, start, goal, nodes, boundary)
    return path, coords

# adds buffer to obstacles
def grow_obstacles(obstacles, pad):
    grown = []
    for ob in obstacles:
        poly = Polygon(ob).buffer(pad, join_style="mitre", mitre_limit=10)
        grown.append(list(poly.exterior.coords)[:-1])   # drop the repeated closing point
    return grown

# determines if a path segment is valid 
def is_visible(a, b, obstacles, boundary):
    seg = LineString([a, b])
    polys = [Polygon(v) for v in obstacles]
    bounds = Polygon(boundary)

    if not bounds.contains(seg):
        return False
    
    for poly in polys:
        # blocked if the segment cuts through the interior;
        # touching a vertex or sliding along an edge is allowed
        if seg.crosses(poly) or seg.within(poly) or seg.crosses(bounds):
            return False
    return True

# calculates distance between nodes
def map_distances(nodes):
    num_nodes = nodes.shape[0]
    distance = np.zeros((num_nodes, num_nodes))

    for i in range(num_nodes):
        for j in range(num_nodes):
            if i != j:
                distance[i, j] = np.linalg.norm(nodes[i] - nodes[j], ord=2)
            else:
                distance[i, j] = 0
    return distance 

# calculates heuristic function for A*
def calc_heuristic(nodes):
    num_nodes = nodes.shape[0]
    heuristic = np.zeros(num_nodes)
    for i in range(num_nodes):
        heuristic[i] = np.linalg.norm(nodes[i] - nodes[2], ord=2)
    return heuristic

# finds optimal path using A*
def path_search(nodes, neighbors, distance, start=0, goal=1):
    nodes = np.asarray(nodes)
    goal_pos = nodes[goal]

    def h(i):
        return float(np.linalg.norm(nodes[i] - goal_pos))

    open_heap = [(h(start), start)]    # (f, node)
    came_from = {}
    g = {start: 0.0}
    closed = set()
    explored = []
    operation_count = 0

    while open_heap:
        f, current = heapq.heappop(open_heap)
        if current in closed:
            continue                    # stale heap entry
        closed.add(current)
        explored.append(current)
        operation_count += 1

        if current == goal:
            path = [current]
            path_coords = []
            while current in came_from:
                current = came_from[current]
                path.append(current)
            for i in range(operation_count-1):
                path_coords.append(nodes[path[i]])
                path_coords.reverse()
            return path[::-1], g[goal], explored, operation_count, path_coords

        for nb in neighbors[current]:
            if nb in closed:
                continue
            tentative = g[current] + distance[current][nb]
            if tentative < g.get(nb, float("inf")):
                g[nb] = tentative
                came_from[nb] = current
                heapq.heappush(open_heap, (tentative + h(nb), nb))

    return None, float("inf"), explored, operation_count   # no path found

# plots final path
def plot_route(path, obstacles, start, goal, nodes, boundary):
    plot_visibility_graph(obstacles, start, goal, boundary)
    print(len(path))
    for i in range(len(path) - 1):
        plt.plot([nodes[path[i], 0], nodes[path[i+1], 0]],
                     [nodes[path[i], 1], nodes[path[i+1], 1]],
                     color="tab:red", linewidth=0.5, alpha=0.5)

# plots all possible path segments      
def plot_path_options(nodes, boundary, obstacles):
    num_nodes = len(nodes)
    neighbors = [[] for _ in range(num_nodes)]

    for i in range(num_nodes):
        for j in range(i + 1, num_nodes):
            if is_visible(nodes[i, :], nodes[j, :], obstacles, boundary):
                neighbors[i].append(j)
                neighbors[j].append(i)
                plt.plot([nodes[i, 0], nodes[j, 0]],
                     [nodes[i, 1], nodes[j, 1]],
                     color="tab:blue", linewidth=0.5, alpha=0.5)
    return neighbors

#plots initial visibility graph
def plot_visibility_graph(obstacles, start, goal, boundary):
    fig, ax = plt.subplots(figsize=(7, 7))

    # 1. obstacles, straight from the vertex lists
    for verts in obstacles:
        ax.add_patch(MplPolygon(verts, closed=True,
                                facecolor="lightgray", edgecolor="black"))
        
    ax.add_patch(MplPolygon(boundary, closed=True, fill=False, edgecolor="black"))

    ax.scatter(*start, s=80, color="green", zorder=4, label="start")
    ax.scatter(*goal, s=80, color="orange", zorder=4, label="goal")
    

    ax.set_aspect("equal")        # keeps angles and lengths undistorted
    ax.autoscale_view()
    ax.legend()
    ax.set_xlabel("x"); ax.set_ylabel("y")