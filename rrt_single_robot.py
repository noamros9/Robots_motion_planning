from arr2_epec_cs_ex import *
import random
import networkx as nx
import time
import Collision_detection
import matplotlib.pyplot as plt
import math
import conversions


class Dynamic_kd_tree:
    tree = None
    buff = []
    nn = None
    eps = FT(Gmpq(0.0))  # 0.0 for exact NN, otherwise approximate NN
    search_nearest = True  # set this value to False in order to search farthest
    sort_neighbors = False  # set this value to True in order to obtain the neighbors sorted by distance
    ed = None
    def __init__(self):
        self.tree = Kd_tree()
        self.buff = []
        self.ed = Euclidean_distance()
    def insert(self, p):
        self.buff.append(p)
        if(len(self.buff) > 100):
            self.tree.insert(self.buff)
            self.buff.clear()
    def nearest_neighbor(self, p):
        k = 1
        nn = K_neighbor_search(self.tree, p, k, self.eps, self.search_nearest, self.ed, self.sort_neighbors)
        dist_from_p = lambda point: self.ed.transformed_distance(p, point)
        res = []
        nn.k_neighbors(res)
        if(res and self.buff):
            neighbor = res[0][0]
            return min(min(self.buff, key=dist_from_p),neighbor, key=dist_from_p)
        elif self.buff:
            return min(self.buff, key=dist_from_p)
        else:
            return res[0][0]

def calc_bbox(obstacles):
    X = []
    Y = []
    for poly in obstacles:
        for point in poly.vertices():
            X.append(point.x())
            Y.append(point.y())
    min_x = min(X)
    max_x = max(X)
    min_y = min(Y)
    max_y = max(Y)
    return (min_x, max_x, min_y, max_y)


def get_new_point(nn, p, etha, d):
    coords = [FT(0) for i in range(6)]
    for i in range(2):
        coords[i] = FT((nn[i] + FT((p[i] -nn[i]).to_double()/((d.to_double())**0.5)) * etha))

    new_point = Point_d(6, coords) #working with 6 dimensional points due to kd-tree compilation issues
    return new_point


def generate_path(path, obstacles,  radius, distance_to_travel, start, end):
    t0 = time.perf_counter()

    etha = FT(1) #parameter of RRT
    distance_per_edge_limit = FT(math.sqrt(distance_to_travel * 0.02)) # added parameter

    #A list of the obstacles, each represented as a CGAL Polygon_2 object
    obstacles = [Polygon_2(obstacle) for obstacle in obstacles]

    #Construct a graph. Add the start and target configurations as vertices
    G = nx.DiGraph()
    #We work with 6 dimensional points because of kd-tree compilation issues
    # print(type(start.x()))
    begin = Point_d(6, [start.x(), start.y(), FT(0), FT(0), FT(0), FT(0)])
    G.add_node(begin)
    # set up kd tree for Nearest neighbor queries

    tree = Dynamic_kd_tree()
    tree.insert(begin)
    eps = FT(Gmpq(0.0))  # 0.0 for exact NN, otherwise approximate NN
    search_nearest = True  # set this value to False in order to search farthest
    sort_neighbors = False  # set this value to True in order to obtain the neighbors sorted by distance

    # Construct a data structure for coliision detection queries
    cd = Collision_detection.Collision_detector(obstacles, radius)

    #compute a bounding box bounding the C-space
    bbox = calc_bbox(obstacles)
    x_range = (bbox[0].to_double(), bbox[1].to_double())
    y_range = (bbox[2].to_double(), bbox[3].to_double())

    #The algorithm:
    lst = []
    ed = Euclidean_distance()
    i = 0
    j = 0
    done = False
    while(done != True):
        if(i%50 == 0):
            #Every 200 iterations attempt to connect the goal configuration
            # changed the parameter to 100
            # print("Number of valid points sampled:", i)
            i += 1
            p = end
            nn = tree.nearest_neighbor(p)
            d = ed.transformed_distance(p, nn)  # returns the true Euclidean distance
            # find the new config at distance eta from nn
            if (d.to_double()**0.5 < distance_per_edge_limit.to_double()**2):
                new_point = p
                # print("less than etha")
            else:
                new_point = get_new_point(nn, p, distance_per_edge_limit, d)
            potential_edge = Segment_2(Point_2(nn[0], nn[1]), Point_2(new_point[0], new_point[1]))
            d = (ed.transformed_distance(nn, new_point).to_double())**0.5  # returns the true Euclidean distance
            if cd.is_edge_valid(potential_edge):
                G.add_node(new_point)
                G.add_weighted_edges_from([(nn, new_point, d), (new_point, nn, d)])
                tree.insert(new_point)

                if new_point == end:
                    done = True
                    break
        else:
            #sample a random configuration (x,y - positions for the center of the robot)
            rand_x = FT(random.uniform(x_range[0], x_range[1]))
            rand_y = FT(random.uniform(y_range[0], y_range[1]))
            p = Point_d(6, [rand_x, rand_y, FT(0), FT(0), FT(0), FT(0)])

            #test whether this configuration is valid.
            #This should be replaced with a code calling the Collision-detector that we will provide you with
            if(cd.is_point_valid(Point_2(rand_x, rand_y))):
                i += 1
                nn = tree.nearest_neighbor(p) #find the nearest config in the tree
                d = ed.transformed_distance(p, nn) # returns the true Euclidean distance
                #find the new config at distance eta from nn
                if(d < etha):
                    new_point = p
                    # print("less than etha")
                else:
                    new_point = get_new_point(nn, p, etha, d)
                #test whether the edge is collision free (consider both collision with the obstacles, and robot-robot collision)
                potential_edge = Segment_2(Point_2(nn[0], nn[1]), Point_2(new_point[0], new_point[1]))
                d = (ed.transformed_distance(nn, new_point).to_double())**0.5  # returns the true Euclidean distance
                if cd.is_edge_valid(potential_edge):
                    #add node + edge to the graph
                    G.add_node(new_point)
                    G.add_weighted_edges_from([(nn, new_point, d), (new_point, nn, d)])

                    tree.insert(new_point)
                    j += 1

    # print("done")
    # print("tree size:", j)
    G.add_node(end)
    """
    if(nx.has_path(G, begin, end)):
        # print("path found")
        #temp = nx.shortest_path(G, begin, end)
        #tempLen = nx.shortest_path_length(G, begin, end)
        temp = nx.dijkstra_path(G, begin, end, weight='weight')
        for p in temp:
            path.append([Point_2(p[0], p[1])])
        # print(path)
        t1 = time.perf_counter()
        # print("time:", t1-t0)
    """

    return G, tree


def find_rrt_single_robot_path(time_left, radius, distance_to_travel, robot, objective, obstacles):
    #Run RRT for a single disk robot

    start = robot
    goal = objective

    path = []
    single_robot_graph = generate_path(path, obstacles, radius, distance_to_travel, start, goal)
    # print("Found path: ", path)

    return single_robot_graph

