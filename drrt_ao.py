from arr2_epec_cs_ex import *
import rrt_single_robot
import time
from rrt_single_robot import Dynamic_kd_tree
import networkx as nx
import Collision_detection
import conversions
import random
import heuristic
import math
import G_tensor


def find_random_point(graph_single_robot, obstacles, team_robots, radius):
    p = []
    cd = Collision_detection.Collision_detector(obstacles, radius)

    # compute a bounding box bounding the C-space
    bbox = rrt_single_robot.calc_bbox(obstacles)
    x_range = (bbox[0].to_double(), bbox[1].to_double())
    y_range = (bbox[2].to_double(), bbox[3].to_double())

    flag = False

    while not flag:
        rand_x = FT(random.uniform(x_range[0], x_range[1]))
        rand_y = FT(random.uniform(y_range[0], y_range[1]))
        p = Point_d(6, [rand_x, rand_y, FT(0), FT(0), FT(0), FT(0)])
        p_Point_2 = Point_2(rand_x, rand_y)

        if cd.is_point_valid(p_Point_2) and Collision_detection.is_point_valid_robot_robot(p_Point_2, team_robots, radius):
            flag = True
            return p

    return p


# informed_decision
# receives for each robot: nearest neighbor, random point that was chosen, the tensor product and the objective
def informed_decision(team, V_near, Q_rand, heuristic_obj):
    G = team.graphs_single_robots
    V_new = [None for i in range(len(G))]
    for i in range(len(G)):
        N = adj(G[i], V_near[i])  # A list of all neighbors of V_near[i] in the roadmap
        if V_near[i] == team.team_objectives[i]:
            V_new[i] = V_near[i]
        elif Q_rand[i] == team.team_objectives[i]: # If we are guiding to a first solution
            # H is the list of heuristics for each neighbor of the nn
            H = [heuristic.calc_heur(heuristic_obj, i, V_near[i], N[j], team.team_objectives[i]) for j in range(len(N))]
            V_new[i] = N[H.index(min(H))] # V_new will be the neighbor with the smallest heuristic value
        else: # if we are exploring better solutions
            V_new[i] = N[random.randint(0,len(N)-1)] # V_new will be chosen randomly from the neighbors of the nn
    return V_new


# returns a list of all neighbors of a node in the graph (if there are)
def adj(graph, node, include_self = False):
    n = []
    if graph.has_node(node):
        n = list(graph.neighbors(node))
    if include_self:
        n.append(node)
    return n


def is_possible_neighbor(V_new, g_tensor, ID):
    nodes = g_tensor.configs[ID]['nodes']
    for i in range(len(V_new)):
        if nodes[i] != V_new[i] and not g_tensor.G[i].has_edge(nodes[i], V_new[i]):
            return False
    return True


def all_neighbors_stay_in_objective(V_new, V_last, g_tensor):
    N = len(g_tensor.G)
    first = True
    possible_neighbor_configs = []
    neighbors = []
    for i in range(N):
        if V_last[i] == g_tensor.team.team_objectives[i]:
            for ID in g_tensor.node_configs[i][V_last[i]]:
                if is_possible_neighbor(V_new, g_tensor, ID):
                    if first:
                        possible_neighbor_configs.append(ID)
                    else:  # we keep only the configurations which keep all the nodes in place if in objectives
                        if ID in possible_neighbor_configs:
                            neighbors.append(ID)

    if len(neighbors) == 0:
        return possible_neighbor_configs
    return neighbors


# return the neighbors, which are adjacent to v_new in G and also been added to tree
def neighbors(V_new, g_tensor):
    N = len(g_tensor.G)
    possible_neighbor_configs = []
    for i in range(N):
        for n in adj(g_tensor.G[i], V_new[i], True):
            if g_tensor.path[i][0].has_node(n):
                for ID in g_tensor.node_configs[i][n]:
                    if is_possible_neighbor(V_new, g_tensor, ID) and ID not in possible_neighbor_configs:
                        possible_neighbor_configs.append(ID)
    return possible_neighbor_configs


# calculate the cost of the path
def cost_path(G, start, neighbor):
    return nx.shortest_path_length(G, start, neighbor, weight='weight')


# calculate the cost of moving from v to v_new
def cost_moving(g_tensor, neighbor, v_new):
    nodes = g_tensor.configs[neighbor]['nodes']
    cost = 0
    for i in range(len(v_new)):
        if nodes[i] != v_new[i]:
            cost += g_tensor.G[i][nodes[i]][v_new[i]]['weight']
    return cost


def calculate_V_best(all_neighbors, V_new, g_tensor, team):
    N = len(V_new)
    s = [conversions.point_2_to_point_d(team.starting_point[i]) for i in range(N)]
    num_neighbors = len(all_neighbors)
    costs = [0 for i in range(num_neighbors)]
    for i in range(num_neighbors):
        costs[i] = g_tensor.configs[all_neighbors[i]]['cost']
        costs[i]+= cost_moving(g_tensor, all_neighbors[i], V_new)
    best_neighbor_ID = all_neighbors[costs.index(min(costs))]
    neighbor = g_tensor.configs[best_neighbor_ID]['nodes']
    if Collision_detection.edges_collision_free(team, neighbor, V_new):
        V_best = neighbor
        return V_best, min(costs)

    return None, 0


# TO DO
# look in the tree, for a path that ends with team_objectives
def connect_to_target(team, tree):
    path = []
    return path


# the expansion step
def expand_drrtAst(g_tensor, team, heuristic_obj, V_last, best_path_cost):
    N = len(team.team_robots)
    Q_rand = [0 for i in range(N)]
    V_near = [0 for i in range(N)]
    if V_last == None:
        # find Q_rand (new random point) in for each single robot
        for i in range(N):
            team_members = [team.team_robots[j] for j in range(N) if j != i]
            Q_rand[i] = find_random_point(team.graphs_single_robots[i], team.obstacles, team_members, team.radius)
        V_near = g_tensor.nearest_config(Q_rand)
    else:
        Q_rand = team.team_objectives
        V_near = V_last
    V_new = informed_decision(team, V_near, Q_rand, heuristic_obj)

    all_neighbors = []
    if V_last != None and True in [V_last[i] == team.team_objectives[i] for i in range(N)]:
        all_neighbors = all_neighbors_stay_in_objective(V_new, V_last, g_tensor)
    else:
        all_neighbors = neighbors(V_new, g_tensor)

    V_best, start_to_V_new_cost = calculate_V_best(all_neighbors, V_new, g_tensor, team)

    if V_best is None:
        return None

    # check if V_new make the travel cheaper
    shortest_paths_single_robot = heuristic_obj.all_pairs_shortest_paths_per_robot
    if not math.isinf(best_path_cost):
        v_new_to_target_cost = sum([cost_path(g_tensor.G[i], V_new[i],team.team_objectives[i]) \
                                    for i in range(N)])
        total_cost_v_new = start_to_V_new_cost + v_new_to_target_cost
        if total_cost_v_new > best_path_cost:
            return None

    v_new_exist = g_tensor.config_by_nodes(V_new)
    if v_new_exist == -1:   # meaning the configuration V_new is not in the tensor roadmap
        g_tensor.add_new_config(V_best, V_new)
    else:
        g_tensor.rewire(V_best, V_new)
    for i in range(len(all_neighbors)):
        if V_new != g_tensor.configs[all_neighbors[i]]['nodes']:
            g_tensor.rewire(V_new, g_tensor.configs[all_neighbors[i]]['nodes'])

    # if we made progress
    heuristic_V_new = sum([heuristic.calc_heur(heuristic_obj, i, None, V_new[i], team.team_objectives[i])\
                           for i in range(N)])
    heuristic_V_best = sum([heuristic.calc_heur(heuristic_obj, i, None, V_best[i], team.team_objectives[i])\
                           for i in range(N)])
    if heuristic_V_new < heuristic_V_best:
        return V_new

    return None


#   dRRT* (G, S, T, nit):
#       G = team.graphs_single_robots
#       S = team.team_robots
#       T = team.team_objectives
#       nit = will be set manually
#       flag == 0 means init search coupon mode, 1 means init search goal mode, 2 means play turn mode
def find_path_drrtAst(team, heuristic_obj, mode, ts, total_nodes):
    t1 = time.time()
    n_it = 100  # number of iterations to expand the tree
    # in the future it will be time-bound. for now we will let it run

    N = len(team.graphs_single_robots)
    team_robots = team.team_robots

    # drrt* initialization (line 1 of drrt* algorithm 6)
    best_path = []
    best_path_cost = math.inf     # cost of pi_best in the beginning
    g_tensor = G_tensor.G_tensorMap(N, team_robots, team, heuristic_obj) # initialize the tensorMap
    V_last = [conversions.point_2_to_point_d(team_robots[i]) for i in range(N)]  # initialize v_last to the starting positions
    # drrt* while loop (line 2 of drrt* algorithm 6)
    # TO DO: change it to work by time elapsed
    curr_t = time.time()
    if mode == 2:
        time_end = ts + team.turn_time
    else:
        time_end = ts + team.init_time
        if mode == 0:
            nodes = 0
            for i in range(len(team.team_robots)):
                nodes += len(team.graphs_single_robots[i].nodes)
            time_limit = curr_t + (time_end-curr_t)*(nodes/total_nodes)
        else:
            time_limit = time_end
    while curr_t < time_limit - 1:
        for i in range(n_it):
            if curr_t < time_limit - 0.75:
                V_last = expand_drrtAst(g_tensor, team, heuristic_obj, V_last, best_path_cost)
            else:
                break
        g_tensor.update_costs()
        # drrt* path - update best_path if the current path is better (and valid)
        target_config = g_tensor.config_by_nodes(team.team_objectives)
        if target_config != -1:  # the target configuration exists
            cost_path_to_target = g_tensor.configs[target_config]['cost']
            if cost_path_to_target < best_path_cost:
                best_path_cost = cost_path_to_target
                g_tensor.best_path = g_tensor.build_path(target_config)
            V_last = None  # if the time isn't over try to keep exploring the map
        curr_t = time.time()
    t2 = time.time()
    print("drrt mode: ", mode, " took: ", t2 - t1)
    return g_tensor

# calculate the RRT for each robot separately
def calculate_consituent_roadmaps(team):
    ts = time.time()
    time_left = team.init_time
    N = len(team.team_robots)
    graphs_single_robot = [0 for i in range(N)]
    trees_single_robot = [0 for i in range(N)]

    # for now - we don't handle time and bonuses.
    # also - an objective is assigned arbitrarily to a robot
    for i in range(N):
        graphs_single_robot[i], trees_single_robot[i] = rrt_single_robot.\
            find_rrt_single_robot_path(time_left, team.radius, team.distance_to_travel,\
            team.team_robots[i], team.team_objectives[i], team.obstacles, team.bonuses)
# Put different arbitrary coupons nodes for each robot
    tf = time.time()
    print("RRT measurment took: ", tf - ts)
    return graphs_single_robot, trees_single_robot
