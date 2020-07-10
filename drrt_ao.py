from arr2_epec_cs_ex import *
import rrt_single_robot
from rrt_single_robot import Dynamic_kd_tree
import networkx as nx
import Collision_detection
import conversions
import random
import heuristic


def find_random_point(graph_single_robot, obstacles, team_robots, radius):
    p = []

    cd = Collision_detection.Collision_detector(obstacles, radius)

    #compute a bounding box bounding the C-space
    bbox = rrt_single_robot.calc_bbox(obstacles)
    x_range = (bbox[0].to_double(), bbox[1].to_double())
    y_range = (bbox[2].to_double(), bbox[3].to_double())

    flag = False

    while not flag:
        rand_x = FT(random.uniform(x_range[0], x_range[1]))
        rand_y = FT(random.uniform(y_range[0], y_range[1]))
        p = Point_d(6, [rand_x, rand_y, FT(0), FT(0), FT(0), FT(0)])
        p_Point_2 = Point_2(rand_x, rand_y)

        if cd.is_point_valid(p_Point_2) and cd.is_point_valid_robot_robot(p_Point_2, team_robots, radius):
            flag = True
            return p

    return p


# informed_decision
# receives for each robot: nearest neighbor, random point that was chosen, the tensor product and the objective
def informed_decision(team, V_near, Q_rand, heuristic_obj):
    G = team.graphs_single_robots
    V_new = [None for i in range(len(G))]
    for i in range(len(G)):
        N = list(G[i].neighbors(V_near[i])) # A list of all neighbors of V_near in the roadmap
        # in the article it is to consider the point itself too (so the robot might stay in place)
        # need to discuss it
        # N.append(V_near[i])
        if conversions.point_d_to_point_2(V_near[i]) == team.team_objectives[i]:
            V_new[i] = V_near[i]
        elif conversions.point_d_to_point_2(Q_rand[i]) == team.team_objectives[i]: # If we are guiding to a first solution
            # H is the list of heuristics for each neighbor of the nn
            H = [heuristic.calc_heur(heuristic_obj,\
                    i, V_near[i], neighbor, team.team_objectives[i]) for neighbor in N]
            V_new[i] = N[H.index(min(H))] # V_new will be the neighbor with the smallest heuristic value
        else: # if we are exploring better solutions
            V_new[i] = N[random.randint(0,len(N)-1)] # V_new will be chosen randomly from the neighbors of the nn
    return V_new


# TO DO
# return the neighbors, which are adjacent to v_new in G and also been added to tree
def neighbors(v_new, G, tree):
    all_neighbors = [None for i in range(len(G))]  # A list of all neighbors of V_new
    return all_neighbors


# TO DO
# look in the tree, for a path that ends with team_objectives
def connect_to_target(team, tree):
    path = []
    return path


# TO DO
# calculate the cost of the path
def cost_path(path):
    cost = 0
    return cost


# TO DO
# calculate the cost of moving from v to v_new
def cost_moving(v, v_new):
    cost = 0
    return cost


def expand_drrtAst(tree, team, heuristic_obj, V_last):
    N = len(team.team_robots)
    Q_rand = [0 for i in range(N)]
    V_near = [0 for i in range(N)]

    if V_last == [None for i in range(N)]:
        # find Q_rand (new random point) in for each single robot
        for i in range(N):
            team_members = [team.team_robots[j] for j in range(N) if j != i]
            Q_rand[i] = find_random_point(team.graphs_single_robots[i], team.obstacles, team_members, team.radius)
            V_near[i] = tree[i].nearest_neighbor(Q_rand[i])
    else:
        for i in range(N):
            Q_rand[i] = conversions.point_2_to_point_d(team.team_objectives[i])
            V_near = V_last

    V_new = informed_decision(team, V_near, Q_rand, heuristic_obj)
    G = team.graphs_single_robots
    all_neighbors = neighbors(V_new, G, tree)
    V_best = all_neighbors.index(min([(cost_path(v) + cost_moving(v, V_new)) for v in all_neighbors]))

    if V_best is None:
        return None
    if cost_path(V_new) > cost_path(V_best):
        return None

    # TO DO - LINES 12-21 OF THE ALGORITHM

    return V_new


#   dRRT* (G, S, T, nit):
#       G = team.graphs_single_robots
#       S = team.team_robots
#       T = team.team_objectives
#       nit = will be set manually
def find_path_drrtAst(team, heuristic_obj):

    n_it = 100  # number of iterations to expand the tree
    # in the future it will be time-bound. for now we will let it run
    num_of_tries = 5

    N = len(team.graphs_single_robots)
    team_robots = team.team_robots

    # drrt* initialization (line 1 of drrt* algorithm 6)
    best_path = [None for i in range(N)]
    tree = [Dynamic_kd_tree() for i in range(N)]
    v_last = [None for i in range(N)]

    for i in range(N):
        robot_i_x, robot_i_y = conversions.point_2_to_xy(team_robots[i])
        robot_in_6d = Point_d(6, [FT(robot_i_x), FT(robot_i_y), FT(0), FT(0), FT(0), FT(0)])
        tree[i].insert(robot_in_6d)  # inserting the starting position of each robot
        v_last[i] = robot_in_6d  # initialize v_last to the starting positions

    # drrt* while loop (line 2 of drrt* algorithm 6)
    while num_of_tries > 0:
        for i in range(n_it):
            v_last = expand_drrtAst(tree, team, heuristic_obj, v_last)
        num_of_tries -= 1
        # drrt* path - update best_path if the current path is better (and valid)
        path = connect_to_target(team, tree)
        if path and cost_path(path) < cost_path(best_path):
            best_path = path

    return best_path


def calculate_consituent_roadmaps(team):
    time_left = team.init_time
    N = len(team.team_robots)
    graphs_single_robot = [0 for i in range(N)]

    # for now - we don't handle time and bonuses.
    # also - an objective is assigned arbitrarily to a robot
    for i in range(N):
        graphs_single_robot[i] = rrt_single_robot.\
            find_rrt_single_robot_path(time_left, team.radius, team.distance_to_travel,\
            team.team_robots[i], team.team_objectives[i], team.obstacles)

    return graphs_single_robot
