from arr2_epec_cs_ex import *
import rrt_single_robot
from rrt_single_robot import Dynamic_kd_tree
import networkx as nx
import Collision_detection
import conversions
import random


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

def calc_heur(neighbor,objective):
    ed = Euclidean_distance()
    return ed.transformed_distance(conversions.point_d_to_point_2(neighbor), objective) # For now the heuristic will be the euclidian distance from the objective

    
# informed_decision receives for each robot: nearest neighbor, random point that was chosen, the tensor product and the objective
def informed_decision(V_near, Q_rand, G, team_objectives):
    num = len(G)
    V_new = [None for i in range(num)]
    for i in range(num):
        N = list(G[i].neighbors(V_near[i])) # A list of all neighbors of V_near in the roadmap
        if conversions.point_d_to_point_2(Q_rand[i]) == team_objectives[i]: # If we are guiding to a first solution (might be a problem because team objectives is a list of Point_2 and Q_rand is a list of Point_d)
            H = [calc_heur(neighbor,team_objectives[i]) for neighbor in N] # H is the list of heuristics for each neighbor of the nn
            V_new[i] = N[H.index(min(H))] # V_new will be the neighbor with the smallest heuristic value
        else: # if we are exploring better solutions
            V_new[i] = N[random.randint(0,len(N)-1)] # V_new will be chosen randomly from the neighbors of the nn
    return V_new


def find_path_in_tensor_roadmap(team):
    N = len(team.graphs_single_robots)
    graphs_single_robots = team.graphs_single_robots
    team_robots = team.team_robots
    team_objectives = team.team_objectives
    path = [Dynamic_kd_tree() for i in range(N)]
    for i in range(N):
        robot_i_x, robot_i_y = conversions.point_2_to_xy(team_robots[i])
        path[i].insert(Point_d(6, [FT(robot_i_x), FT(robot_i_y), FT(0), FT(0), FT(0), FT(0)])) #inserting the starting position of each robot
    # find Q_rand (new random point) in for each single robot
    Q_rands = [0 for i in range(N)]
    V_near = [0 for i in range(N)]
    for i in range(N):
        team_members = [team_robots[j] for j in range(N) if j != i]
        Q_rands[i] = find_random_point(graphs_single_robots[i], team.obstacles, team_members, team.radius)
        V_near[i] = path[i].nearest_neighbor(Q_rands[i])

    V_new = informed_decision(V_near,Q_rands,graphs_single_robots,team_objectives)

    return path

def initialize(init_time, radius,distance_to_travel, team_robots, team_objectives, obstacles):
    time_left = init_time
    graphs_single_robot = [0 for i in range(len(team_robots))]

    # for now - we don't handle time and bonuses.
    # also - an objective is assigned arbitrarily to a robot
    for i in range(len(team_robots)):
        graphs_single_robot[i] = rrt_single_robot.\
            find_rrt_single_robot_path(time_left, radius, distance_to_travel,\
            team_robots[i], team_objectives[i], obstacles)

    return graphs_single_robot
