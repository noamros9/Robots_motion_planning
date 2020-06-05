from arr2_epec_cs_ex import *
import rrt_single_robot
import networkx as nx
import Collision_detection

def find_random_point(graph_single_robot, obstacles, team_robots, radius):
    p = []

    cd = Collision_detection.Collision_detector(obstacles, radius)

    #compute a bounding box bounding the C-space
    bbox = rrt_single_robot.calc_bbox(obstacles)
    x_range = (bbox[0].to_double(), bbox[1].to_double())
    y_range = (bbox[2].to_double(), bbox[3].to_double())

    #change it to False when is_point_valid_robot_robot is working
    flag = True

    while not flag:
        rand_x = FT(random.uniform(x_range[0], x_range[1]))
        rand_y = FT(random.uniform(y_range[0], y_range[1]))
        p = Point_d(6, [rand_x, rand_y, FT(0), FT(0), FT(0), FT(0)])
        p_Point_2 = Point_2(rand_x, rand_y)

        # TO-DO: need to write is_point_valid_robot_robot
        #if cd.is_point_valid(p_Point_2) and cd.is_point_valid_robot_robot(p_Point_2, team_robots):
        #    flag = True
        #    return p

    return p

def calc_heur(node,objective):
    return ed.transformed_distance(node, objective) # For now the heuristic will be the euclidian distance from the objective

    
    # Oracle receives for each robot: nearest neighbor, random point that was chosen, the tensor product and the objective
def oracle(V_near,Q_rand,G,team_objectives):
    #V_new = [empty for i in range(N)] 
    for i in range(N):
        if Q_rand[i] = team_objectives[i]: # If we are guiding to a solution 
            # H = [calc_heur(neighbor,team_objectives[i]) for each neighbor) # H is the list of heuristics for each neighbor of the nn
            # V_new[i] = neighbor[H.index(min(values))] # V_new will be the neighbor with the smallest heuristic value
        else: # if we are exploring better solutions
            # V_new[i] = neighbor[randint(0,Length(neighbor))] # V_new will be chosen randomly from the neighbors of the nn
    return V_new


def find_path_in_tensor_roadmap(team):
    path = []

    # find Q_rand (new random point) in for each single robot
    graphs_single_robots = team.graphs_single_robots
    N = len(graphs_single_robots)
    Q_rands = [0 for i in range(N)]
    V_near = [0 for i in range(N)]
    for i in range(N):
        team_members = [team.team_robots[j] for j in range(N) if j != i]
        Q_rands[i] = find_random_point(graphs_single_robots[i], team.obstacles, team_members, team.radius)
        #V_near[i] = tree.nearest_neighbor(Q_rands[i])
    V_new = oracle(V_near,Q_rands,,team_objectives)

    # to-do: oracle and rest of algorithm 7 in the drrt* paper

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
