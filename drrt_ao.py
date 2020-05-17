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


def oracle():
   # place holder
   a = 1


def find_path_in_tensor_roadmap(team):
    path = []

    # find Q_rand (new random point) in for each single robot
    graphs_single_robots = team.graphs_single_robots
    N = len(graphs_single_robots)
    Q_rands = [0 for i in range(N)]
    for i in range(N):
        team_members = [team.team_robots[j] for j in range(N) if j != i]
        Q_rands[i] = find_random_point(graphs_single_robots[i], team.obstacles, team_members, team.radius)


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
