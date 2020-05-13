from arr2_epec_cs_ex import *
import rrt_single_robot

def build_tensor_roadmap(paths_single_robot):
    a = 1


def initialize_path(init_time, radius,distance_to_travel, team_robots, team_objectives, obstacles):
    time_left = init_time
    paths_single_robot = [0 for i in range(len(team_robots))]

    # for now - we don't handle time and bonuses.
    # also - an objective is assigned arbitrarily to a robot
    for i in range(len(team_robots)):
        paths_single_robot[i] = rrt_single_robot.\
            find_rrt_single_robot_path(time_left, radius, distance_to_travel,\
            team_robots[i], team_objectives[i], obstacles)

    return paths_single_robot
