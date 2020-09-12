from arr2_epec_cs_ex import *
import drrt_ao
import heuristic
import utility
import conversions
import networkx as nx
import time
import Collision_detection
import random
from walk_path import walk_best_path


class BlueTeam:
    def __init__(self, init_time, turn_time, total_time, distance_to_travel, radius, team_robots, opponent_robots, \
                 team_objectives, opponent_objectives, obstacles, bonuses):
        self.graphs_single_robots = []
        self.trees_single_robots = []
        self.init_time = init_time
        self.distance_to_travel = distance_to_travel
        self.radius = radius
        self.team_robots = team_robots
        self.opponent_robots = opponent_robots
        self.starting_point = team_robots
        self.team_objectives = [conversions.point_2_to_point_d(team_objectives[i]) for i in range(len(team_robots))]
        self.opponent_objectives = opponent_objectives
        self.obstacles = obstacles
        self.bonuses = bonuses
        self.turn_time = turn_time
        self.total_time = total_time
        self.g_tensor = []


def initialize(params):
    # assign meaningful names to varialbes
    ts = time.time()
    radius = params[0]
    turn_time = params[1]
    init_time = params[2]
    total_time = params[3]
    distance_to_travel = params[4]
    team_robots = params[5]
    opponent_robots = params[6]
    team_objectives = params[7]
    opponent_objectives = params[8]
    obstacles = params[9]
    bonuses = params[10]
    bonuses.sort(key=lambda x: x[1], reverse=True)

    # initialize team
    cd = Collision_detection.Collision_detector(obstacles, radius)
    mn = min(len(team_robots), len(bonuses))
    team_objectives_coup = list(team_objectives)
    for i in range(mn):
        cx = 0
        cy = 0
        tups = conversions.polygon_2_to_tuples_list(bonuses[i][0])
        for j in range(3):
            cx += tups[j][0]
            cy += tups[j][1]
        cx = cx/3
        cy = cy/3
        p = conversions.xy_to_point_2(cx, cy)
        while not cd.is_point_valid(p):
            rand_x = random.uniform(-radius.to_double(), radius.to_double())
            rand_y = random.uniform(-radius.to_double(), radius.to_double())
            p = conversions.xy_to_point_2(cx + rand_x, cy + rand_y)
        team_objectives_coup[i] = conversions.xy_to_point_2(cx, cy)
    teamcoup = BlueTeam(init_time, turn_time, total_time, distance_to_travel, radius, team_robots, opponent_robots, \
                       team_objectives_coup, opponent_objectives, obstacles, bonuses)
    team_robots = team_objectives_coup
    teamgoal = BlueTeam(init_time, turn_time, total_time, distance_to_travel, radius, team_robots, opponent_robots, \
                       team_objectives, opponent_objectives, obstacles, bonuses)
    utility.team_init(teamcoup, teamgoal, params, ts)



def play_turn(params):
    utility.team_play_turn(params)
