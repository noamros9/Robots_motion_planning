from arr2_epec_cs_ex import *
import drrt_ao

def initialize(params):
    print("params loaded")
    #print(params)
    #assign meaningful names to varialbes
    init_time = params[0]
    distance_to_travel = params[1]
    radius = params[2]
    team_robots = params[3]
    opponent_robots = params[4]
    team_objectives = params[5]
    opponent_objectives = params[6]
    obstacles = params[7]
    bonuses = params[8]
    #drrt_ao.create_path()

def play_turn(params):
    print(params)
    params[0].append([Point_2(5,5), Point_2(5, 8), Point_2(5, 10)])
    params[0].append([Point_2(5, 10), Point_2(5, 13), Point_2(5, 15)])