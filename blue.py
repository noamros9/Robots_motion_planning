from arr2_epec_cs_ex import *
import drrt_ao

def initialize(params):
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

    # create the basic map for the initialization phase. For now, don't consider distance to travel
    # because we don't actually play in this turn
    # for now don't consider opponent objects and bonuses
    # need to be handled later
    # in the initialize phase opponent_robots aren't obstacles (otherwise we can't compute)
    path = drrt_ao.initialize_path(init_time, radius,distance_to_travel, team_robots, team_objectives, obstacles)

    #return path


def play_turn(params):
    #print(params)
    params[0].append([Point_2(5,5), Point_2(5, 8), Point_2(5, 10)])
    params[0].append([Point_2(5, 10), Point_2(5, 13), Point_2(5, 15)])