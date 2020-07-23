from arr2_epec_cs_ex import *
import drrt_ao
import heuristic
import conversions

class BlueTeam:
    def __init__(self, init_time, distance_to_travel, radius, team_robots, opponent_robots, \
                 team_objectives, opponent_objectives, obstacles, bonuses):
        self.graphs_single_robots = []
        self.graphs_single_robots = []
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


def initialize(params):
    # assign meaningful names to varialbes
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

    # create the basic map for the initialization phase. For now, don't consider distance to travel
    # because we don't actually play in this turn
    # for now don't consider opponent objects and bonuses
    # need to be handled later
    # in the initialize phase opponent_robots aren't obstacles (otherwise we can't compute)

    blue_team = BlueTeam(init_time, distance_to_travel, radius, team_robots, opponent_robots, \
                       team_objectives, opponent_objectives, obstacles, bonuses)
    graphs_single_robots, trees_singles_robots = drrt_ao.calculate_consituent_roadmaps(blue_team)
    blue_team.graphs_single_robots = graphs_single_robots
    blue_team.trees_single_robots = trees_singles_robots
    blue_team_heuristic_obj = heuristic.makeHeuristic(blue_team.graphs_single_robots)

    # as in initialization - we don't consider opponent robots as obstacles in this phase.
    # we would consider it in play_turn
    path = drrt_ao.find_path_drrtAst(blue_team, blue_team_heuristic_obj)
    return path


def play_turn(params):
    #print(params)
    params[0].append([Point_2(5,5), Point_2(5, 8), Point_2(5, 10)])
    params[0].append([Point_2(5, 10), Point_2(5, 13), Point_2(5, 15)])