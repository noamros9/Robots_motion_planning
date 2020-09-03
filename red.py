from arr2_epec_cs_ex import *
import drrt_ao
import heuristic
import conversions


class RedTeam:
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

    # initialize team
    red_team = RedTeam(init_time, turn_time, total_time, distance_to_travel, radius, team_robots, opponent_robots, \
                       team_objectives, opponent_objectives, obstacles, bonuses)
    graphs_single_robots, trees_singles_robots = drrt_ao.calculate_consituent_roadmaps(red_team)
    red_team.graphs_single_robots = graphs_single_robots
    red_team.trees_single_robots = trees_singles_robots
    red_team_heuristic_obj = heuristic.makeHeuristic(red_team.graphs_single_robots)
    red_team.g_tensor = drrt_ao.find_path_drrtAst(red_team, red_team_heuristic_obj)
    params[11].append(red_team)
    return red_team.g_tensor.best_path


def play_turn(params):
    team_status = params[1]
    opponent_status = params[2]
    bonuses = params[3]
    data = params[4]
    remaining_time = params[5]
    red_team = data[0]
    d = 0
    k = 1
    while d <= red_team.distance_to_travel and k < len(red_team.g_tensor.best_path):
        for i in range(len(red_team.team_robots)):
            d += (((conversions.point_d_to_point_2(red_team.g_tensor.best_path[k-1][i])).x().to_double() -
                   (conversions.point_d_to_point_2(red_team.g_tensor.best_path[k][i])).x().to_double())**2 +
                  ((conversions.point_d_to_point_2(red_team.g_tensor.best_path[k - 1][i])).y().to_double() -
                   (conversions.point_d_to_point_2(red_team.g_tensor.best_path[k][i])).y().to_double())**2) ** 0.5
        k += 1
    i = 0
    path = [red_team.g_tensor.best_path[i] for i in range(k)]
    for i in range(len(path)):
        path[i] = [conversions.point_d_to_point_2(path[i][j]) for j in range(len(path[i]))]
    params[0].extend(path)
    if remaining_time < red_team.turn_time:
        red_team.turn_time = remaining_time