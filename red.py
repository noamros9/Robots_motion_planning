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


def best_path_step_distance(team_robots, i, k):
    # distance for the k'th step of robot number i
    d = (((conversions.point_d_to_point_2(team_robots.g_tensor.best_path[k - 1][i])).x().to_double() -
           (conversions.point_d_to_point_2(team_robots.g_tensor.best_path[k][i])).x().to_double()) ** 2 +
          ((conversions.point_d_to_point_2(team_robots.g_tensor.best_path[k - 1][i])).y().to_double() -
           (conversions.point_d_to_point_2(team_robots.g_tensor.best_path[k][i])).y().to_double()) ** 2) ** 0.5
    return d

# def update_best_path(team_robots):


def get_last_point(team_robots, path_len):
    # return the last point of the turn
    # we may have fuel only for a part of the the robots, so the other robots will stay in the same place
    step_index = max(path_len)
    last_point = []
    for i in range(team_robots.team_robots):
        if path_len[i] == step_index:
            last_point.append(team_robots.g_tensor.best_path[step_index][i])
        else:
            last_point.append(team_robots.g_tensor.best_path[step_index-1][i])
    return last_point


def get_turn_path(team_robots, path_len):
    # get the path of each robot we will make in this current turn
    path = [team_robots.g_tensor.best_path[i] for i in range(min(path_len))]
    if max(path_len) > min(path_len):
        path.append(get_last_point(team_robots, path_len))
    for i in range(len(path)):
        path[i] = [conversions.point_d_to_point_2(path[i][j]) for j in range(len(team_robots.team_robots))]
    return path


def update_best_path(team_robots, path_len):
    # update best_path by removing all the points that we already used
    first_index = min(path_len)
    team_robots.g_tensor.best_path = team_robots.g_tensor.best_path[first_index:]
    for i in range(len(team_robots.team_robots)):
        if path_len[i] != first_index:
            team_robots.g_tensor.best_path[0][i] = team_robots.g_tensor.best_path[0][i+1]


def walk_best_path(team_robots):
    # walk on the best path and update it's starting points
    total_distance, k = 0, 1
    path_len = [0] * len(team_robots.team_robots)
    is_distance_left = True
    while k < len(team_robots.g_tensor.best_path) and is_distance_left:
        for i in range(len(team_robots.team_robots)):
            d = best_path_step_distance(team_robots, i, k)
            if total_distance + d <= team_robots.distance_to_travel:
                total_distance = total_distance + d
                path_len[i] += 1
            else:
                is_distance_left = False
        k = k + 1

    path = get_turn_path(team_robots, path_len)
    update_best_path(team_robots, path_len)
    return path


def play_turn(params):
    team_status = params[1]
    opponent_status = params[2]
    bonuses = params[3]
    data = params[4]
    remaining_time = params[5]
    red_team = data[0]

    if remaining_time < red_team.turn_time:
        red_team.turn_time = remaining_time

    #update_best_path(red_team)  # update our best path according to the last step the opponent took
    path = walk_best_path(red_team)  # walk on the best path and update it's starting points
    params[0].extend(path)
