from arr2_epec_cs_ex import *
import drrt_ao
import heuristic
from walk_path import walk_best_path
import time
import conversions
import networkx as nx


def team_init(teamcoup, teamgoal, params, ts):
    teamcoup.graphs_single_robots, teamcoup.trees_single_robots = drrt_ao.calculate_consituent_roadmaps(teamcoup)
    teamcoup_heuristic_obj = heuristic.makeHeuristic(teamcoup.graphs_single_robots, teamcoup.trees_single_robots)
    teamgoal.graphs_single_robots, teamgoal.trees_single_robots = drrt_ao.calculate_consituent_roadmaps(teamgoal)
    teamgoal_heuristic_obj = heuristic.makeHeuristic(teamgoal.graphs_single_robots, teamgoal.trees_single_robots)
    total_nodes = 0
    for i in range(len(teamcoup.team_robots)):
        total_nodes += len(teamcoup.graphs_single_robots[i].nodes) + len(teamgoal.graphs_single_robots[i].nodes)
    teamcoup.g_tensor = drrt_ao.find_path_drrtAst(teamcoup, teamcoup_heuristic_obj, 0, ts, total_nodes)
    best_path = teamcoup.g_tensor.best_path
    del best_path[-1] # To remove duplication of point
    teamgoal.g_tensor = drrt_ao.find_path_drrtAst(teamgoal, teamgoal_heuristic_obj, 1, ts, total_nodes)
    best_path.extend(teamgoal.g_tensor.best_path)
    teamcoup.g_tensor.best_path = best_path
    data = [teamcoup, teamgoal]
    params[11].append(data)

def team_play_turn(params):
    team_status = params[1]
    opponent_status = params[2]
    bonuses = params[3]
    data = params[4]
    remaining_time = params[5]
    teamcoup = data[0][0]
    teamgoal = data[0][1]
    if remaining_time < teamcoup.turn_time:
        teamcoup.turn_time = remaining_time
        teamgoal.turn_time = remaining_time
    path = walk_best_path(teamcoup, opponent_status)
    params[0].extend(path)


