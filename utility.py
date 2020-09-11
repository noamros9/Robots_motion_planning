from arr2_epec_cs_ex import *
import drrt_ao
import heuristic
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
    teamgoal.g_tensor = drrt_ao.find_path_drrtAst(teamgoal, teamgoal_heuristic_obj, 1, ts, total_nodes)
    best_path.extend(teamgoal.g_tensor.best_path)
    data = [teamcoup, teamgoal, best_path, 0] # 0 is index of current configuration
    params[11].append(data)