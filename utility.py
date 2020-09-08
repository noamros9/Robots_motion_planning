from arr2_epec_cs_ex import *
import drrt_ao
import heuristic
import conversions
import networkx as nx


def team_init(team,params, team_objectives):
    mn = min(len(team.team_robots), len(team.bonuses))
    for i in range(mn):
        team.team_objectives[i] = conversions.tup_to_point_d((conversions.polygon_2_to_tuples_list(team.bonuses[i][0]))[0])
    graphs_single_robots, trees_singles_robots = drrt_ao.calculate_consituent_roadmaps(team)
    team.graphs_single_robots = graphs_single_robots
    team.trees_single_robots = trees_singles_robots
    team_heuristic_obj = heuristic.makeHeuristic(team.graphs_single_robots, team.trees_single_robots)
    team.g_tensor = drrt_ao.find_path_drrtAst(team, team_heuristic_obj)
    best_path = team.g_tensor.best_path
    team.team_objectives = [conversions.point_2_to_point_d(team_objectives[i]) for i in range(len(team.team_robots))]
    team.team_robots = [conversions.point_d_to_point_2(point) for point in best_path[-1]]
    team.starting_point = team.team_robots
    graphs_single_robots, trees_singles_robots = drrt_ao.calculate_consituent_roadmaps(team)
    team.graphs_single_robots = graphs_single_robots
    team.trees_single_robots = trees_singles_robots
    team_heuristic_obj = heuristic.makeHeuristic(team.graphs_single_robots, team.trees_single_robots)
    team.g_tensor = drrt_ao.find_path_drrtAst(team, team_heuristic_obj)
    best_path.extend(team.g_tensor.best_path)
    data = [team, best_path]
    params[11].append(data)