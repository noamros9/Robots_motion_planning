from arr2_epec_cs_ex import *
import drrt_ao
import heuristic
from walk_path import walk_best_path
import time
import conversions
import Collision_detection
import random
import networkx as nx


def coupon_selection(team_robots, obstacles, radius, bonuses, team_objectives):
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
        cx = cx / 3
        cy = cy / 3
        p = conversions.xy_to_point_2(cx, cy)
        while not cd.is_point_valid(p):
            rand_x = random.uniform(-radius.to_double(), radius.to_double())
            rand_y = random.uniform(-radius.to_double(), radius.to_double())
            p = conversions.xy_to_point_2(cx + rand_x, cy + rand_y)
        team_objectives_coup[i] = conversions.xy_to_point_2(cx, cy)
    return team_objectives_coup

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
    for i in range(len(teamcoup.team_robots)):
        teamcoup.team_robots[i] = team_status[i][0]
    path = walk_best_path(teamcoup, opponent_status)
    params[0].extend(path)


