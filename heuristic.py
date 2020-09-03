from arr2_epec_cs_ex import *
import networkx as nx
import Collision_detection
import conversions

class Heuristic():
    def __init__(self, graphs_single_robot, all_pairs_shortest_paths_per_robot):
        self.graphs_single_robot = graphs_single_robot
        self.all_pairs_shortest_paths_per_robot = all_pairs_shortest_paths_per_robot


def calc_heur(heuristic_obj, i, v_near, neighbor, objective,coupons,r):
    G = heuristic_obj.graphs_single_robot[i]
    v_near_to_neighbor = 0
    if v_near != neighbor and v_near != None:
        coupcord = [coupon[0] for coupon in coupons]
        tup = Collision_detection.is_collision_robot_coupons(coupcord, conversions.point_d_to_point_2(v_near), r)
        if tup[0] == True:
            v_near_to_neighbor = G[v_near][neighbor]['weight'] #- tup[1][1]*1000
        else:
            v_near_to_neighbor = G[v_near][neighbor]['weight']

    neighbor_to_objective_path_len = 0
    path = heuristic_obj.all_pairs_shortest_paths_per_robot[i][neighbor][objective]

    for i in range(1,len(path)):
        neighbor_to_objective_path_len += G[path[i]][path[i - 1]]['weight']
    return v_near_to_neighbor + neighbor_to_objective_path_len


def compute_all_pair_shortest_path(graphs_single_robot):
    all_pairs_shortest_paths_per_robot = []
    for i in range(len(graphs_single_robot)):
        all_pairs_shortest_paths_per_robot.append(\
            nx.johnson(graphs_single_robot[i], weight='weight'))

    return all_pairs_shortest_paths_per_robot


def makeHeuristic(graphs_single_robot):
    all_pairs_shortest_paths_per_robot = compute_all_pair_shortest_path(graphs_single_robot)
    heuristic_obj = Heuristic(graphs_single_robot, all_pairs_shortest_paths_per_robot)

    return heuristic_obj