from arr2_epec_cs_ex import *
import networkx as nx
import Collision_detection
import conversions
import random

class Heuristic():
    def __init__(self, graphs_single_robot, all_pairs_shortest_paths_per_robot):
        self.graphs_single_robot = graphs_single_robot
        self.all_pairs_shortest_paths_per_robot = all_pairs_shortest_paths_per_robot


def calc_heur(heuristic_obj, i, v_near, neighbor, objective):
    G = heuristic_obj.graphs_single_robot[i]
    v_near_to_neighbor = 0
    if v_near != neighbor and v_near != None:
        v_near_to_neighbor = G[v_near][neighbor]['weight']

    neighbor_to_objective_path_len = 0
    path = heuristic_obj.all_pairs_shortest_paths_per_robot[i][neighbor][objective]

    for i in range(1,len(path)):
        neighbor_to_objective_path_len += G[path[i]][path[i - 1]]['weight']
    return v_near_to_neighbor + neighbor_to_objective_path_len


def compute_all_pair_shortest_path(graphs_single_robot):
    all_pairs_shortest_paths_per_robot = []
    subgraph_nodes = random.sample(graphs_single_robot.nodes,len(graphs_single_robot.nodes)/10)
    for i in range(len(graphs_single_robot)):
        for j in range(len(subgraph_nodes)):
            all_pairs_shortest_paths_per_robot.append(nx.single_source_bellman_ford(graphs_single_robot[i], subgraph_nodes[j], target=None, weight='weight')[1])
        for j in range(len(graphs_single_robot.nodes)):
            p = graphs_single_robot.nodes[j]
            if p in all_pairs_shortest_paths_per_robot:
                continue
            else:
                nn = tree.nearest_neighbor(p)
                paths =  all_pairs_shortest_paths_per_robot[nn]
                all_pairs_shortest_paths_per_robot[p] = paths
        all_pairs_shortest_paths_per_robot.append(\
            nx.johnson(graphs_single_robot[i], weight='weight'))

    return all_pairs_shortest_paths_per_robot


def makeHeuristic(graphs_single_robot):
    all_pairs_shortest_paths_per_robot = compute_all_pair_shortest_path(graphs_single_robot)
    heuristic_obj = Heuristic(graphs_single_robot, all_pairs_shortest_paths_per_robot)

    return heuristic_obj
