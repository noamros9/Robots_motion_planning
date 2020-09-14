from arr2_epec_cs_ex import *
import networkx as nx
import time
import random
import drrt_ao
import math
import Collision_detection
import conversions

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

    for i in range(1, len(path)):
        neighbor_to_objective_path_len += G[path[i]][path[i - 1]]['weight']
    return v_near_to_neighbor + neighbor_to_objective_path_len


def compute_all_pair_shortest_path(graphs_single_robot, trees_single_robot):
    all_pairs_shortest_paths_per_robot = []
    all_pairs_shortest_paths_per_robot_b = [dict() for i in range(len(graphs_single_robot))]
    subgraph_nodes = [0] * len(graphs_single_robot)
    for i in range(len(graphs_single_robot)):
        all_pairs_shortest_paths_per_robot.append( \
            nx.johnson(graphs_single_robot[i], weight='weight'))
        subgraph_nodes[i] = random.sample(graphs_single_robot[i].nodes, math.ceil(len(graphs_single_robot[i].nodes)/10))
        if len(subgraph_nodes[i]) == 1: # single_source_bellman_ford gets stuck on single node graph, so this is an edge case
            all_pairs_shortest_paths_per_robot_b[i][subgraph_nodes[i][0]] = dict()
            all_pairs_shortest_paths_per_robot_b[i][subgraph_nodes[i][0]][subgraph_nodes[i][0]] = [subgraph_nodes[i][0]]
        else:
            for j in range(len(subgraph_nodes[i])):
                all_pairs_shortest_paths_per_robot_b[i][subgraph_nodes[i][j]] = nx.single_source_bellman_ford(graphs_single_robot[i], subgraph_nodes[i][j], target=None, weight='weight')[1]
        while len(all_pairs_shortest_paths_per_robot_b[i]) < len(graphs_single_robot[i].nodes):
            for node in list(all_pairs_shortest_paths_per_robot_b[i].keys()):
                N = drrt_ao.adj(graphs_single_robot[i], node)
                for neighbor in N:
                    if neighbor in all_pairs_shortest_paths_per_robot_b[i]:
                        continue
                    else:
                        all_pairs_shortest_paths_per_robot_b[i][neighbor] = dict()
                        for key in all_pairs_shortest_paths_per_robot_b[i][node].keys():
                            all_pairs_shortest_paths_per_robot_b[i][neighbor][key] = list(all_pairs_shortest_paths_per_robot_b[i][
                                node][key]) # attaching copy of the list of the neighbor
                            all_pairs_shortest_paths_per_robot_b[i][neighbor][key].insert(0, neighbor)
                            if key == neighbor:
                                all_pairs_shortest_paths_per_robot_b[i][neighbor][key] = all_pairs_shortest_paths_per_robot_b[i][neighbor][key][:1]
    for i in range(len(graphs_single_robot)): # A check to see the johnson and bellman ford dictionaries are equal
        for key in all_pairs_shortest_paths_per_robot[i].keys():
            if key not in all_pairs_shortest_paths_per_robot_b[i]:
                print("check out")
            else:
                for inkey in all_pairs_shortest_paths_per_robot[i][key].keys():
                    if inkey not in all_pairs_shortest_paths_per_robot_b[i][key]:
                        print("check in")
    return all_pairs_shortest_paths_per_robot_b


def makeHeuristic(graphs_single_robot, trees_single_robot):
    ts = time.time()
    all_pairs_shortest_paths_per_robot = compute_all_pair_shortest_path(graphs_single_robot, trees_single_robot)
    heuristic_obj = Heuristic(graphs_single_robot, all_pairs_shortest_paths_per_robot)
    tf = time.time()
    print("Heuristic measurment took: ", tf-ts)
    return heuristic_obj