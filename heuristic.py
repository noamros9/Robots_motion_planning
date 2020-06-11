from arr2_epec_cs_ex import *
import networkx as nx
import conversions

class Heuristic():
    def __init__(self, graphs_single_robot, all_pairs_shortest_paths_per_robot):
        self.graphs_single_robot = graphs_single_robot
        self.all_pairs_shortest_paths_per_robot = all_pairs_shortest_paths_per_robot



def calc_heur(heuristic_obj, i, v_near, neighbor, objective):
    ed = Euclidean_distance()
    v_near_to_neighbor = (ed.transformed_distance(v_near, neighbor).to_double())**0.5

    neighbor_to_objective_path_len = 0
    objective_6d = conversions.point_2_to_point_d(objective)
    neighbor_to_objective_path = heuristic_obj.all_pairs_shortest_paths_per_robot[i][neighbor][objective_6d]
    for i in range(1,len(neighbor_to_objective_path)):
        neighbor_to_objective_path_len += \
            (ed.transformed_distance(neighbor_to_objective_path[i], neighbor_to_objective_path[i-1]).to_double())**0.5
    return neighbor_to_objective_path_len


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