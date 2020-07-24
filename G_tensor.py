from arr2_epec_cs_ex import *
import networkx as nx
from rrt_single_robot import Dynamic_kd_tree
import conversions
import Collision_detection
import math
import heuristic

class G_tensorMap:
    def __init__(self, N, start, team, heuristic_obj):
        self.path = []
        self.team = team
        self.heuristic_obj = heuristic_obj
        self.G = team.graphs_single_robots
        self.confg_cnt = 0
        self.configs = dict()
        self.node_configs = [dict() for i in range(N)]
        self.initialize(N, start)
        self.best_path = []


    # g_tensor is the tensor road map, it contains Kd_tree for searching and regular networkxs
    # graph for other operations. NOTE that the DiGraph is contains exactly the same points as
    # the kd_tree. The dictionary will contain the parent of every node, which by reverse walking
    # is the shortest path to root, and the cose of the SHORTEST path from start to the node
    def initialize(self, N, start):
        self.path = [[nx.DiGraph(), Dynamic_kd_tree()] for i in range(N)]
        robot_in_6d = [0 for i in range(N)]
        for i in range(N):
            robot_i_x, robot_i_y = conversions.point_2_to_xy(start[i])
            robot_in_6d[i] = Point_d(6, [FT(robot_i_x), FT(robot_i_y), FT(0), FT(0), FT(0), FT(0)])
            self.path[i][0].add_node(robot_in_6d[i])
            self.path[i][0].add_weighted_edges_from([(robot_in_6d[i], robot_in_6d[i], 0)])
            self.path[i][1].insert(robot_in_6d[i])  # inserting the starting position of each robot
            self.node_configs[i][robot_in_6d[i]] = [self.confg_cnt]
        self.configs[self.confg_cnt] = {'nodes': robot_in_6d, 'cost': 0, 'parent': None}
        self.confg_cnt += 1

    def add_new_config(self, a, b):
        edge_weight_sum = 0
        for i in range(len(self.G)):
            if a[i] == b[i]:
                edge_weight = 0
                self.node_configs[i][b[i]].append(self.confg_cnt)
            else:
                edge_weight = self.G[i][a[i]][b[i]]['weight']
                # add node and edge to the graph
                if not self.path[i][0].has_node(b[i]):
                    self.path[i][0].add_node(b[i])
                    self.path[i][0].add_weighted_edges_from([(a[i], b[i], edge_weight), (b[i], a[i], edge_weight)])
                    # add point to the kd-tree
                    self.path[i][1].insert(b[i])
                    # add that node is in the configuration
                    self.node_configs[i][b[i]] = [self.confg_cnt]
                else:
                    self.node_configs[i][b[i]].append(self.confg_cnt)
                    if not self.path[i][0].has_edge(a[i], b[i]):
                        self.path[i][0].add_weighted_edges_from([(a[i], b[i], edge_weight), (b[i], a[i], edge_weight)])
            edge_weight_sum += edge_weight

        # add to the dictionary. The shortest path to b must be through a
        aID = self.config_by_nodes(a)
        cost_to_b_through_a = edge_weight_sum + self.configs[aID]['cost']
        self.configs[self.confg_cnt] = {'nodes': b, 'cost': cost_to_b_through_a, 'parent': aID}
        self.confg_cnt += 1

    # check if we should change parent(b) to a or leave it as is. Note that a, b are neighbors
    def rewire(self, a, b):
        N = len(self.G)
        aID = self.config_by_nodes(a)
        bID = self.config_by_nodes(b)
        if aID == bID:
            return
        # compute whether we should change the parent
        curr_cost = self.configs[bID]['cost']  # current cost of getting to configuration b
        edge_weight_sum = 0
        for i in range(N):
            if a[i] != b[i]:
                edge_weight_sum += self.G[i][a[i]][b[i]]['weight']
        cost_to_b_through_a = self.configs[aID]['cost'] + edge_weight_sum
        # if so - does it collisions free?
        if cost_to_b_through_a < curr_cost:
            if Collision_detection.edges_collision_free(self.team, a, b): # update parents and costs
                self.configs[bID]['parent'] = aID
                self.configs[bID]['cost'] = cost_to_b_through_a

    # given nodes, return the config ID
    # note that we use it only when we know for sure that the configuration exists
    def config_by_nodes(self, nodes):
        for configID in self.configs:
            if self.configs[configID]['nodes'] == nodes:
                return configID
        return -1  # not exist

    # finds the nearest configuration to a point near which is based on Q_rand
    def nearest_config(self, Q_rand):
        N = len(self.G)
        min_config = -1
        min_cost = math.inf
        near = [self.team.trees_single_robots[i].nearest_neighbor(Q_rand[i]) for i in range(N)]
        for configID in self.configs:
            config = self.configs[configID]
            cost = sum([heuristic.calc_heur(self.heuristic_obj, i, None, config['nodes'][i], near[i]) \
                        for i in range(N)])
            if cost < min_cost:
                min_cost = cost
                min_config = configID
        return self.configs[min_config]['nodes']

    # some costs may be not updated, we update it forward
    def update_costs(self):
        for i in range(1, self.confg_cnt): # no need to update the start configuration
            curr_cost = self.configs[i]['cost']
            cost_to_parent = self.configs[self.configs[i]['parent']]['cost']
            nodes_curr = self.configs[i]['nodes']
            nodes_parent = self.configs[self.configs[i]['parent']]['nodes']
            cost_parent_to_config = 0
            for j in range(len(self.G)):
                if nodes_parent[j] != nodes_curr[j]:
                    cost_parent_to_config += self.path[j][0][nodes_parent[j]][nodes_curr[j]]['weight']
            total_cost = cost_to_parent + cost_parent_to_config
            if total_cost < curr_cost:
                self.configs[i]['cost'] = cost_to_parent + cost_parent_to_config

    # returns the path from the specified configuration to the root
    def build_path(self, config):
        path = []
        while config != None:
            path.append(self.configs[config]['nodes'])
            config = self.configs[config]['parent']
        path = path[::-1]
        return path