import time

from arr2_epec_cs_ex import *
from conversions import point_2_to_xy, tuples_list_to_polygon_2, polygon_2_to_tuples_list
import Collision_detection
import read_input
import motion_planner
import approximate_cspace
from gui.gui import GUI, QtCore, QtGui, QtWidgets, Qt, QPointF
import itertools
import math
from typing import List

colors = [Qt.yellow, Qt.green, Qt.red, Qt.blue, Qt.magenta]
OBJECTIVE_RADIUS = 0.1


class Scene():
    def __init__(self, gui: GUI):
        self.gui = gui
        self.reset()

    def reset(self):
        # Turn counter
        self.turn: int = -2
        # Turn time
        self.turn_time: float = 0.0
        # Preprocessing time
        self.preprocess_time: float = 0.0
        # Total time
        self.total_time: float = 0.0
        # Robot radius
        self.R: FT = FT(0)
        # Turn max distance
        self.D: float = 0.0
        # The robots are pairs of Point_2(center) and float (accumulated score)
        self.red_robots = []
        self.blue_robots = []
        # The objectives are Point_2 objects
        self.red_objectives: List[Point_2] = []
        self.blue_objectives: List[Point_2] = []
        # The bonuses are pairs of Polygon_2 and float (current value)
        self.bonuses = []
        self.total_bonuses_value: float = 0.0
        # The obstacles are Polygon_2 objects
        self.obstacles: List[Polygon_2] = []
        self.gui_red_robots = []
        self.gui_blue_robots = []
        self.gui_objectives = []
        self.gui_bonuses = []
        self.gui_obstacles = []
        # To be filled by the two teams each turn
        self.path = []
        self.red_score: float = 0
        self.blue_score: float = 0

        # Collision detector for intersection with obstacles
        self.collision_detector = None
        # Collision detectors for intersections with bonuses
        self.expanded_bonuses = []

        self.red_data = []
        self.blue_data = []

        self.gui.set_label(0, "Turn: -")
        self.gui.set_label(1, "Red team score: 0")
        self.gui.set_label(2, "Blue team score: 0")
        self.gui.set_label(3, "Remaining game time: ")
        self.gui.set_label(4, "Remaining turn time: ")

    def draw_scene(self):
        self.gui.empty_queue()
        self.gui.clear_scene()
        self.gui_robots = []
        for robot in self.red_robots:
            self.gui_red_robots.append(
                self.gui.add_disc_robot(self.R.to_double(), *point_2_to_xy(robot[0]), fill_color=Qt.red))
        for robot in self.blue_robots:
            self.gui_blue_robots.append(
                self.gui.add_disc_robot(self.R.to_double(), *point_2_to_xy(robot[0]), fill_color=Qt.blue))
        for objective in self.red_objectives:
            self.gui_objectives.append(
                self.gui.add_disc(OBJECTIVE_RADIUS, *point_2_to_xy(objective), fill_color=Qt.red))
        for objective in self.blue_objectives:
            self.gui_objectives.append(
                self.gui.add_disc(OBJECTIVE_RADIUS, *point_2_to_xy(objective), fill_color=Qt.blue))
        for obstacle in self.obstacles:
            self.gui_obstacles.append(
                self.gui.add_polygon([point_2_to_xy(p) for p in obstacle.vertices()], Qt.darkGray))
        for bonus in self.bonuses:
            p = self.gui.add_polygon([point_2_to_xy(p) for p in bonus[0].vertices()], Qt.yellow)
            fp = next(bonus[0].vertices())
            x = fp.x().to_double()
            y = fp.y().to_double()
            t = self.gui.add_text(str(bonus[1]), x, y, 1)
            self.gui_bonuses.append((p, t))

    def load_scene(self, filename):
        scene = read_input.read_scene(filename)
        self.reset()
        N = scene[0]
        obstacles = scene[1]
        bonuses = scene[2]
        self.R = scene[3]
        self.turn_time = scene[4]
        self.preprocess_time = scene[5]
        self.total_time = scene[6]
        self.D = scene[7]
        base = 8
        for i in range(base, base + N):
            self.red_robots.append([scene[i], 0.0])
        base += N
        for i in range(base, base + N):
            self.blue_robots.append([scene[i], 0.0])
        base += N
        for i in range(base, base + N):
            self.red_objectives.append(scene[i])
        base += N
        for i in range(base, base + N):
            self.blue_objectives.append(scene[i])
        base += N
        for i in range(base, base + obstacles):
            self.obstacles.append(scene[i])
        base += obstacles
        for i in range(base, base + bonuses):
            self.bonuses.append(scene[i])
            self.total_bonuses_value += scene[i][1]
        self.build_ex_sets()

        self.draw_scene()

    def build_ex_sets(self):
        self.collision_detector = Collision_detection.Collision_detector(self.obstacles, self.R)
        for i in range(len(self.bonuses)):
            self.expanded_bonuses.append([Collision_detection.Collision_detector([self.bonuses[i][0]], self.R), i])
        pass

    def is_step_valid(self, edges, d):
        # check that d is large enough
        s = 0
        for edge in edges:
            s += (edge.squared_length().to_double()) ** 0.5
        if s > d:
            print("Travel distances exceed D")
            return False
        # check intersection with robots
        if self.turn % 2 == 0:
            opponent_robots = self.blue_robots
        else:
            opponent_robots = self.red_robots
        if Collision_detection.check_intersection_against_robots(edges, opponent_robots, self.R):
            print("Intersection with robots")
            return False
        # check intersection with obstacles
        for edge in edges:
            if not self.collision_detector.is_edge_valid(edge):
                print("Intersection with obstacles")
                return False
        return True

    def process_turn(self):
        robots = None
        gui_robots = None
        d = self.D
        if self.turn % 2 == 0:
            robots = self.red_robots
            gui_robots = self.gui_red_robots
        else:
            robots = self.blue_robots
            gui_robots = self.gui_blue_robots
        for step in self.path:
            edges = []
            for i in range(len(robots)):
                edges.append(Segment_2(robots[i][0], step[i]))

            if self.is_step_valid(edges, d):
                # Update d
                s = 0
                for edge in edges:
                    s += (edge.squared_length().to_double()) ** 0.5
                d -= s

                # Collect bonuses
                bonus_anims = []
                for i in range(len(robots)):
                    edge = edges[i]
                    for expanded_bonus in self.expanded_bonuses:
                        bonus = self.bonuses[expanded_bonus[1]]
                        if not bonus[1] == 0:
                            if not expanded_bonus[0].is_edge_valid(edge):
                                if self.turn % 2 == 0:
                                    self.red_score += bonus[1]
                                else:
                                    self.blue_score += bonus[1]
                                robots[i][1] += bonus[1]
                                gui_robots[i].set_text(str(int(robots[i][1])))
                                bonus[1] = 0
                                bonus_anims.append(
                                    self.gui.visibility_animation(self.gui_bonuses[expanded_bonus[1]][0], False))
                                bonus_anims.append(
                                    self.gui.visibility_animation(self.gui_bonuses[expanded_bonus[1]][1], False))

                # Queue animations
                for i in range(len(robots)):
                    robots[i][0] = step[i]
                anims = []
                for i in range(len(robots)):
                    edge = edges[i]
                    anims.append(self.gui.linear_translation_animation
                                 (gui_robots[i], *point_2_to_xy(edge.source()), *point_2_to_xy(edge.target())
                                                                              , duration=2000/len(self.path)))
                anim = self.gui.parallel_animation(*anims)
                self.gui.queue_animation(anim)
                anim = self.gui.parallel_animation(*bonus_anims)
                self.gui.queue_animation(anim)
            else:
                break

    def check_win_condition(self) -> bool:
        if self.check_win_condition_for_team(self.red_robots, self.red_objectives, self.red_score):
            print("Player 1 wins!")
            self.gui.set_label(4, "Player 1 wins!")
            return True
        elif self.check_win_condition_for_team(self.blue_robots, self.blue_objectives, self.blue_score):
            print("Player 2 wins!")
            self.gui.set_label(4, "Player 2 wins!")
            return True
        return False

    def check_win_condition_for_team(self, robots, objectives: List[Point_2], team_score: float):
        if 2 * team_score >= self.total_bonuses_value:
            for robot in robots:
                flag: bool = False
                for objective in objectives:
                    if robot[0] == objective:
                        flag = True
                        break
                if not flag:  # No objective matches robot location
                    return False
            return True
        return False  # Score is less than 0.5 of total bonuses score

    def min_distance_to_destinations(self, robots, objectives, motion_planner):
        potential_scores = []
        scores = {}
        for i in range(len(robots)):
            scores[i] = {}
            for objective in objectives:
                d = motion_planner.shortest_path(robots[i][0], objective)
                scores[i][objective] = d
        for permutation in (itertools.permutations(objectives)):
            score = 0.0
            for i in range(len(robots)):
                d = scores[i][permutation[i]]
                if d == -1:
                    score = math.inf
                    break
                else:
                    score += d
            potential_scores.append(score)
        min_score = min(potential_scores)
        return min_score
        # TODO normalize?

    def determine_winner(self):
        print("Calculating final score for each team")
        cspace: Arrangement_2 = approximate_cspace.approximate_cspace(self.obstacles, self.R)
        mp = motion_planner.Motion_planner(cspace)
        red_min_dist: float = self.min_distance_to_destinations(self.red_robots, self.red_objectives, mp)
        blue_min_dist: float = self.min_distance_to_destinations(self.blue_robots, self.blue_objectives, mp)
        perimeter = self.scene_perimeter()
        # Final score is max(0, (2*scene bbox perimeter*num_robots_per_team - min_distance_to_objectives)) + collected bonuses
        red_score = max(0.0, 2*perimeter*len(self.red_robots) - red_min_dist) + self.red_score
        blue_score = max(0.0, 2*perimeter*len(self.blue_robots) - blue_min_dist) + self.blue_score
        print("Player 1 score:", red_score)
        print("Player 2 score:", blue_score)
        if red_score > blue_score:
            print("Player 1 wins!")
            self.gui.set_label(4, "Player 1 wins!")
        elif red_score < blue_score:
            print("Player 2 wins!")
            self.gui.set_label(4, "Player 2 wins!")
        else:
            print("Tie!")
            self.gui.set_label(4, "Tie!")

    def scene_perimeter(self):
        perimeter = 0.0
        x_min = min(obstacle.left_vertex().x() for obstacle in self.obstacles)
        x_max = max(obstacle.right_vertex().x() for obstacle in self.obstacles)
        y_min = min(obstacle.bottom_vertex().y() for obstacle in self.obstacles)
        y_max = max(obstacle.top_vertex().y() for obstacle in self.obstacles)
        perimeter += 2*((y_max-y_min).to_double() + (x_max-x_min).to_double())
        return perimeter
