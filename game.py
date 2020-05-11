from arr2_epec_cs_ex import *
import time
import importlib
from gui.gui import GUI, QtCore, QtGui, QtWidgets, Qt, QPointF
import read_input
from conversions import point_2_to_xy, tuples_list_to_polygon_2, polygon_2_to_tuples_list
import Collision_detection
import copy_list

colors = [Qt.yellow, Qt.green, Qt.red, Qt.blue, Qt.magenta]
OBJECTIVE_RADIUS = 0.1

class Polygons_scene():
    def __init__(self):
        self.reset()

    def reset(self):
        self.turn = 0
        self.time = 0
        self.R = 0
        self.D = 0
        self.red_robots = []
        self.blue_robots = []
        self.red_objectives = []
        self.blue_objectives = []
        self.bonuses = []
        self.obstacles = []
        self.gui_red_robots = []
        self.gui_blue_robots = []
        self.gui_objectives = []
        self.gui_bonuses = []
        self.gui_obstacles = []
        self.path = []

        self.collision_detector = None
        self.expanded_bonuses = []

        self.red_data = []
        self.blue_data = []

    def draw_scene(self):
        gui.empty_queue()
        gui.clear_scene()
        self.gui_robots = []
        for robot in self.red_robots:
            self.gui_red_robots.append(gui.add_disc(self.R.to_double(), *point_2_to_xy(robot[0]), fill_color=Qt.red))
        for robot in self.blue_robots:
            self.gui_blue_robots.append(gui.add_disc(self.R.to_double(), *point_2_to_xy(robot[0]), fill_color=Qt.blue))
        for objective in self.red_objectives:
            self.gui_objectives.append(gui.add_disc(OBJECTIVE_RADIUS, *point_2_to_xy(objective), fill_color=Qt.red))
        for objective in self.blue_objectives:
            self.gui_objectives.append(gui.add_disc(OBJECTIVE_RADIUS, *point_2_to_xy(objective), fill_color=Qt.blue))
        for obstacle in self.obstacles:
            self.gui_obstacles.append(gui.add_polygon([point_2_to_xy(p) for p in obstacle.vertices()], Qt.darkGray))
        for bonus in self.bonuses:
            self.gui_bonuses.append(gui.add_polygon([point_2_to_xy(p) for p in bonus[0].vertices()], Qt.yellow))

    def load_scene(self, filename):
        scene = read_input.read_scene(filename)
        self.reset()
        N = scene[0]
        obstacles = scene[1]
        bonuses = scene[2]
        self.R = scene[3]
        self.time = scene[4]
        self.D = scene[5]
        base = 6
        for i in range(base, base+N):
            self.red_robots.append([scene[i], 0.0])
        base += N
        for i in range(base, base+N):
            self.blue_robots.append([scene[i], 0.0])
        base += N
        for i in range(base, base+N):
            self.red_objectives.append(scene[i])
        base += N
        for i in range(base, base+N):
            self.blue_objectives.append(scene[i])
        base += N
        for i in range(base, base+obstacles):
            self.obstacles.append(scene[i])
        base += obstacles
        for i in range(base, base+bonuses):
            self.bonuses.append(scene[i])
        self.build_ex_sets()

        self.draw_scene()

    def build_ex_sets(self):
        self.collision_detector = Collision_detection.Collision_detector(self.obstacles, self.R)
        for i in range(len(self.bonuses)):
            self.expanded_bonuses.append([Collision_detection.Collision_detector([self.bonuses[i][0]], self.R), i])
        pass

    def is_step_valid(self, edges, d):
        #check that D is large enough
        s = 0
        for edge in edges:
            s += (edge.squared_length().to_double())**0.5
        if s > d:
            return False
        #check intersection with robots
        #check intersection with obstacles
        for edge in edges:
            if(not self.collision_detector.is_edge_valid(edge)):
                return False
        return True

    def process_turn(self):
        robots = None
        gui_robots = None
        d = self.D
        if(self.turn%2 == 0):
            robots = self.red_robots
            gui_robots = self.gui_red_robots
        else:
            robots = self.blue_robots
            gui_robots = self.gui_blue_robots
        for step in self.path:
            edges = []
            for i in range(len(robots)):
                edges.append(Segment_2(robots[i][0], step[i]))

            if(self.is_step_valid(edges, d)):
                #Update d
                s = 0
                for edge in edges:
                    s += (edge.squared_length().to_double()) ** 0.5
                d -= s

                #Collect bonuses
                bonus_anims = []
                for i in range(len(robots)):
                    edge = edges[i]
                    for expanded_bonus in self.expanded_bonuses:
                        bonus = self.bonuses[expanded_bonus[1]]
                        if not bonus[1] == 0:
                            if not expanded_bonus[0].is_edge_valid(edge):
                                robots[i][1] += bonus[1]
                                bonus[1] = 0
                                bonus_anims.append(gui.visibility_animation(self.gui_bonuses[expanded_bonus[1]], False))

                #Queue animations
                for i in range(len(robots)):
                    robots[i][0] = step[i]
                anims = []
                for i in range(len(robots)):
                    edge = edges[i]
                    anims.append(gui.linear_translation_animation(gui_robots[i], *point_2_to_xy(edge.source()), *point_2_to_xy(edge.target())))
                anim = gui.parallel_animation(*anims)
                gui.queue_animation(anim)
                anim = gui.parallel_animation(*bonus_anims)
                gui.queue_animation(anim)
        #gui.play_queue()
        #gui.empty_queue()
        self.turn += 1
        print("Done processing turn for player" , 2-(self.turn % 2))

def set_up_scene():
    gui.clear_scene()
    scene_file = gui.get_field(0)
    ps.load_scene(scene_file)
    initialize()
    initialize()
    print("loaded scene from", scene_file)


def initialize():
    ps.path = []
    gui.empty_queue()
    if(ps.turn%2 == 0):
        path_name = gui.get_field(1)
        team_robots = ps.red_robots
        opponent_robots = ps.blue_robots
        team_objectives = ps.red_objectives
        opponent_objectives = ps.blue_objectives
        data = ps.red_data
    else:
        path_name = gui.get_field(2)
        team_robots = ps.blue_robots
        opponent_robots = ps.red_robots
        team_objectives = ps.blue_objectives
        opponent_objectives = ps.red_objectives
        data = ps.blue_data
    print("1")
    print(path_name)
    gp = importlib.import_module(path_name)
    print("2")
    params =  [ps.time,
               float(ps.D),
               FT(ps.R)]

    params.extend([
        copy_list.copy_points([robot[0] for robot in team_robots]),
        copy_list.copy_points([robot[0] for robot in opponent_robots]),
        copy_list.copy_points(team_objectives),
        copy_list.copy_points(opponent_objectives),
        copy_list.copy_polygons(ps.obstacles),
        copy_list.copy_bonuses(ps.bonuses),
        data
    ])

    gp.initialize(params)
    ps.turn += 1
    print("Initializing done for player", 2-(ps.turn%2))

def play_turn():
    ps.path = []
    #gui.empty_queue()
    if(ps.turn%2 == 0):
        path_name = gui.get_field(1)
        team_robots = ps.red_robots
        opponent_robots = ps.blue_robots
        data = ps.red_data
    else:
        path_name = gui.get_field(2)
        team_robots = ps.blue_robots
        opponent_robots = ps.red_robots
        data = ps.blue_data

    gp = importlib.import_module(path_name)
    params =  [ps.path,
               ps.time,
               float(ps.D)]

    params.extend([copy_list.copy_robots(team_robots),
                   copy_list.copy_robots(opponent_robots),
                   copy_list.copy_bonuses(ps.bonuses),
                   data])
    gp.play_turn(params)
    ps.process_turn()

def animate_moves():
    gui.play_queue()

if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    gui = GUI()
    ps = Polygons_scene()
    gui.set_program_name("Multi-robot Motion Planning Game")
    gui.set_field(0, "scene0.txt")
    gui.set_field(1, "red")
    gui.set_field(2, "blue")
    gui.set_logic(0, set_up_scene)
    gui.set_button_text(0, "Load scene")
    gui.set_logic(1, play_turn)
    gui.set_button_text(1, "Play turn")
    #gui.set_logic(2, load_path)
    gui.set_button_text(2, "Unused")
    gui.set_logic(3, animate_moves)
    gui.set_button_text(3, "Animate moves")
    #gui.set_logic(4, is_path_valid)
    gui.set_button_text(4, "Unused")
    gui.set_button_text(5, "Unused")
    gui.set_button_text(6, "Unused")
    gui.set_button_text(7, "Unused")
    gui.MainWindow.show()
    sys.exit(app.exec_())