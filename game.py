from PyQt5.QtCore import QObject, pyqtSignal, QTimer
from PyQt5.QtWidgets import QFileDialog
from arr2_epec_cs_ex import *
import time
import importlib
from importlib import util
from gui.gui import GUI, QtCore, QtGui, QtWidgets, Qt, QPointF, QGraphicsLineItem
import Collision_detection
from Scene import Scene
import copy_list
import traceback



class WorkerSignals(QObject):
    finished = pyqtSignal()


class Worker(QtCore.QRunnable):
    def __init__(self, fn, *args):
        super(Worker, self).__init__()

        # Store constructor arguments (re-used for processing)
        self.fn = fn
        self.args = args
        self.signals = WorkerSignals()

    @QtCore.pyqtSlot()
    def run(self):
        '''
        Initialise the runner function with passed args, kwargs.
        '''

        # Retrieve args/kwargs here; and fire processing using them
        self.fn(*self.args)
        self.signals.finished.emit()  # Done


def set_up_scene():
    global game_over
    game_over = False
    gui.clear_scene()
    scene_file = gui.get_field(0)
    scene.load_scene(scene_file)
    print("loaded scene from", scene_file)

    if autoplay:
        gui.set_animation_finished_action(play_turn)  # Play the next turn automatically
        gui.set_button_text(1, "Play game")
        gui.set_logic(1, initialize)
    else:
        gui.set_animation_finished_action(enable_advance)  # Enable playing the next turn
        gui.set_button_text(1, "Unavailable")
        gui.set_button_text(2, "Advance")
        gui.set_logic(2, initialize)


def initialize():
    disable_advance()
    scene.path = []
    gui.empty_queue()
    if scene.turn % 2 == 0:
        path_name = gui.get_field(1)
        team_robots = scene.red_robots
        opponent_robots = scene.blue_robots
        team_objectives = scene.red_objectives
        opponent_objectives = scene.blue_objectives
        data = scene.red_data
    else:
        path_name = gui.get_field(2)
        team_robots = scene.blue_robots
        opponent_robots = scene.red_robots
        team_objectives = scene.blue_objectives
        opponent_objectives = scene.red_objectives
        data = scene.blue_data
    spec = importlib.util.spec_from_file_location(path_name, path_name)
    gp = util.module_from_spec(spec)
    spec.loader.exec_module(gp)
    params = [
        scene.R,
        scene.turn_time,
        scene.preprocess_time,
        scene.total_time,
        scene.D]

    params.extend([
        copy_list.copy_points([robot[0] for robot in team_robots]),
        copy_list.copy_points([robot[0] for robot in opponent_robots]),
        copy_list.copy_points(team_objectives),
        copy_list.copy_points(opponent_objectives),
        copy_list.copy_polygons(scene.obstacles),
        copy_list.copy_bonuses(scene.bonuses),
        data
    ])

    if scene.turn % 2 == 0:
        color = Qt.red
    else:
        color = Qt.blue
    gui.set_label(0, "Initializing: " + str(scene.turn), color)
    print("Player " + str(scene.turn % 2 + 1) + " initializing started")
    worker = Worker(gp.initialize, params)
    worker.signals.finished.connect(preprocess)
    global t
    t = scene.preprocess_time
    preprocess_countdown()
    global timer
    timer = QTimer()
    timer.setInterval(100)
    timer.timeout.connect(preprocess_countdown)
    timer.start()
    global t0
    t0 = time.perf_counter()
    threadpool.start(worker)

# for debugging
def display_graph(G, n):
    colors = [Qt.green, Qt.magenta, Qt.darkYellow]
    for item in gui.scene.items():
        if isinstance(item, QGraphicsLineItem):
            gui.scene.removeItem(item)
    for edge in list(G.edges):
        for i in range(n):
            s = (edge[0][i*2].to_double(), edge[0][i*2+1].to_double())
            t = (edge[1][i*2].to_double(), edge[1][i*2+1].to_double())
            gui.add_segment(*s, *t, line_color=colors[i % len(colors)])

def preprocess():
    global game_over
    t1 = time.perf_counter()
    turn_time = t1 - t0
    timer.stop()
    print("Pre-processing took:", turn_time, "seconds")
    if turn_time > scene.preprocess_time:
        print("Pre-processing took too long!")
        scene.path = []
        game_over = True
        gui.set_label(3, "Game ended")
        gui.set_label(4, "Player " + str((scene.turn + 1) % 2 + 1) + " wins!")
    else:
        # Uncomment to display the graph after pre-processing
        # G = scene.red_data[-5] # change this to where the graph is stored
        # display_graph(G, 1) # change "1" to the (effective) dimension of the points in the graph divided by 2
        scene.turn += 1
        print("Initializing done for player", 2 - (scene.turn % 2))
        if scene.turn < 0:
            initialize()
        else:
            scene.turn -= 1 # Since play turn raises scene.turn upon entering
            play_turn()


def play_turn():
    disable_advance()
    global game_over
    if game_over:
        return
    gui.set_label(3, "Remaining game time: " + str(round(scene.total_time, 2)))
    if scene.check_win_condition():
        game_over = True
        gui.set_label(3, "Game ended")
        return
    if scene.total_time < 0 or (scene.total_time < scene.turn_time and (scene.turn % 2) == 1):
        # Tie breaker
        game_over = True
        gui.set_label(3, "Game ended")
        gui.set_label(4, "")
        scene.determine_winner()
        return
    scene.turn += 1
    if scene.turn % 2 == 1:
        color = Qt.blue
    else:
        color = Qt.red
    gui.set_label(0, "Turn: " + str(scene.turn), color)
    gui.set_label(3, "Remaining game time: " + str(round(scene.total_time, 2)))
    scene.path = []
    if scene.turn % 2 == 0:
        path_name = gui.get_field(1)
        team_robots = scene.red_robots
        opponent_robots = scene.blue_robots
        data = scene.red_data
    else:
        path_name = gui.get_field(2)
        team_robots = scene.blue_robots
        opponent_robots = scene.red_robots
        data = scene.blue_data

    spec = importlib.util.spec_from_file_location(path_name, path_name)
    gp = util.module_from_spec(spec)
    spec.loader.exec_module(gp)
    params = [scene.path]

    params.extend([copy_list.copy_robots(team_robots),
                   copy_list.copy_robots(opponent_robots),
                   copy_list.copy_bonuses(scene.bonuses),
                   data,
                   scene.total_time])
    print("Player " + str(scene.turn % 2 + 1) + " turn started")
    worker = Worker(gp.play_turn, params)
    worker.signals.finished.connect(process_turn)
    global t
    t = scene.turn_time
    countdown()
    global timer
    timer = QTimer()
    timer.setInterval(100)
    timer.timeout.connect(countdown)
    timer.start()
    global t0
    t0 = time.perf_counter()
    threadpool.start(worker)


def process_turn():
    global game_over
    t1 = time.perf_counter()
    turn_time = t1 - t0
    timer.stop()
    print("Turn took:", turn_time, "seconds")
    if turn_time > scene.turn_time:
        print("Turn took too long!")
        scene.path = []
        game_over = True
        gui.set_label(3, "Game ended")
        gui.set_label(4, "Player " + str((scene.turn + 1) % 2 + 1) + " wins!")
    else:
        scene.total_time -= max(turn_time, 3.0)
        scene.process_turn()
        print("Done processing turn for player", (scene.turn % 2) + 1)
        animate_moves()
    if scene.turn % 2 == 0:
        gui.set_label(1, "Red team score: " + str(scene.red_score))
        # Uncomment to display the graph after each turn
        # G = scene.red_data[-5] # change this to where the graph is stored
        # display_graph(G, 1) # change "1" to the (effective) dimension of the points in the graph divided by 2
    else:
        gui.set_label(2, "Blue team score: " + str(scene.blue_score))


def getfile():
    dlg = QFileDialog()
    dlg.setFileMode(QFileDialog.AnyFile)
    if dlg.exec_():
        filenames = dlg.selectedFiles()
        return filenames[0]

def set_scene_file():
    s = getfile()
    if s:
        gui.set_field(0, s)

def set_red_file():
    s = getfile()
    if s:
        gui.set_field(1, s)

def set_blue_file():
    s = getfile()
    if s:
        gui.set_field(2, s)

def animate_moves():
    gui.play_queue()

def countdown():
    global t
    t = max(0, t - 0.1)
    gui.set_label(4, "Remaining turn time: " + str(round(t, 2)))
    gui.set_label(3, "Remaining game time: " + str(round(scene.total_time - scene.turn_time + t, 2)))

def preprocess_countdown():
    global t
    t = max(0, t - 0.1)
    gui.set_label(4, "Remaining pre-processing time: " + str(round(t, 2)))

def enable_advance():
    gui.set_button_text(2, "Advance")
    gui.set_logic(2, play_turn)


def disable_advance():
    gui.set_button_text(2, "Unavailable")
    gui.set_logic(2, lambda: None)


if __name__ == "__main__":
    scene_file = ""
    red_file = ""
    blue_file = ""
    autoplay = False
    import sys
    if len(sys.argv) >= 4:
        scene_file = sys.argv[1]
        red_file = sys.argv[2]
        blue_file = sys.argv[3]
    if len(sys.argv) >= 5:
        autoplay = True
    t0 = None
    t = None
    timer = QTimer()
    game_over = False
    app = QtWidgets.QApplication(sys.argv)
    gui = GUI()
    scene = Scene(gui)
    gui.set_program_name("Multi-robot Motion Planning Game")
    gui.set_field(0, scene_file)
    gui.set_field(1, red_file)
    gui.set_field(2, blue_file)
    gui.set_logic(0, set_up_scene)
    gui.set_logic(3, set_scene_file)
    gui.set_logic(4, set_red_file)
    gui.set_logic(5, set_blue_file)
    gui.set_button_text(0, "Load scene")
    gui.set_button_text(1, "Unavailable")
    gui.set_button_text(2, "Unavailable")
    gui.set_button_text(3, "...")
    gui.set_button_text(4, "...")
    gui.set_button_text(5, "...")
    gui.set_label(0, "Turn: -")
    gui.set_label(1, "Red team score: 0")
    gui.set_label(2, "Blue team score: 0")
    gui.set_label(3, "Remaining game time: ")
    gui.set_label(4, "Remaining turn time: ")
    gui.set_label(5, "Remaining travel distance: ")
    # gui.set_animation_finished_action(lambda: None)
    threadpool = QtCore.QThreadPool()
    gui.MainWindow.show()
    sys.exit(app.exec_())
