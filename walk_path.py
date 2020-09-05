from arr2_epec_cs_ex import *
import conversions
import numpy as np


def best_path_step_distance(team_robots, i, k):
    # distance for the k'th step of robot number i
    d = (((conversions.point_d_to_point_2(team_robots.g_tensor.best_path[k - 1][i])).x().to_double() -
           (conversions.point_d_to_point_2(team_robots.g_tensor.best_path[k][i])).x().to_double()) ** 2 +
          ((conversions.point_d_to_point_2(team_robots.g_tensor.best_path[k - 1][i])).y().to_double() -
           (conversions.point_d_to_point_2(team_robots.g_tensor.best_path[k][i])).y().to_double()) ** 2) ** 0.5
    return d

# def update_best_path(team_robots):


def get_last_point(team_robots, path_len):
    # return the last point of the turn
    # we may have fuel only for a part of the the robots, so the other robots will stay in the same place
    step_index = max(path_len)
    last_point = []
    for i in range(team_robots.team_robots):
        if path_len[i] == step_index:
            last_point.append(team_robots.g_tensor.best_path[step_index][i])
        else:
            last_point.append(team_robots.g_tensor.best_path[step_index-1][i])
    return last_point


def get_turn_path(team_robots, path_len):
    # get the path of each robot we will make in this current turn
    path = [team_robots.g_tensor.best_path[i] for i in range(min(path_len))]
    if max(path_len) > min(path_len):
        path.append(get_last_point(team_robots, path_len))
    for i in range(len(path)):
        path[i] = [conversions.point_d_to_point_2(path[i][j]) for j in range(len(team_robots.team_robots))]
    return path


def update_best_path(team_robots, path_len):
    # update best_path by removing all the points that we already used
    first_index = min(path_len)
    team_robots.g_tensor.best_path = team_robots.g_tensor.best_path[first_index:]
    for i in range(len(team_robots.team_robots)):
        if path_len[i] != first_index:
            team_robots.g_tensor.best_path[0][i] = team_robots.g_tensor.best_path[0][i+1]


def is_collision_during_movement(opponent_robot, starting_p, ending_p, radius):
    # checks collision between the walk (from starting_p to ending_p) and the opponent_robot
    p1 = np.array([starting_p.x().to_double(), starting_p.y().to_double()])
    p2 = np.array([ending_p.x().to_double(), ending_p.y().to_double()])
    p3 = np.array([opponent_robot.x().to_double(), opponent_robot.y().to_double()])
    d = np.cross(p2 - p1, p3 - p1) / np.linalg.norm(p2 - p1)
    if d <= 2 * radius.to_double():
        return True
    return False


def is_opponent_collide(opponent_robots, starting_p, ending_p, radius):
    for i in range(len(opponent_robots)):
        x_diff = ending_p.x().to_double() - opponent_robots[i].x().to_double()
        y_diff = ending_p.y().to_double() - opponent_robots[i].y().to_double()
        d_diff = (x_diff ** 2 + y_diff ** 2) ** 0.5
        # check that the opponent robots do not collide with the final position ending_p
        if d_diff < 2 * radius.to_double():
            return True
        # check the edges do not collide
        if is_collision_during_movement(opponent_robots[i], starting_p, ending_p, radius):
            return True
    return False


def is_step_collide(team_robots, i, k):
    # return true if the k'th step of robot number i collide/not valid, false otherwise
    starting_p = (conversions.point_d_to_point_2(team_robots.g_tensor.best_path[k - 1][i]))
    ending_p = (conversions.point_d_to_point_2(team_robots.g_tensor.best_path[k][i]))
    opponent_robots = team_robots.opponent_robots
    radius = team_robots.radius
    if is_opponent_collide(opponent_robots, starting_p, ending_p, radius):
        return True
    return False


def change_best_path(team_robots, i, k):
    print("change best path")


def walk_best_path(team_robots):
    # walk on the best path and update it's starting points
    total_distance, k = 0, 1
    path_len = [0] * len(team_robots.team_robots)
    is_distance_left = True
    while k < len(team_robots.g_tensor.best_path) and is_distance_left:
        for i in range(len(team_robots.team_robots)):
            if is_step_collide(team_robots, i, k):
                change_best_path(team_robots, i, k)
                is_distance_left = False
                break
            d = best_path_step_distance(team_robots, i, k)
            if total_distance + d <= team_robots.distance_to_travel:
                total_distance = total_distance + d
                path_len[i] += 1
            else:
                is_distance_left = False
        k = k + 1

    path = get_turn_path(team_robots, path_len)
    update_best_path(team_robots, path_len)
    return path
