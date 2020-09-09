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


def get_path_points(team_robots, path_len):
    # return the path points of the turn
    # remark: we may have fuel only for a part of the the robots, so the other robots will stay in the same place
    step_index = max(path_len)
    turn_points = [team_robots.g_tensor.best_path[0]]
    for j in range(1, step_index+1, 1):
        point_i = team_robots.g_tensor.best_path[j]
        for i in range(len(team_robots.team_robots)):
            if path_len[i] < j:
                point_i[i] = team_robots.g_tensor.best_path[path_len[i]][i]
        turn_points.append(point_i)
    return turn_points


def is_crossing_robot_walks(path, step_index, i, j):
    # return True if robot i collide with robot j on the road to path[step_index] point, False otherwise
    return False


def remove_crossing_steps(path):
    # remove crossing steps
    # we may have steps in the path that robots cross each other
    # to be sure we don't have any collision, we separate each crossing step as follows:
    #   let's say a crosses b - first step: a moves, second step: b moves
    new_path = [list(path[0])]
    print("{} new path".format(new_path))
    for step_index in range(1, len(path)):
        print("{} step index".format(step_index))
        new_step = path[step_index-1]
        for i in range(len(path[0])):
            for j in range(0, i):
                if is_crossing_robot_walks(path, step_index, i, j):
                    print(new_step)
                    new_path.append(list(new_step))
                    break
            new_step[i] = path[step_index][i]
        print(new_step)
        new_path.append(list(new_step))
    return new_path


def get_turn_path(team_robots, path_len):
    # get the path of each robot we will make in this current turn
    path = get_path_points(team_robots, path_len)
    for i in range(len(path)):
        path[i] = [conversions.point_d_to_point_2(path[i][j]) for j in range(len(team_robots.team_robots))]
    print(path)
    # path = remove_crossing_steps(path)
    print(path)
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
    d = abs(np.cross(p2 - p1, p1 - p3) / np.linalg.norm(p2 - p1))
    if d <= 2 * radius.to_double():
        if d == 0:
            if np.linalg.norm(p1 - p3) <= 2 * radius.to_double() or np.linalg.norm(p2 - p3) <= 2 * radius.to_double():
                print("hhhhhhhhhhhhhh")
                return True
            else:
                return False
        else:
            return True
    return False


def is_robots_collide(robots_arr, starting_p, ending_p, radius):
    for i in range(len(robots_arr)):
        x_diff = ending_p.x().to_double() - robots_arr[i].x().to_double()
        y_diff = ending_p.y().to_double() - robots_arr[i].y().to_double()
        d_diff = (x_diff ** 2 + y_diff ** 2) ** 0.5
        # check that the robots in the array do not collide with the final position ending_p
        if d_diff < 2 * radius.to_double():
            return True
        # check the edges do not collide
        # if is_collision_during_movement(robots_arr[i], starting_p, ending_p, radius):
        #     return True
    return False


def is_step_collide(team_robots, cur_team_position, i, k):
    # return true if the k'th step of robot number i collide/not valid, false otherwise
    starting_p = (conversions.point_d_to_point_2(team_robots.g_tensor.best_path[k - 1][i]))
    ending_p = (conversions.point_d_to_point_2(team_robots.g_tensor.best_path[k][i]))
    opponent_robots = team_robots.opponent_robots
    our_robots = team_robots.team_robots
    radius = team_robots.radius
    # collision with enemies
    if is_robots_collide(opponent_robots, starting_p, ending_p, radius):
        print("enemies")
        return True
    robot_friends = [our_robots[j] for j in range(len(our_robots)) if j != i]
    print("{} our robots".format(our_robots))
    print("{} robot friends".format(robot_friends))
    # collision with friends
    if is_robots_collide(robot_friends, starting_p, ending_p, radius):
        print("friends")
        return True
    return False


def change_best_path(team_robots, i, k):
    print("change best path")


def walk_best_path(team_robots):
    # walk on the best path and update it's starting points
    total_distance, k = 0, 1
    path_len = [0] * len(team_robots.team_robots)
    cur_team_position = team_robots.team_robots
    is_distance_left = True
    while k < len(team_robots.g_tensor.best_path) and is_distance_left:
        for i in range(len(team_robots.team_robots)):
            if is_step_collide(team_robots, cur_team_position, i, k):
                change_best_path(team_robots, i, k)
                is_distance_left = False
                break
            d = best_path_step_distance(team_robots, i, k)
            if total_distance + d <= team_robots.distance_to_travel:
                cur_team_position[i] = (conversions.point_d_to_point_2(team_robots.g_tensor.best_path[k][i]))
                total_distance = total_distance + d
                path_len[i] += 1
            else:
                is_distance_left = False
        k = k + 1
    path = get_turn_path(team_robots, path_len)
    update_best_path(team_robots, path_len)
    return path
