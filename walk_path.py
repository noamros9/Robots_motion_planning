from arr2_epec_cs_ex import *
import conversions
import numpy as np
import random

def best_path_step_distance(best_path, i, k):
    # distance for the k'th step of robot number i
    d = (((conversions.point_d_to_point_2(best_path[k - 1][i])).x().to_double() -
           (conversions.point_d_to_point_2(best_path[k][i])).x().to_double()) ** 2 +
          ((conversions.point_d_to_point_2(best_path[k - 1][i])).y().to_double() -
           (conversions.point_d_to_point_2(best_path[k][i])).y().to_double()) ** 2) ** 0.5
    return d


def get_path_points(best_path, path_len):
    # return the path points of the turn
    # remark: we may have fuel only for a part of the the robots, so the other robots will stay in the same place
    step_index = max(path_len)
    turn_points = [list(best_path[0])]
    for j in range(1, step_index+1, 1):
        point_i = list(best_path[j])
        for i in range(len(best_path[0])):
            if path_len[i] < j:
                point_i[i] = best_path[path_len[i]][i]
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
    for step_index in range(1, len(path)):
        new_step = path[step_index-1]
        for i in range(len(path[0])):
            for j in range(0, i):
                if is_crossing_robot_walks(path, step_index, i, j):
                    new_path.append(list(new_step))
                    break
            new_step[i] = path[step_index][i]
        new_path.append(list(new_step))
    return new_path


def get_turn_path(best_path, path_len):
    # get the path of each robot we will make in this current turn
    path = get_path_points(best_path, path_len)
    for i in range(len(path)):
        path[i] = [conversions.point_d_to_point_2(path[i][j]) for j in range(len(best_path[0]))]
    # path = remove_crossing_steps(path)
    return path


def update_best_path(best_path, path_len):
    # update best_path by removing all the points that we already used
    first_index = min(path_len)
    for i in range(first_index):
        del best_path[0]
    for i in range(len(best_path[0])):
        if path_len[i] != first_index:
            best_path[0][i] = best_path[1][i]


def is_collision_during_movement(opponent_robot, starting_p, ending_p, radius):
    # checks collision between the walk (from starting_p to ending_p) and the opponent_robot
    p1 = np.array([starting_p.x().to_double(), starting_p.y().to_double()])
    p2 = np.array([ending_p.x().to_double(), ending_p.y().to_double()])
    p3 = np.array([opponent_robot.x().to_double(), opponent_robot.y().to_double()])
    d = abs(np.cross(p2 - p1, p1 - p3) / np.linalg.norm(p2 - p1))
    if d <= 2 * radius.to_double():
        if d == 0:
            if np.linalg.norm(p1 - p3) <= 2 * radius.to_double() or np.linalg.norm(p2 - p3) <= 2 * radius.to_double():
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
            return robots_arr[i]
        # check the edges do not collide
        # if is_collision_during_movement(robots_arr[i], starting_p, ending_p, radius):
        #     return True
    return None


def is_step_collide(team_robots, best_path, i, k):
    # return true if the k'th step of robot number i collide/not valid, false otherwise
    starting_p = (conversions.point_d_to_point_2(best_path[k - 1][i]))
    ending_p = (conversions.point_d_to_point_2(best_path[k][i]))
    opponent_robots = team_robots.opponent_robots
    our_robots = team_robots.team_robots
    radius = team_robots.radius
    # collision with enemies
    colliding_robot = is_robots_collide(opponent_robots, starting_p, ending_p, radius)
    if colliding_robot is not None:
        return colliding_robot
    robot_friends = [our_robots[j] for j in range(len(our_robots)) if j != i]
    # collision with friends
    colliding_robot = is_robots_collide(robot_friends, starting_p, ending_p, radius)
    if colliding_robot is not None:
        return colliding_robot
    return None


def change_best_path(team_robots, best_path, i, k, colliding_robot):
    # if it's the last point of the path, so we can't change it (we must get there). so we will just wait
    if k == len(best_path) - 1:
        return True
    # let's say we are on point a, and we have the following path a -> b -> c where b is colliding with something.
    # b is approximately in middle of the road from a to c.
    # we will choose a random point b' and if it doesn't collide we will update the path to a -> b' -> c,
    # otherwise, we will not move in this turn/
    radius = team_robots.radius.to_double()
    cur_point = conversions.point_d_to_point_2(best_path[k - 1][i])
    next_next_point = conversions.point_d_to_point_2(best_path[k + 1][i])
    midx = (cur_point.x().to_double() + next_next_point.x().to_double()) / 2
    midy = (cur_point.y().to_double() + next_next_point.y().to_double()) / 2
    for j in range(10):
        randx = midx + (random.random() - 0.5) * radius
        randy = midy + (random.random() - 0.5) * radius
        rand_in_6d = Point_d(6, [FT(randx), FT(randy), FT(0), FT(0), FT(0), FT(0)])
        orig_copy = best_path[k][i]
        best_path[k][i] = rand_in_6d
        if is_step_collide(team_robots, best_path, i, k) is None:
            print("change", flush=True)
            return False
        best_path[k][i] = orig_copy
    print("back to original", flush=True)
    return True
    #########################


def walk_best_path(team_robots, best_path, opponent_status):
    # update opponent robots
    for i in range(len(team_robots.opponent_robots)):
        team_robots.opponent_robots[i] = opponent_status[i][0]
    # walk on the best path and update it's starting points
    total_distance, k = 0, 1
    path_len = [0] * len(team_robots.team_robots)
    is_distance_left = True
    is_path_changed = [True] * len(team_robots.team_robots)
    is_continue = True
    while k < len(best_path) and is_distance_left and is_continue:
        for i in range(len(team_robots.team_robots)):
            colliding_robot = is_step_collide(team_robots,best_path, i, k)
            if colliding_robot is not None:
                if not is_path_changed[i] or change_best_path(team_robots, best_path, i, k, colliding_robot):
                    is_continue = False
                    continue
                is_path_changed[i] = False
            d = best_path_step_distance(best_path, i, k)
            if total_distance + d <= team_robots.distance_to_travel:
                total_distance = total_distance + d
                path_len[i] += 1
            else:
                is_distance_left = False
        k = k + 1
    path = get_turn_path(best_path, path_len)
    update_best_path(best_path, path_len)
    return path
