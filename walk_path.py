from arr2_epec_cs_ex import *
import conversions
import random
from Collision_detection import is_step_collide, is_crossing_robot_walks


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
    turn_points = [list(team_robots.g_tensor.best_path[0])]
    for j in range(1, step_index+1, 1):
        point_i = list(team_robots.g_tensor.best_path[j])
        for i in range(len(team_robots.team_robots)):
            if path_len[i] < j:
                point_i[i] = team_robots.g_tensor.best_path[path_len[i]][i]
        turn_points.append(point_i)
    return turn_points


def is_identical_steps(step1, step2):
    # return True if step1 == step2, False otherwise
    for i in range(len(step1)):
        if step1[i].x().to_double() != step2[i].x().to_double() or step1[i].y().to_double() != step2[i].y().to_double():
            return False
    return True


def remove_identical_steps(path):
    final_path = [path[0]] + [path[i] for i in range(1, len(path)) if not is_identical_steps(path[i-1], path[i])]
    return final_path


def remove_crossing_steps(path, radius):
    # remove crossing steps
    # we may have steps in the path that robots cross each other
    # to be sure we don't have any collision, we separate each crossing step as follows:
    #   let's say a crosses b - first step: a moves, second step: b moves
    # remark: we also remove steps that are identical
    new_path = [list(path[0])]
    for step_index in range(1, len(path)):
        cur_step = path[step_index-1]
        for i in range(len(path[0])):
            for j in range(0, i):
                if is_crossing_robot_walks(path, step_index, i, j, radius):
                    new_path.append(cur_step.copy())
                    break
            cur_step[i] = path[step_index][i]
        new_path.append(cur_step.copy())
    # remove identical steps
    final_path = remove_identical_steps(new_path)
    return final_path


def get_turn_path(team_robots, path_len):
    # get the path of each robot we will make in this current turn
    path = get_path_points(team_robots, path_len)
    for i in range(len(path)):
        path[i] = [conversions.point_d_to_point_2(path[i][j]) for j in range(len(team_robots.team_robots))]
    path = remove_crossing_steps(path, team_robots.radius.to_double())
    return path


def update_best_path(team_robots, path_len):
    # update best_path by removing all the points that we already used
    first_index = min(path_len)
    team_robots.g_tensor.best_path = team_robots.g_tensor.best_path[first_index:]
    for i in range(len(team_robots.team_robots)):
        if path_len[i] != first_index:
            team_robots.g_tensor.best_path[0][i] = team_robots.g_tensor.best_path[1][i]


def change_best_path(team_robots, i, k, colliding_robot):
    # if it's the last point of the path, so we can't change it (we must get there). so we will just wait
    if k == len(team_robots.g_tensor.best_path) - 1:
        return True
    # let's say we are on point a, and we have the following path a -> b -> c where b is colliding with something.
    # b is approximately in middle of the road from a to c.
    # we will choose a random point b' and if it doesn't collide we will update the path to a -> b' -> c,
    # otherwise, we will not move in this turn/
    radius = team_robots.radius.to_double()
    cur_point = conversions.point_d_to_point_2(team_robots.g_tensor.best_path[k - 1][i])
    next_next_point = conversions.point_d_to_point_2(team_robots.g_tensor.best_path[k + 1][i])
    midx = (cur_point.x().to_double() + next_next_point.x().to_double()) / 2
    midy = (cur_point.y().to_double() + next_next_point.y().to_double()) / 2
    for j in range(10):
        randx = midx + (random.random() - 0.5) * radius
        randy = midy + (random.random() - 0.5) * radius
        rand_in_6d = Point_d(6, [FT(randx), FT(randy), FT(0), FT(0), FT(0), FT(0)])
        orig_copy = team_robots.g_tensor.best_path[k][i]
        team_robots.g_tensor.best_path[k][i] = rand_in_6d
        if is_step_collide(team_robots, i, k) is None:
            return False
        team_robots.g_tensor.best_path[k][i] = orig_copy
    return True
    #########################


def walk_best_path(team_robots, opponent_status):
    # update opponent robots
    for i in range(len(team_robots.opponent_robots)):
        team_robots.opponent_robots[i] = opponent_status[i][0]
    # walk on the best path and update it's starting points
    total_distance, k = 0, 1
    path_len = [0] * len(team_robots.team_robots)
    is_distance_left = True
    is_path_changed = [True] * len(team_robots.team_robots)
    while k < len(team_robots.g_tensor.best_path) and is_distance_left and any(is_path_changed):
        for i in range(len(team_robots.team_robots)):
            colliding_robot = is_step_collide(team_robots, i, k)
            if colliding_robot is not None:
                if not is_path_changed[i] or change_best_path(team_robots, i, k, colliding_robot):
                    continue
                is_path_changed[i] = False
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
