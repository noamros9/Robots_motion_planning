from arr2_epec_cs_ex import *


def copy_points(points):
    return [Point_2(point) for point in points]


def copy_polygons(polygons):
    return [Polygon_2(polygon) for polygon in polygons]


def copy_bonuses(bonuses):
    return [(Polygon_2(bonus[0]), bonus[1]) for bonus in bonuses]


def copy_robots(robots):
    return [[Point_2(robot[0]), float(robot[1])] for robot in robots]
