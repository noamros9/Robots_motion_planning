from arr2_epec_cs_ex import *
import motion_planner
import approximate_cspace


def is_reachable(obstacles, radius, source, destination):
    # checks if source point is reachable from destination.
    # is used for checking availability of coupons.
    # source: Point_2, destination: Point_2
    cspace: Arrangement_2 = approximate_cspace.approximate_cspace(obstacles, radius)
    mp = motion_planner.Motion_planner(cspace)
    if mp.shortest_path(source, destination) == -1:
        return False
    return True
