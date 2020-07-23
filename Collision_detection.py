from arr2_epec_cs_ex import *
from circle_triangle_collision import is_circle_collide_triangle
import conversions

class Collision_detector:
    cspace = None
    pl = None

    def __init__(self, obstacles, offset: FT):
        traits = Arr_face_overlay_traits(lambda x, y: x + y)

        arrangements = []
        # build an arrangement for each expanded polygon
        for polygon in obstacles:
            ms = approximated_offset_2(polygon, offset, 0.001)
            arr = Arrangement_2()
            # Arrangement for the sum
            insert(arr, [curve for curve in ms.outer_boundary().curves()])
            for hole in ms.holes():
                insert(arr, [curve for curve in hole.curves()])
            ubf = arr.unbounded_face()
            ubf.set_data(0)
            invalid_face = next(next(ubf.inner_ccbs())).twin().face()
            invalid_face.set_data(1)
            for ccb in invalid_face.inner_ccbs():
                valid_face = next(ccb).twin().face()
                valid_face.set_data(0)
            arrangements.append(arr)

        # overlay the arrangements
        initial = Arrangement_2()
        ubf = initial.unbounded_face()
        ubf.set_data(0)
        arrangements.insert(0, initial)
        res = None
        for i in range(len(arrangements) - 1):
            res = Arrangement_2()
            overlay(arrangements[i], arrangements[i + 1], res, traits)
            arrangements[i + 1] = res

        self.cspace = res
        self.pl = Arr_trapezoid_ric_point_location(self.cspace)

    def is_edge_valid(self, curve: Segment_2):
        res = []
        if curve.is_degenerate():
            return True
        zone(self.cspace, X_monotone_curve_2(curve.source(), curve.target()), res, self.pl)
        for obj in res:
            f = Face()
            if obj.get_face(f):
                if f.data() > 0:
                    return False
        return True

    def is_point_valid(self, p: Point_2):
        p = TPoint(p.x(), p.y())
        obj = self.pl.locate(p)
        f = Face()
        if obj.get_face(f):
            if f.data() > 0:
                return False
        return True


def check_intersection_against_robots(edges, opponent_robots, radius: FT):
    ms_squared_radius = FT(4) * radius * radius
    # Check intersection against opponent robots
    arr = Arrangement_2()
    for robot in opponent_robots:
        c = Circle_2(robot[0], ms_squared_radius, CLOCKWISE)
        insert(arr, Curve_2(c))
    edge: Segment_2
    for edge in edges:
        if not edge.is_degenerate() and do_intersect(arr, X_monotone_curve_2(edge.source(), edge.target())):
            return True
    # Check intersection between two robots while following the path
    for i in range(len(edges)):
        for j in range(i + 1, len(edges)):
            arr = Arrangement_2()
            c = Circle_2(edges[j].source(), ms_squared_radius, CLOCKWISE)
            insert(arr, Curve_2(c))
            v1 = Vector_2(edges[i])
            v2 = Vector_2(edges[j])
            v = v1 - v2
            cv = X_monotone_curve_2(edges[i].source(), edges[i].source() + v)
            if cv.source() != cv.target():
                if do_intersect(arr, cv):
                    return True
    return False


#   checks collision of the circle (with center p and radius r) with other team_robots
def is_point_valid_robot_robot(p, team_robots, radius):

    point_x, point_y = conversions.point_2_to_xy(p)
    for team_robot in team_robots:
        robot_x, robot_y = conversions.point_2_to_xy(team_robot)
        distX = point_x - robot_x
        distY = point_y - robot_y
        distance = ((distX*distX) + (distY*distY))**0.5
        if distance < 2*radius.to_double():
            return False
    return True

#   checks collision of the circle (with center p and radius r) with coupons
#   output: if there's a collision with a coupon with positive value - return true. return false otherwise


def is_collision_robot_coupons(coupons, p, radius):
    cd = Collision_detector(coupons, radius)
    return not cd.is_point_valid(p)

    #   checks collision of the circle (with center p and radius r) with coupons
    #   output: if there's a collision with a coupon with positive value - return the coupon. return None otherwise

def coupon_collide_robot(coupons, p, r):

    for coupon in coupons:
        if is_circle_collide_triangle(p, r, coupon):
            return coupon
    return None


def on_segment(p, q, r):
    if r[0] <= max(p[0], q[0]) and r[0] >= min(p[0], q[0]) and r[1] <= max(p[1], q[1]) and r[1] >= min(p[1], q[1]):
        return True
    return False


def orientation(p, q, r):
    val = ((q[1] - p[1]) * (r[0] - q[0])) - ((q[0] - p[0]) * (r[1] - q[1]))
    if val == 0:
        return 0
    return 1 if val > 0 else -1


# taken from https://kite.com/python/answers/how-to-check-if-two-line-segments-intersect-in-python
def lines_intersect(p1, q1, p2, q2):

    p1 = [p1.cartesian(0).to_double(), p1.cartesian(1).to_double()]
    p2 = [p2.cartesian(0).to_double(), p2.cartesian(1).to_double()]
    q1 = [q1.cartesian(0).to_double(), q1.cartesian(1).to_double()]
    q2 = [q2.cartesian(0).to_double(), q2.cartesian(1).to_double()]

    o1 = orientation(p1, q1, p2)
    o2 = orientation(p1, q1, q2)
    o3 = orientation(p2, q2, p1)
    o4 = orientation(p2, q2, q1)

    if o1 != o2 and o3 != o4:
        return True

    if o1 == 0 and on_segment(p1, q1, p2): return True
    if o2 == 0 and on_segment(p1, q1, q2): return True
    if o3 == 0 and on_segment(p2, q2, p1): return True
    if o4 == 0 and on_segment(p2, q2, q1): return True

    return False


# checks if there's no collision in the candidate edges
# need to check 1. the final position is not within radius of another robot
#               2. no collision during movement

def edges_collision_free(team, a, b):
    G = team.graphs_single_robots
    N = len(G)

    # check that the robots in V_new do not collide
    for i in range(N):
        for j in range(i+1, N):
            x_diff = b[i].cartesian(0).to_double() - b[j].cartesian(0).to_double()
            y_diff = b[i].cartesian(1).to_double() - b[j].cartesian(1).to_double()
            if (x_diff**2 + y_diff**2)**0.5 < 2 * team.radius.to_double():
                return False

    # check the edges do not colide (use orientation predicat)
    for i in range(N):
        for j in range(i+1, N):
            if lines_intersect(a[i], b[i], a[j], b[j]):
                return False

    return True
