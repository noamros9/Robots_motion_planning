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
def is_point_valid_robot_robot(self, p, team_robots, radius):

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
