from arr2_epec_cs_ex import *

class Collision_detector:
    cspace = None
    pl = None
    def __init__(self, obstacles, offset):
        traits = Arr_face_overlay_traits(lambda x, y: x + y)

        arrangements = []
        # build an arrangement for each expanded polygon
        for polygon in obstacles:
            #print("POLYGON in CD: ", polygon)
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

    def is_edge_valid(self, curve):
        res = []
        zone(self.cspace, X_monotone_curve_2(curve.source(), curve.target()), res, self.pl)
        for obj in res:
            f = Face()
            if (obj.get_face(f)):
                if (f.data() > 0):
                    return False
        return True

    def is_point_valid(self, p):
        tp = TPoint(p.x(), p.y())
        obj = self.pl.locate(tp)
        f = Face()
        if(obj.get_face(f)):
            if(f.data()>0):
                return False
        return True

    def is_point_valid_robot_robot(p, team_robots, radius):

        point_x, point_y = point_2_to_xy(p)
        for team_robot in team_robots:
            robot_x, robot_y = point_2_to_xy(team_robot)
            distX = point_x - robot_x
            distY = point_y - robot_y
            distance = ((distX*distX) + (distY*distY))**0.5
            if distance < 2*radius:
                return False
        return True
    
