from arr2_epec_cs_ex import *
import math
from typing import List

N = 6


def approximate_cspace(obstacles: List[Polygon_2], radius: FT) -> Arrangement_2:
    p_base = Point_2(radius, FT(0))
    points = []
    angle = 0
    step = 2 * math.pi / N
    for i in range(N):
        s = math.sin(angle)
        c = math.cos(angle)
        t = Aff_transformation_2(Rotation(), Direction_2(c, s), FT(1), FT(1000))
        points.append(t.transform(p_base))
        angle += step
    approx_polygon: Polygon_2 = Polygon_2(points)

    traits = Arr_face_overlay_traits(lambda x, y: x + y)

    arrangements = []
    for obstacle in obstacles:
        ms = minkowski_sum_2(obstacle, approx_polygon)
        arr = Arrangement_2()
        # Arrangement for sum
        insert(arr, [Curve_2(edge) for edge in ms.outer_boundary().edges()])
        for hole in ms.holes():
            insert(arr, [Curve_2(edge) for edge in hole.edges()])
        for face in arr.faces():
            face.set_data(0)
        ubf = arr.unbounded_face()
        invalid_face = next(next(ubf.inner_ccbs())).twin().face()
        invalid_face.set_data(1)
        arrangements.append(arr)

    # overlay arrangements
    initial = Arrangement_2()
    ubf = initial.unbounded_face()
    ubf.set_data(0)
    arrangements.insert(0, initial)
    res = None
    for i in range(len(arrangements) - 1):
        res = Arrangement_2()
        overlay(arrangements[i], arrangements[i + 1], res, traits)
        arrangements[i + 1] = res

    cspace = res
    return cspace


if __name__ == "__main__":
    approximate_cspace([], FT(1))
