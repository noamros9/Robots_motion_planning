from arr2_epec_cs_ex import *
from gui.gui import GUI, QtWidgets
from networkx import *
import conversions
import math


def TPoint_to_Point_2(p: TPoint) -> Point_2:
    p.x().simplify()
    p.y().simplify()
    assert (p.x().is_extended() == False)
    assert (p.y().is_extended() == False)
    return Point_2(p.x().a0(), p.y().a0())


class Motion_planner:
    cspace = None
    G = None

    def __init__(self, cspace: Arrangement_2):
        self.cspace = cspace
        self.G: nx.DiGraph = nx.DiGraph()
        for v in self.cspace.vertices():
            self.G.add_node(TPoint_to_Point_2(v.point()))
        he: Halfedge
        for he in self.cspace.halfedges():
            if he.face().data() == 0:
                assert (he.curve().is_linear())
                c = he.curve()
                p1 = c.source()
                p1 = TPoint_to_Point_2(p1)
                p2 = c.target()
                p2 = TPoint_to_Point_2(p2)
                w = math.sqrt(squared_distance(p1, p2).to_double())
                self.G.add_edge(p1, p2, weight=w)
                self.G.add_edge(p2, p1, weight=w)

        nodes = list(self.G.nodes)
        for i in range(len(nodes)):
            for j in range(i+1, len(nodes)):
                v = nodes[i]
                u = nodes[j]
                res = []
                zone(self.cspace, X_monotone_curve_2(v, u), res)
                valid = True
                obj: Object
                for obj in res:
                    f = Face()
                    if obj.get_face(f):
                        if f.data() >= 1:
                            valid = False
                            break
                    # Check edge inside forbidden space
                    e = Halfedge()
                    if obj.get_halfedge(e):
                        if e.face().data() > 0 and e.twin().face().data() > 0:
                            valid = False
                            break
                if valid:
                    w = math.sqrt(squared_distance(v, u).to_double())
                    self.G.add_edge(v, u, weight=w)
                    self.G.add_edge(u, v, weight=w)


    def shortest_path(self, source: Point_2, destination: Point_2) -> float:
        if source == destination:
            return 0.0
        self.G.add_node(source)
        self.G.add_node(destination)
        nodes = list(self.G.nodes)
        for v in [source, destination]:
            for u in nodes:
                if u != v:
                    res = []
                    zone(self.cspace, X_monotone_curve_2(v, u), res)
                    valid = True
                    obj: Object
                    for obj in res:
                        f = Face()
                        if obj.get_face(f):
                            if f.data() >= 1:
                                valid = False
                                break
                        # Check edge inside forbidden space
                        e = Halfedge()
                        if obj.get_halfedge(e):
                            if e.face().data() > 0 and e.twin().face().data() > 0:
                                valid = False
                                break
                    if valid:
                        w = math.sqrt(squared_distance(v, u).to_double())
                        self.G.add_edge(v, u, weight=w)
                        self.G.add_edge(u, v, weight=w)
        if not (has_path(self.G, source, destination)):
            # self.display_cspace()
            # self.display_graph()
            self.G.remove_node(source)
            self.G.remove_node(destination)
            return -1
        length = shortest_path_length(self.G, source, destination, weight='weight')
        self.G.remove_node(source)
        if destination in self.G.nodes:
            self.G.remove_node(destination)
        return length

    def display_cspace(self):
        gui = GUI()
        for he in self.cspace.edges():
            s = conversions.point_2_to_xy(he.source().point())
            t = conversions.point_2_to_xy(he.target().point())
            gui.add_segment(*s, *t)

        gui.MainWindow.show()

    def display_graph(self):
        gui = GUI()
        for edge in self.G.edges():
            s = conversions.point_2_to_xy(edge[0])
            t = conversions.point_2_to_xy(edge[1])
            gui.add_segment(*s, *t)
        gui.MainWindow.show()

if __name__ == "__main__":
    arr = Arrangement_2()
    p0 = Point_2(0, 0)
    p1 = Point_2(1, 0)
    p2 = Point_2(1, 1)
    p3 = Point_2(0, 1)

    insert(arr, Curve_2(Segment_2(p0, p1)))
    insert(arr, Curve_2(Segment_2(p1, p2)))
    insert(arr, Curve_2(Segment_2(p2, p3)))
    insert(arr, Curve_2(Segment_2(p3, p0)))

    ubf: Face = arr.unbounded_face()
    ubf.set_data(0)
    for ccb in ubf.inner_ccbs():
        f = next(ccb).twin().face()
        f.set_data(1)

    for face in arr.faces():
        print(face.data())

    mp = Motion_planner(arr)
    # print(TPoint_to_Point_2(TPoint(FT(Gmpq(1,3)), FT(Gmpq(1,3)))))
    print(mp.shortest_path(Point_2(0, 0), Point_2(1, 1)))
