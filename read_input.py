from arr2_epec_cs_ex import FT, Gmpq, Point_2, Polygon_2


def coordinate_to_FT(coord):
  if ("." in coord):
    return FT(Gmpq(float(coord)))
  else:
    return FT(Gmpq(coord))

def read_scene(filename):
  out = []
  with open(filename, "r") as f:
    for line in f:
      line = line.strip()
      input_data = line.split(" ")
      if input_data[0] == 'N':
        out.append(int(input_data[1]))
      elif input_data[0] == 'OBSTACLES':
        out.append(int(input_data[1]))
      elif input_data[0] == 'BONUSES':
        out.append(int(input_data[1]))
      elif input_data[0] == 'RADIUS':
        out.append(coordinate_to_FT(input_data[1]))
      elif input_data[0] == 'TIME':
        out.append(int(input_data[1]))
      elif input_data[0] == 'PREPROCESS_TIME':
        out.append(int(input_data[1]))
      elif input_data[0] == 'TOTAL_TIME':
        out.append(int(input_data[1]))
      elif input_data[0] == 'D':
        out.append(float(input_data[1]))
      elif len(input_data) == 2:
        out.append(Point_2(coordinate_to_FT(input_data[0]), coordinate_to_FT(input_data[1])))
      elif input_data[0] == 'O':
        polygon = []
        for i in range(0, int(input_data[1])):
          polygon.append(Point_2(coordinate_to_FT(input_data[2 * i + 2]), coordinate_to_FT(input_data[2 * i + 3])))
        polygon = Polygon_2(polygon)
        if not polygon.is_counterclockwise_oriented():
          polygon.reverse_orientation()
        out.append(polygon)
      elif input_data[0] == 'B':
        polygon = []
        for i in range(0, int(input_data[1])):
          polygon.append(Point_2(coordinate_to_FT(input_data[2 * i + 2]), coordinate_to_FT(input_data[2 * i + 3])))
        polygon = Polygon_2(polygon)
        if not polygon.is_counterclockwise_oriented():
          polygon.reverse_orientation()
        out.append([polygon, int(input_data[-1])])
  return out

def save_path(path, filename):
  file = open(filename, 'w')
  for i in range(len(path)):
    p = path[i]
    x = p.x().exact()
    y = p.y().exact()
    line = str(x.numerator()) + '/' + str(x.denominator()) + ' ' + str(y.numerator()) + '/' + str(y.denominator())
    if i < len(path) - 1:  line = line + '\n'
    file.write(line)
  file.close()

def load_path(path, filename, number_of_robots):
  with open(filename, 'r') as file:
    for line in file:
      coords = line.split(" ")
      tup = []
      for i in range(number_of_robots):
        x = coordinate_to_FT(coords[i*2])
        y = coordinate_to_FT(coords[i*2 + 1])
        p = Point_2(x, y)
        tup.append(p)
      path.append(tup)
