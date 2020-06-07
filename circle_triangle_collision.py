def is_circle_collide_triangle(circle_center, radius, triangle_vertexes):
    #
    # TEST 1: Vertex within circle
    #
    centrex, centrey = circle_center[0], circle_center[1]
    v1x, v1y, v2x, v2y, v3x, v3y = triangle_vertexes[0][0], triangle_vertexes[0][1], triangle_vertexes[1][0], triangle_vertexes[1][1],triangle_vertexes[2][0], triangle_vertexes[2][1],
    
    c1x = centrex - v1x
    c1y = centrey - v1y
    
    radiusSqr = radius*radius
    c1sqr = c1x*c1x + c1y*c1y - radiusSqr
    
    if c1sqr <= 0:
        return True
    
    c2x = centrex - v2x
    c2y = centrey - v2y
    c2sqr = c2x*c2x + c2y*c2y - radiusSqr
    
    if c2sqr <= 0:
        return True
    
    c3x = centrex - v3x
    c3y = centrey - v3y
    
    c3sqr = c3x*c3x + c3y*c3y - radiusSqr
    
    if c3sqr <= 0:
        return True
    
    
    #
    # TEST 2: Circle centre within triangle
    #
    
    #
    # Calculate edges
    #
    e1x = v2x - v1x
    e1y = v2y - v1y
    
    e2x = v3x - v2x
    e2y = v3y - v2y
    
    e3x = v1x - v3x
    e3y = v1y - v3y
    
    if (e1y*c1x - e1x*c1y) >= 0 | (e2y*c2x - e2x*c2y) >= 0 | (e3y*c3x - e3x*c3y) >= 0:
        return True
    
    
    #
    # TEST 3: Circle intersects edge
    #
    k = c1x*e1x + c1y*e1y
    
    if k > 0:
        length = e1x*e1x + e1y*e1y
        
        if k < length:
            if c1sqr * length <= k*k:
                return True
    
    # Second edge
    k = c2x*e2x + c2y*e2y
    
    if k > 0:
          length = e2x*e2x + e2y*e2y
        
          if k < length:
              if c2sqr * length <= k*k:
                  return True
    
    # Third edge
    k = c3x*e3x + c3y*e3y
    
    if k > 0:
          length = e3x*e3x + e3y*e3y
        
          if k < length:
              if c3sqr * length <= k*k:
                  return True
    
    # We're done, no intersection
    return False
