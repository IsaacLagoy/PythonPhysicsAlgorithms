import glm

# geometry functions
def inside_plane(point:glm.vec3, plane_point:glm.vec3, plane_normal:glm.vec3) -> bool:
    """determines if the point is inside the given plane"""
    return glm.dot(plane_normal, point - plane_point) >= 0

def line_intersect(point1:glm.vec3, point2:glm.vec3, plane_point:glm.vec3, plane_normal:glm.vec3) -> glm.vec3:
    """computes the line intersection of the line and plane"""
    line = point2 - point1
    # watch this line for division by zero
    t = glm.dot(plane_normal, plane_point - point1) / glm.dot(plane_normal, line)
    return point1 + t * line