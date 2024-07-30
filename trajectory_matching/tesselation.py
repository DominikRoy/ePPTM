import math
import numpy as np


def compute_ref_capsule(trajectory: np.array, collision_threshold: int, max_gps_deviation: int):
    """
    Computes the reference capsule. Returns a 3-tuple containing the reference capsule: (h, r, theta),
    where h represents the length of the capsule, r represents the radius of the capsule, theta represents
    the angle of the capsule with respect to the x-axis.

    trajectory: a numpy array, containing tuples of (x,y) coordinates
    collision_threshold: the collision threshold delta, in grid units (TODO: determine what unit - cm?)
    max_gps_deviation: the maximum deviation of gps, in grid units (TODO: determine what unit - cm?)
    """
    try:
        x, y = trajectory.shape
        if y != 2:
            raise ValueError()
    except ValueError:
        raise ValueError(
            f"Provided trajectory must contain tuples of [x,y] values and therefore be of shape '(length, 2)' " +
            f"but is of shape '{trajectory.shape}'.") from None

    start_point = [trajectory[0][0], trajectory[0][1]]
    end_point = [trajectory[-1][0],trajectory[-1][1]]
    # Remove start and endpoint from trajectory to get the intermediate points.
    intermediate_points = np.delete(trajectory, [0, -1], axis=0)
    max_dist = 0
    # Compute the maximum distance of intermediate points to the line of symmetry.
    # Map all intermediate points to distances.
    distances = map(lambda p: distance_to_line(start_point, end_point, p), intermediate_points)
    # Now determine the maximum distance.
    for i in distances:
        # Take the point's distance to the line, discard the closest point on the line
        point_dist, _ = i
        max_dist = point_dist if point_dist > max_dist else max_dist
    # Compute the radius of the reference capsule.
    r = max_dist + collision_threshold + max_gps_deviation
    # Compute the reference length of the reference capsule
    dist_line = distance(start_point, end_point)
    h = dist_line + 2 * r
    # Compute the orientation of the reference capsule, i.e. the angle between the first and
    # the last point in the trajectory. If there is no line (startpoint and endpoint are equal),
    # determine angle to be 0.
    v_line = points_to_vector(start_point, end_point)
    theta = angle_of_vector(v_line)
    x,y = v_line
    #theta = math.atan(abs(y)/abs(x))
    theta_np = angle_of_vector_numpy(v_line)
    #print("start cap values:")
    #print(start_point)
    #print(end_point)
    #print("h:",{h})
    #print("radius",{r})
    #print("angle:",{theta})
    #print("distance start and end:",{dist_line})
    #print("endcap")
    return h, r, theta, theta_np # float("{:.3f}".format(theta))


def distance_to_line(line_start, line_end, p):
    """
    Computes the distance from point p to line l. Returns a tuple containing the distance and
    the point on the line which lies the closest to point p; i.e. (dist, [x,y]).

    line_start: point where the line starts
    line_end: point where the line ends
    point: the point p from which we compute the distance to the closest point on the line.
    """
    # if line is actually a point (line_start == line_end)
    if line_start[0] == line_end[0] and line_start[1] == line_end[1]:
        # return distance from line_start to point
        return distance(line_start, p), line_start
    # Create vectors; we now orient the space around line_vector; i.e. the vector starts
    # at (0, 0) but its orientation is maintained.
    line_vector = points_to_vector(line_start, line_end)
    point_vector = points_to_vector(line_start, p)
    # Compute length of line vector, convert to unit vector.
    line_length = length_of_vector(line_vector)
    unit_line_vector = as_unit_vector(line_vector)
    # Scale point_vector with respect to unitLine
    scaled_point_vector = scale_vector(point_vector, 1.0 / line_length)
    # Project scaled_point_vector on unitLine. If the projection lies beyond
    # the line (i.e. is bigger than 1, or smaller than 0), that means the 
    # nearest point is either the start of the line or the end of the line.
    # Therefore, snap the projection to max(0, proj) and min(1, proj)
    proj = dot_product(unit_line_vector, scaled_point_vector)
    proj = max(0, proj)
    proj = min(1, proj)
    # Now, compute the nearest point by multiplying the line_vector with the projection.
    # If the projection was bigger than the line itself, it is now 1 and the nearest point
    # is the end of the line. If it was smaller than the line, it is now 0 and the nearest
    # point is the beginning of the line.
    # Note that this "nearest point" is its location with respect to lineStart.
    nearest_point = scale_vector(line_vector, proj)
    dist = distance(nearest_point, point_vector)
    # Translate the nearest point back to the orientation of line l in the original 2D space.
    return dist, translate_to_point(nearest_point, line_start)


def lines_intersect(a_A, a_B, b_A, b_B):
    """
    Given the start point, end point of two line segments.
    Returns a boolean indicating whether the two line segments intersect.
    """
    def ccw(A, B, C):
        return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])
    return ccw(a_A, b_A, b_B) != ccw(a_B, b_A, b_B) and ccw(a_A, a_B, b_A) != ccw(a_A, a_B, b_B)


def capsules_intersect(a_A, a_B, a_radius, b_A, b_B, b_radius):
    """
    Given the start point, end point and radius of two capsules.
    Returns a boolean indicating whether the two capsules intersect.
    """
    # if lines intersect, return true
    if lines_intersect(a_A, a_B, b_A, b_B):
        return True
    # get distances between capsule endpoints
    d0, _ = distance_to_line(b_A, b_B, a_A)
    d1, _ = distance_to_line(b_A, b_B, a_B)
    d2, _ = distance_to_line(a_A, a_B, b_A)
    d3, _ = distance_to_line(a_A, a_B, b_B)
    # get minimum distance between two capsules
    min_dist = min(d0, d1, d2, d3)
    # get depth of capsules penetration
    penetration_depth = a_radius + b_radius - min_dist
    # if penetration depth positive -> the capsules intersect
    return penetration_depth > 0


def dot_product(v1, v2):
    """
    Computes the dot product v1 * v2.
    """
    x1, y1 = v1
    x2, y2 = v2
    return x1 * x2 + y1 * y2


def points_to_vector(p1, p2):
    """
    Computes a vector that points from point p1 to point p2.
    """
    x1, y1 = p1
    x2, y2 = p2
    return [x2 - x1, y2 - y1]


def is_zero_vector(v):
    """
    Determines whether vector v is a zero vector.
    """
    x, y = v
    return x == 0 and y == 0


def angle_of_vector(v):
    """
    Computes the angle of vector v in rad.
    """
    x, y = v
    # Feed the slope of the vector to inverse tangent function.
    return math.atan2(y, x)

def angle_of_vector_numpy(v):
    """
    Computes the angle of vector v in rad.
    """
    x, y = v
    # Feed the slope of the vector to inverse tangent function.
    return np.arctan2(y, x)


def length_of_vector(v):
    """
    Computes the length of vector v.
    """
    x, y = v
    return math.sqrt(math.pow(x, 2) + math.pow(y, 2))


def as_unit_vector(v):
    """
    Returns vector v as a unit vector; a vector with magnitude 1 of equal orientation and direction
    as vector v.
    """
    length = length_of_vector(v)
    if length == 0:
        raise ValueError("Cannot compute unit vector: length of vector is 0")
    x, y = v
    return [x / length, y / length]


def distance(p1, p2):
    """
    Computes the distance between point p1 and point p2.
    """
    return length_of_vector(points_to_vector(p1, p2))


def scale_vector(v, s):
    """
    Scales vector v with value s. Returns a scaled vector.
    """
    x, y = v
    return [x * s, y * s]


def translate_to_point(v, p):
    """
    Translates vector v to start at point p. Returns the new endpoint to which the translated vector points.
    """
    x1, y1 = v
    x2, y2 = p
    return [x1 + x2, y1 + y2]
