# Take from: https://codereview.stackexchange.com/questions/81865/travelling-salesman-using-brute-force-and-heuristics
# and modified to my needs
import copy


def optimized_travelling_salesman(points, start=None):
    """
    As solving the problem in the brute force way is too slow,
    this function implements a simple heuristic: always
    go to the nearest city.

    Even if this algoritmh is extremely simple, it works pretty well
    giving a solution only about 25% longer than the optimal one (cit. Wikipedia),
    and runs very fast in O(N^2) time complexity.
    """
    if start is None:
        start = points[0]

    # deep copy list as we are changing the list as we go
    must_visit = copy.deepcopy(points)

    path = [start]
    must_visit.remove(start)
    while must_visit:
        nearest = min(must_visit, key=lambda x: dist_squared(path[-1], x))
        path.append(nearest)
        must_visit.remove(nearest)
    return path


def dist_squared(point1, point2):
    t1 = point2.utm_x - point1.utm_x
    t2 = point2.utm_y - point1.utm_y
    return t1 ** 2 + t2 ** 2

