#%%
import numpy as np
from scipy.optimize import linear_sum_assignment
import matplotlib.pyplot as plt
from random import random
import numpy as np
import json
import matplotlib.pyplot as plt
from functools import reduce


def euclidean_distance(p1, p2):
    """Compute the Euclidean distance between two points

    Args:
        p1 (tuple or list): point 1
        p2 (tuple or list): point 2

    Returns:
        float: Euclidean distance between p1 and p2
    """
    return np.linalg.norm(np.array(p1) - np.array(p2))



def lines_to_endpoints(lines):
    """extract endpoints from a list of polylines

    Args:
        lines (list of lists): list of polylines

    Returns:
        list of tuples: list of start, end point tuples, for each polyline pen stroke.
    """
    return [(tuple(line[0]), tuple(line[-1])) for line in lines]

def get_cost_matrix(stroke_segments):
    """Compute the cost matrix for the TSP problem"""
    n = len(stroke_segments)
    cost_matrix = np.zeros((2*n, 2*n))
    for i, (A1, B1) in enumerate(stroke_segments):
        a1_idx = 2*i
        b1_idx = 2*i + 1

        #no self connection = inf cost
        cost_matrix[a1_idx, a1_idx] = np.inf
        cost_matrix[b1_idx, b1_idx] = np.inf

        for j, (A2, B2) in enumerate(stroke_segments):
            a2_idx = 2*j
            b2_idx = 2*j + 1

            pt_idxs, pts = [a1_idx, b1_idx, a2_idx, b2_idx], [A1, B1, A2, B2]

            for src_idx, src_pt in zip(pt_idxs, pts):
                for dst_idx, dst_pt in zip(pt_idxs, pts):
                    cost =  euclidean_distance(src_pt, dst_pt)
                    if src_idx //2 == dst_idx // 2 and src_idx != dst_idx:
                        cost = 0
                    if src_idx == dst_idx:
                        continue
                    cost_matrix[src_idx, dst_idx] = cost
    return cost_matrix


# Nearest Neighbor TSP Solver
def tsp_nearest_neighbor(dist_matrix):
    """Solve the TSP problem using the nearest neighbor heuristic"""
    assert dist_matrix.shape[0] == dist_matrix.shape[1], "Distance matrix must be square"
    n = dist_matrix.shape[0]
    visited = [False] * n
    tour = [0]  # Start from the first city
    visited[0] = True
    total_distance = 0

    for _ in range(n - 1):
        current_city = tour[-1]
        next_city = np.argmin(
            [dist_matrix[current_city, j] if not visited[j] else np.inf for j in range(n)]
        )
        tour.append(next_city)
        total_distance += dist_matrix[current_city, next_city]
        visited[next_city] = True

    # Return to start city
    total_distance += dist_matrix[tour[-1], tour[0]]
    tour.append(0)
    tour = [int(d) for d in tour]

    return tour, total_distance

def stroke_dist(stroke):
    """Compute the total distance of a stroke"""
    substroke_dists = np.linalg.norm(stroke[:-1] - stroke[1:], axis=1).sum()
    return substroke_dists

def plot_lines(lines, ax=None, **kwargs):
    """Plot a list of polylines"""
    ax_was_none = ax is None
    if ax is None:
        fig, ax = plt.subplots()
    for line in lines:
        x, y = zip(*line)
        if 'c' not in kwargs:
            kwargs['c'] = 'k'

        ax.plot(x, y, **kwargs)

    if ax_was_none:
        return fig, ax
    
def tour_to_robot_waypoints(lines,
                            stroke_segments,
                            tour,
                            paper_height_fn=None,
                            pen_clearance=1.0,
                            paper_width=0.1,
                            paper_height=0.1,
                            xoffset=.25,
                            yoffset=0.05):
    """Convert a TSP tour to a list of robot waypoints"""
    assert paper_height_fn is not None, "paper_height_fn must be provided"
    pen_up_dists = []
    robot_waypoints = []

    for i, (j1, j2) in enumerate(zip(tour[:-1], tour[1:])):
        line_segment_idx1 = j1//2
        sub_field1 = j1%2

        line_segment_idx2 = j2//2
        sub_field2 = j2%2
            
        A = stroke_segments[line_segment_idx1][sub_field1]
        B = stroke_segments[line_segment_idx2][sub_field2]
        if line_segment_idx1 == line_segment_idx2: # this is a pen-down stroke.
            if sub_field1 > sub_field2:
                segment_waypoints = lines[line_segment_idx1][::-1]
            else:
                segment_waypoints = lines[line_segment_idx1]
            
            for waypoint in segment_waypoints: 
                a,b = waypoint
                robot_waypoints.append((a*paper_width + xoffset, b*paper_height + yoffset, paper_height_fn(*waypoint)))
        else:
            
            for (a, b) in [A,B]:
                robot_waypoints.append((a*paper_width + xoffset, b*paper_height + yoffset, paper_height_fn(*waypoint)+pen_clearance))

            pen_up_dists.append(euclidean_distance(A, B))

    return pen_up_dists, robot_waypoints

def plot_robot_waypoints(robot_waypoints, ax=None, paper_height_fn=None, **kwargs):
    """Plot the robot waypoints"""
    if paper_height_fn is None: # default to flat paper
        _, _, z = zip(*robot_waypoints)
        mean_z = np.mean(z)
        paper_height_fn = lambda x, y: mean_z

    ax_was_none = ax is None
    if ax is None:
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
    for w1, w2 in zip(robot_waypoints[:-1], robot_waypoints[1:]):
        if w1[2] == w2[2]:
            c = 'grey' if w1[2] <= paper_height_fn(w1[0], w1[1]) else 'pink'
            ax.plot3D([w1[0], w2[0]], [w1[1], w2[1]], [w1[2], w2[2]], linestyle='dashed', c=c)
        else:
             ax.plot3D([w1[0], w2[0]], [w1[1], w2[1]], [w1[2], w2[2]], linestyle='dotted', c='k')
             
    if ax_was_none:
        return fig, ax
        
# %%
