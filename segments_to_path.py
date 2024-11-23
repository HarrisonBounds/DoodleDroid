#%%
import numpy as np
from scipy.optimize import linear_sum_assignment
import matplotlib.pyplot as plt
from random import random
def euclidean_distance(p1, p2):
    return np.linalg.norm(np.array(p1) - np.array(p2))
#%%
# List of line segments [(A, B), ...]
def get_line_seg():
    x,y = random(), random()
    dx, dy = random(), random()
    return ((x, y), (x+dx*0.1, y+dy*0.1))
line_segments = [get_line_seg() for _ in range(100)]


#%%
# Build cost matrix
n = len(line_segments)
cost_matrix = np.zeros((2*n, 2*n))
for i, (A1, B1) in enumerate(line_segments):
    a1_idx = 2*i
    b1_idx = 2*i + 1

    #no self connection = inf cost
    cost_matrix[a1_idx, a1_idx] = np.inf
    cost_matrix[b1_idx, b1_idx] = np.inf

    # cost_matrix[a1_idx, b1_idx] = 0
    # cost_matrix[b1_idx, a1_idx] = 0

    for j, (A2, B2) in enumerate(line_segments):
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

#%%
# Solve the TSP (Hungarian Algorithm for Assignment Problem as a simplification)
row_ind, col_ind = linear_sum_assignment(cost_matrix)
optimal_order = col_ind

print("Optimal Drawing Order:", optimal_order)
#%%
# Plot line segments
for A, B in line_segments:
    plt.plot([A[0], B[0]], [A[1], B[1]], 'bo-')
    
# Plot optimal order
for i, j in enumerate(optimal_order):
    line_segment_idx = j//2
    sub_field = j%2
    A = line_segments[line_segment_idx][sub_field]

    plt.text(A[0], A[1], str(i), fontsize=12, color='red')
# %%
import numpy as np
import matplotlib.pyplot as plt

# Generate random coordinates for cities
# np.random.seed(42)
# num_cities = 10
# cities = np.random.rand(num_cities, 2) * 100  # Random points in a 100x100 grid

# # Calculate distance matrix
# def distance_matrix(cities):
#     n = len(cities)
#     dist_matrix = np.zeros((n, n))
#     for i in range(n):
#         for j in range(n):
#             dist_matrix[i, j] = np.linalg.norm(cities[i] - cities[j])
#     return dist_matrix

# Nearest Neighbor TSP Solver
def tsp_nearest_neighbor(dist_matrix):
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

    return tour, total_distance

# Solve TSP
tour, total_distance = tsp_nearest_neighbor(cost_matrix)

# Print Results
print("Tour:", tour)
print("Total Distance:", total_distance)

for A, B in line_segments:
    plt.plot([A[0], B[0]], [A[1], B[1]], 'bo-')
    
# Plot optimal order
tour = [int(d) for d in tour]

for i, (j1, j2) in enumerate(zip(tour[:-1], tour[1:])):
    line_segment_idx1 = j1//2
    sub_field1 = j1%2

    line_segment_idx2 = j2//2
    sub_field2 = j2%2
    A = line_segments[line_segment_idx1][sub_field1]
    B = line_segments[line_segment_idx2][sub_field2]

    # plt.text(A[0], A[1], str(i), fontsize=12, color='red')
    plt.plot([A[0], B[0]], [A[1], B[1]], linestyle="dashed")


# %%
