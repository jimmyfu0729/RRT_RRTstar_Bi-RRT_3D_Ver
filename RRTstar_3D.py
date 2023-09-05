'''
Finding the path from start to goal position in  3D space
using Rapidly exploring Random Tree (RRT)
'''
import pickle
import torch
import numpy as np
import math
# from Node3d import Node
from Node3d import Node
from retrace_path_3d import retrace_path
#from retrace_path import retrace_path
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


# Calculating heuristic using distance formula
def distance(point_a, point_b):
    return pow(pow(point_a.x - point_b.x, 2) + pow(point_a.y - point_b.y, 2) + pow(point_a.z - point_b.z, 2), 0.5)

# Calculating the nearest neighbor with low cost and forming a link
def findNeigh(dict_cost, new_pt, rad):
    # temp_list storing all the neighbors to the new_pt
    # within a distance of 0.4 units as defined by rad
    temp_list = []

    for var in dict_cost:
        if distance(var, new_pt) < rad:
            temp_list.append(var)

    temp_val = dict_cost[new_pt]

    for var in temp_list:
        if dict_cost[var] < temp_val:
            temp_val = dict_cost[var]
            temp_var = var

    return temp_var


# Calculating the shortest path
def finding_path(start, goal, n_x, n_y, n_z, vertices, intersection, obstacles,iter=1):
    # dict_parent to store the previous (parent) node
    # Searching radius should higher than delta_d.
    pt_list = [start]
    dict_parent = {}
    delta_d = 0.15
    dict_cost = {}
    rad = 0.3
    dict_cost[start] = 0
    path = []
    pt_valid = None
    pt_valid_array = np.zeros(len(obstacles))
    obs_indi = np.array(obstacles)
    x_range = obs_indi[:,0]
    y_range = obs_indi[:,1]
    z_range = obs_indi[:,2]
    path_vec_z = goal.z -start.z
    # generating the number of random vertices
    for i in range(vertices):
        # print(i)
        # Generating a random point
        rand_pt = Node(round(np.random.uniform(0, n_x), 3), round(np.random.uniform(0, n_y), 3), round(np.random.uniform(0, n_z), 3))

        # Finding the node in the open_list with minimum distance to the
        # random point
        temp_var = pt_list[0]
        temp_val = distance(temp_var, rand_pt)

        for var in pt_list:
            if distance(var, rand_pt) < temp_val:
                temp_val = distance(var, rand_pt)
                temp_var = var

        # Calculating the angle between the selected node with minimum
        # distance and the randomly generated point and expanding the selected node
        vec_x = rand_pt.x - temp_var.x
        vec_y = rand_pt.y - temp_var.y
        vec_z = rand_pt.z - temp_var.z
        theta = math.atan2(vec_y,vec_x)
        phi = math.acos(math.sqrt(vec_y**2+vec_x**2)/math.sqrt(vec_y**2+vec_x**2+vec_z**2))
        rd = np.random.uniform(0, 1) 
        if path_vec_z < 0:
            phi *= -1
        if (i>0)&(goal.z-delta_dz<0)&(path_vec_z < 0):
            phi = abs(phi)
        if (i>0)&(goal.z-delta_dz>0)&(path_vec_z < 0):
            phi *=-1
        if (i>0)&(goal.z-delta_dz<0)&(path_vec_z > 0):
            phi *=-1
        if (i>0)&(goal.z-delta_dz>0)&(path_vec_z > 0):
            phi = abs(phi)
        if rd < 0.5:
            phi = 0
        delta_dx = round(temp_var.x + delta_d * math.cos(phi)*math.cos(theta), 3)
        delta_dy = round(temp_var.y + delta_d * math.cos(phi)*math.sin(theta), 3)
        delta_dz = round(temp_var.z + delta_d * math.sin(phi), 3)

        new_pt = Node(delta_dx, delta_dy, delta_dz)
        # To check if the expanded point lies in the obstacle or not
        # if not, then add the point to the point list
        #TODO Temperally only for rectangular shape, might find another library or write a library to replace 2D shape library.
        pt_valid = True
            if {((new_pt.x -x_range[j,0] < -intersection) | (new_pt.x - x_range[j,1] > intersection ))
                &((new_pt.y - y_range[j,0] < -intersection)  | (new_pt.y - y_range[j,1] > intersection)) 
                & ((new_pt.z - z_range[j,0] < intersection) | (new_pt.z - z_range[j,1] > intersection))
                & (new_pt.z  >= intersection)}:
                pt_valid_array[j] = 1
                        
        for j in pt_valid_array:
            if j != 1:
                pt_valid = False
        
        if pt_valid == True:
            pt_list.append(new_pt)

            # Storing the parent node of the new node
            dict_cost[new_pt] = dict_cost[temp_var] + distance(temp_var, new_pt)  

            best_pt = findNeigh(dict_cost, new_pt, rad)
            plt.scatter([best_pt.x, new_pt.x], [best_pt.y, new_pt.y], [best_pt.z, new_pt.z], color='orange')
            dict_parent[new_pt] = best_pt

    # Defining a region to check if the current point is near to the
    # goal or not. If yes, then connect the current node to the goal
    max_val = max(dict_cost.values())

    for var in pt_list:
        if var == goal:
            path = retrace_path(dict_parent, goal, start)

        elif distance(var, goal) < 0.5 and dict_cost[var] <= max_val:
            pt_list.append(goal)
            # g_np_x = goal.x.to("cpu")
            # g_np_y = goal.y.to("cpu")
            # g_np_z = goal.z.to("cpu")
            plt.scatter([goal.x, var.x], [goal.y, var.y], [goal.z, var.z], color='orange')
            dict_parent[goal] = var
            max_val = dict_cost[var]
            path = retrace_path(dict_parent, goal, start)

    # Return the path by adding user defined vertices everytime until path found.
    if not path:
        print("Path not found: Increase  vertices by 1000")
        # print("Path not found: Increase the vertices")
        # return pt_list, None
        iter +=1
        if iter== 20:
            print("Path not found. Target might be generated inside the obstacles.")
            return None,None
        vertices += 1000
        pt_list, path= finding_path(start, goal, n_x, n_y, n_z, vertices, intersection, obstacles,iter)
        return pt_list, path
    else:
        print("Path found")
        return pt_list, path


# Initialize the input grid, start and end goal
# n_x, n_y, n_z represents the no. of x,y,z grids space.
target_pose = torch.tensor([5.0,12.5,0.45])
total_path = []
print(target_pose)
n_x, n_y, n_z = 20, 20, 20
start = Node(7.5, 10.0, 1.5)
goal = Node(target_pose[0].item(), target_pose[1].item(), target_pose[2].item())
vertices = 5000
# intersection is the distance that the node will be considered as inside the obstacle.
intersection = 0.2

fig = plt.figure()
ax = fig.add_subplot(projection='3d')
# Defining the rectangle obstacle
# The list should be [x_range, y_range, z_range]
obs_set =[]

#table2
obs_set.append([[9.8, 9.9], [4.6, 5.4], [0.0, 0.15]])
obs_set.append([[9.8, 9.9], [4.6, 5.4], [0.0, 0.7]])
obs_set.append([[9.8, 9.9], [5.3, 5.4], [0.0, 0.7]])
obs_set.append([[9.9, 10.75], [5.3, 5.4], [0.0, 0.15]])
obs_set.append([[9.8, 9.9], [4.6, 5.4], [0.7, 0.75]])
obs_set.append([[9.9, 10.75], [4.6, 4.7], [0.7, 0.75]])
obs_set.append([[10.65, 10.75], [4.6,4.7], [0.7, 0.75]])
obs_set.append([[9.9, 10.75], [5.3, 5.4], [0.7, 0.75]])
obs_set.append([[10.65, 10.75], [5.3, 5.4], [0.0, 0.7]])
obs_set.append([[10.65, 10.75], [4.6, 4.7], [0.0, 0.7]])
obs_set.append([[10.65, 10.75], [4.6, 4.7], [0.0, 0.15]])
obs_set.append([[9.8, 10.75], [4.6, 5.4], [0.75, 0.8]])

#table
obs_set.append([[9.8, 9.9], [14.6, 15.4], [0.0, 0.15]])
obs_set.append([[9.8, 9.9], [14.6, 15.4], [0.0, 0.7]])
obs_set.append([[9.8, 9.9], [15.3, 15.4], [0.0, 0.7]])
obs_set.append([[9.9, 10.75], [15.3, 15.4], [0.0, 0.15]])
obs_set.append([[9.8, 9.9], [14.6, 15.4], [0.7, 0.75]])
obs_set.append([[9.9, 10.75], [14.6, 14.7], [0.7, 0.75]])
obs_set.append([[10.65, 10.75], [14.6,14.7], [0.7, 0.75]])
obs_set.append([[9.9, 10.75], [15.3, 15.4], [0.7, 0.75]])
obs_set.append([[10.65, 10.75], [15.3, 15.4], [0.0, 0.7]])
obs_set.append([[10.65, 10.75], [14.6, 14.7], [0.0, 0.7]])
obs_set.append([[10.65, 10.75], [14.6, 14.7], [0.0, 0.15]])
obs_set.append([[9.8, 10.75], [14.6, 15.4], [0.75, 0.8]])

#boxes and brick
obs_set.append([[10.2, 0.4], [14.9, 15.1], [0.8, 0.89]])
obs_set.append([[10.1, 10.5], [4.8, 5.2], [0.0, 0.3]])
obs_set.append([[10.2, 10.4], [4.9, 5.1], [0.3, 0.4]])
x = np.zeros((len(obs_set),2))
y = np.zeros((len(obs_set),2))
z = np.zeros((len(obs_set),2))
obs_array = np.array(obs_set)
for i in range(len(obs_set)):
    buf_0 = obs_array[i,0,0].item()
    buf_1 = obs_array[i,0,1].item()
    x = np.linspace(obs_array[i,0,0],obs_array[i,0,1])
    y = np.linspace(obs_array[i,1,0],obs_array[i,1,1])
    z = np.array([[obs_array[i,2,0]],[obs_array[i,2,1]]])
    ax.plot_surface(x,y,z,color='cyan')
    
# Finding the final path and the navigated points
final_list, path = finding_path(start, goal, n_x, n_y, n_z, vertices, intersection, obs_set)
total_path.append(path)
    
if path :
    print("path:", len(path))
# print("list:",len(final_list))
# Plotting the overall search nodes
for var in final_list:
    ax.scatter(var.x, var.y, var.z, color='green', marker='o')

# Plotting the final path and the nodes from start to goal
if path is not None:
    for i in range(1, len(path)):
        ax.scatter(path[i].x, path[i].y, path[i].z, color='red', marker='x')
        ax.scatter([path[i].x, path[i - 1].x], [path[i].y, path[i - 1].y], [path[i].z, path[i - 1].z], color='red')

ax.scatter(start.x, start.y, start.z, color='blue', marker='o')
# g_np_x = goal.x.to("cpu")
# g_np_y = goal.y.to("cpu")
# g_np_z = goal.z.to("cpu")
ax.scatter(goal.x, goal.y, goal.z, color='purple', marker='o')

plt.show()
