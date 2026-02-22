#! /usr/bin/env python3

import cv2
import numpy as np
import copy
import csv 
import time

import numpy as np


def djikstra(node_list,node_distances_path,start_vertex_data):

    start_vertex = node_list.index(start_vertex_data)
    distances = [float('inf')] * len(node_list)
    distances[start_vertex] = 0
    predecessors = [None] * len(node_list)
    visited = [False] * len(node_list)
    distances_file = open(node_distances_path,mode = 'r')
    node_distances = distances_file.readlines()
    for _ in range(len(node_list)):
        min_distance = float('inf')
        u = None
        for i in range(len(node_list)):
            if not visited[i] and distances[i] < min_distance:
                min_distance = distances[i]
                u = i
        
        if u is None :
            break
    

        visited[u] = True 
        node_distances_data = line_to_float_multiarray(node_distances[u])

        for v in range(len(node_list)):
            if node_distances_data[v] != 0 and not visited[v]:
                alt = distances[u] + node_distances_data[v]
                if alt < distances[v]:
                    distances[v] = alt
                    predecessors[v] = u
        
    distances_file.close()
    return distances,predecessors



def string_to_float_multiarray(string):
    float_ = ''
    data = []
    for i in string:
        if i == ',' or i == ']' :
            if float_ == '' :
                pass
            else:
                data.append(float(float_))
                float_ = ''
        elif i == '[':
            pass
        else:
            float_ = float_ + i
    return data 

def line_to_float_multiarray(line):
    float_ = ''
    data = []
    for i in line :
        if i == ']':
            if float_ == '':
                pass
            else:
                data.append(float(float_))
            break
        if i == ',' :
            if float_ == '':
                pass
            else:
                data.append(float(float_))
                float_ = ''
        elif i == '[':
            pass
        else:
            float_ = float_ + i
    return data                

def get_path_refined(node_list,predecessors,start_vertex,end_vertex):
    path = []
    current = node_list.index(end_vertex)
    while current is not None:
        path.insert(0, node_list[current])
        current = predecessors[current]
        if current == node_list.index(start_vertex):
            path.insert(0, start_vertex)
            break
    return path  


def generate_reachable_pose(resolution1,resolution2,resolution3,resolution4,resolution5):
    reachable_points = []
    displacement = []
    first_element = True
    for i in range(0,181,resolution1):  ### First link
        A = np.array([[np.cos(i*np.pi/180), - np.sin(i*np.pi/180), 0,0],
                    [np.sin(i*np.pi/180), np.cos(i*np.pi/180), 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])
        T = np.array([[1,0,0,0.16934],
                    [0,1,0,0],
                    [0,0,1,0.111],
                    [0,0,0,1]])
        A = A@T
        for j in range(0,181,resolution2): ### Second link
            B = np.array([[1, 0, 0, 0],
                        [0, np.cos(j*np.pi/180), -np.sin(j*np.pi/180), 0],
                        [0, np.sin(j*np.pi/180), np.cos(j*np.pi/180), 0],
                        [0, 0, 0, 1]])
            T = np.array([[1,0,0,-0.12959],
                    [0,1,0,0],
                    [0,0,1,0.61182],
                    [0,0,0,1]])
            B = B@T
            for k in range(0,181,resolution3) : ### Last link
                C = np.array([[1, 0, 0, 0],
                            [0, np.cos(k*np.pi/180), -np.sin(k*np.pi/180), 0],
                            [0, np.sin(k*np.pi/180), np.cos(k*np.pi/180), 0],
                            [0, 0, 0, 1]])
                T = np.array([[1,0,0,0.11519],
                            [0,1,0,0],
                            [0,0,1,0.5722],
                            [0,0,0,1]])
                C = C@T
                for m in range(0,91,resolution4) : ### Last link
                    D = np.array([[1, 0, 0, 0],
                                [0, np.cos(m*np.pi/180), -np.sin(m*np.pi/180), 0],
                                [0, np.sin(m*np.pi/180), np.cos(m*np.pi/180), 0],
                                [0, 0, 0, 1]])
                    T = np.array([[1,0,0,0],
                                [0,1,0,0],
                                [0,0,1,0.11382],
                                [0,0,0,1]])
                    D = D@T
                    for n in range(0,91,resolution5) : ### Last link
                        E = np.array([[np.cos(n*np.pi/180), - np.sin(n*np.pi/180), 0,0],
                                    [np.sin(n*np.pi/180), np.cos(n*np.pi/180), 0, 0],
                                    [0, 0, 1, 0],
                                    [0, 0, 0, 1]])
                        T = np.array([[1,0,0,0.08247],
                                    [0,1,0,0],
                                    [0,0,1,0],
                                    [0,0,0,1]])
                        E = E@T
                        end_effector = A@B@C@D@E
                        pose = solve_for_orientation(end_effector)
                        if end_effector[0:3,3][2] >= 0 and pose != [] :
                            reachable_points.append(end_effector[0:3,3])
                            displacement.append([[i,j,k,m,n],end_effector[0:3,3],pose])
    return np.array(reachable_points), displacement
    

def node_classification_used_for_anchor(reachable_points,displacement):
    c = [[[[] for i in range(20)] for i in range(20)] for i in range(20)]

    for point, orientation in zip(reachable_points,displacement):
        i = int((point[0]+2)/0.2) 
        if i == 20 :
            i = 19
        j = int((point[1]+2)/0.2) 
        if j == 20 :
            j = 19
        k = int((point[2]+2)/0.2) 
        if k == 20 :
            k = 19 
        c[i][j][k].append(orientation)

    return c

def create_anchor_for_the_UR10(node_not_count_collided):
    reachable_points,displacement = generate_reachable_pose(30,15,15,30,30)
    c = node_classification_used_for_anchor(reachable_points,displacement)
    anchors = []
    for  i in range(20):
        for j in range(20):
            for k in range(20):
                if len(c[i][j][k]) != 0:
                    for point in c[i][j][k]:
                        if point[0] in node_not_count_collided:
                            anchors.append(point[0])
                            break

    return anchors


def solve_for_orientation(R):
    #------------------------------
    Solution = []
    possible_solutions = []
    alpha_possible_solution = [np.arctan(R[2][1]/R[2][2]),-np.pi + np.arctan(R[2][1]/R[2][2])]
    #print(alpha_possible_solution)
    beta_possible_solution = [np.arcsin(- R[2][0] ),np.pi - np.arcsin(- R[2][0] )]
    #print(beta_possible_solution)
    sigma_possible_solution = [np.arctan(R[1][0]/R[0][0]),-np.pi + np.arctan(R[1][0]/R[0][0])]
    #print(sigma_possible_solution)
    for alpha in alpha_possible_solution :
        for beta in beta_possible_solution: 
            for sigma in sigma_possible_solution : 
                possible_solutions.append([alpha,beta,sigma])
    #-----------------------------------
    for solution in possible_solutions :
        if abs(np.cos(solution[1])*np.cos(solution[2]) - R[0][0]) > 0.05:
            continue 
        elif abs(np.cos(solution[1])*np.sin(solution[2]) - R[1][0]) > 0.05:
            continue
        elif abs(np.sin(solution[0])*np.cos(solution[1]) - R[2][1]) > 0.05:
            continue
        elif abs(np.cos(solution[0])*np.cos(solution[1]) - R[2][2]) > 0.05:
            continue
        else:
            Solution = solution
    if Solution :       
        return [Solution[2],Solution[1],Solution[0]]
    else:
        return []



def main(args=None):

    node_list = []
    with open('/home/vb/control_robot_arm_with_node_graph/30_15_15_30_30/node.csv', mode ='r')as file:
        csvFile = csv.reader(file)
        for line in csvFile:
            for  i in line:
                node_list.append(string_to_float_multiarray(i))
    
    anchors = []
    for k in range(0,61,30):
        for j in range(0,61,30):
            for i in range(0,181,30):
                try :
                    node_list.index([i,j,k,0,0])
                    anchors.append([i,j,k,0,0])
                except ValueError:
                    pass
    print(len(anchors))
    start_time = time.time()
    pred_array = []
    for i,anchor in enumerate(anchors):
        distances,predecessors = djikstra(node_list,'/home/vb/control_robot_arm_with_node_graph/30_15_15_30_30/node_distances.txt',anchor)
        pred_array.append(predecessors)
    
    with open('/home/vb/control_robot_arm_with_node_graph/30_15_15_30_30/pred_array_avoid_obstacles.csv','w',newline = '') as csvfile_node:
        writer = csv.writer(csvfile_node)
        writer.writerows(pred_array)



    processing_time = time.time() - start_time
    print(processing_time)
    path = get_path_refined(node_list,predecessors,[0,0,0,0,0],[90,90,90,90,90])
    print(path)



if __name__ == '__main__':
    main()