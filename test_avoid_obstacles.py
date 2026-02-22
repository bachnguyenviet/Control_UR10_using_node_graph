#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import copy
import csv 

from std_msgs.msg import String
from std_msgs.msg import Bool 
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Pose

import numpy as np


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



class UR_Node_Full:
    def __init__(self, node,
                 range_of_each_link,
                 resolution_of_each_link):
        #print(node)
        self.num_of_state_in_each_node = len(node)
        self.node = node
        self.neighboring_node = []
        self.range = range_of_each_link
        self.res = resolution_of_each_link

    def add_neighboring_node(self):
        neighbor_node = []
        in_index = [0 for i in range(self.num_of_state_in_each_node)]
        while True:
            node = []
            for i in range(self.num_of_state_in_each_node):
                a = [self.node[i] - self.res[i], self.node[i],self.node[i] + self.res[i]][in_index[i]]
                if a < self.range[i][0] or a > self.range[i][1]:
                    pass
                else:
                    node.append(a)
            if len(node) == self.num_of_state_in_each_node and node != self.node:
                neighbor_node.append(node) 
            if in_index == [2 for i in range(self.num_of_state_in_each_node)]:
                break
            in_index = increment_array_of_int(in_index)

        self.neighboring_node = neighbor_node
                

def increment_array_of_int(a):
    r = a
    for i in range(len(a) - 1, -1, -1):
        if a[i] == 0:
            r[i] = 1
            break
        elif a[i] == 1:
            r[i] = 2
            break
        elif a[i] == 2:
            r[i] = 0
    return r

def calculate_distance(point1,point2):
    return ((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2 + (point1[2] - point2[2])**2)**0.5


def calculate_end_effector_UR5(i,j,k,m,n):

    A = np.array([[np.cos(i*np.pi/180), - np.sin(i*np.pi/180), 0,0],
                  [np.sin(i*np.pi/180), np.cos(i*np.pi/180), 0, 0],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]])
    T = np.array([[1,0,0,0.16934],
                  [0,1,0,0],
                  [0,0,1,0.111],
                  [0,0,0,1]])
    A = A@T
    
    B = np.array([[1, 0, 0, 0],
                  [0, np.cos(j*np.pi/180), -np.sin(j*np.pi/180), 0],
                  [0, np.sin(j*np.pi/180), np.cos(j*np.pi/180), 0],
                  [0, 0, 0, 1]])
    T = np.array([[1,0,0,-0.12959],
                  [0,1,0,0],
                  [0,0,1,0.61182],
                  [0,0,0,1]])
    B = B@T

    C = np.array([[1, 0, 0, 0],
                  [0, np.cos(k*np.pi/180), -np.sin(k*np.pi/180), 0],
                  [0, np.sin(k*np.pi/180), np.cos(k*np.pi/180), 0],
                  [0, 0, 0, 1]])
    T = np.array([[1,0,0,0.11519],
                  [0,1,0,0],
                  [0,0,1,0.5722],
                  [0,0,0,1]])
    C = C@T

    D = np.array([[1, 0, 0, 0],
              [0, np.cos(m*np.pi/180), -np.sin(m*np.pi/180), 0],
              [0, np.sin(m*np.pi/180), np.cos(m*np.pi/180), 0],
              [0, 0, 0, 1]])
    T = np.array([[1,0,0,0],
                  [0,1,0,0],
                  [0,0,1,0.11382],
                  [0,0,0,1]])
    D = D@T

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
    
    return end_effector[0:3,3],pose



class Testing_msg(Node):

    def __init__(self,node_list,pred_array,anchors):
        super().__init__('testing_node')
        self.publisher_ = self.create_publisher(String, '/link_displacement', 10)
        self.subscription_ = self.create_subscription(
            Bool,
            '/next_location_now',
            self.listener_callback,
            10) 
        self.subscription_location = self.create_subscription(
            String,
            '/item_location',
            self.listener_callback_location,
            10)
        self.node_list = node_list
        self.pred_array = pred_array        
        self.path_index = 0
        self.reachable_pose, self.displacement = generate_reachable_pose(30,15,15,30,30) 
        self.reachable_pose_classify = node_classification(self.reachable_pose,self.displacement)
        self.current_link_displacement = [0,0,0,0,0]
        self.path = [[0,0,0,0,0]]
        ### 25 anchors
        self.anchors = anchors
        
        #self.goal = [0,0,0,0,0]
    
    
    def listener_callback(self,msg):
        if self.path:
            d = String()
            d.data = str(self.path[self.path_index])
            self.publisher_.publish(d)
            #---------------------------------------
            self.current_link_displacement = self.path[self.path_index]
            if self.path_index == (len(self.path) - 1):
                pass
            else:
                self.path_index += 1
    
    def listener_callback_location(self,msg):
        pose = string_to_float_multiarray(msg.data)
        start_vertex = self.current_link_displacement
        self.goal = estimate_goalPosition([pose[0],pose[1],pose[2]],self.reachable_pose_classify)
        print(self.goal)
        if len(self.goal) == 0:
            print("No way available or has arrived. Who know ???")
        else:
            temp = copy.deepcopy(self.goal[0])
            goal = temp[0]
            start_vertex = self.current_link_displacement
            #print(start_vertex)
            if goal == start_vertex:
                self.path =[start_vertex]
            else:
                #-----------------------
                min_distance = float('inf')
                index = 0
                for i,anchor in enumerate(self.anchors):
                    anchor_location, _ = calculate_end_effector_UR5(*anchor)
                    d = calculate_distance(temp[1],anchor_location)
                    if d < min_distance:
                        min_distance = d
                        index = i
                #------------------------

                #print(self.anchors[index])
                self.path = get_path_refined(self.node_list,self.pred_array[index], self.anchors[index], start_vertex)
                self.path.reverse()
                #print(self.path)
                path_from_anchors = get_path_refined(self.node_list,self.pred_array[index],self.anchors[index],goal)
                #print(path_from_anchors)
                index_to = len(self.path) - 1
                index_from = 0
                for i,checkpoint_1 in enumerate(self.path):
                    for k,checkpoint_2 in enumerate(path_from_anchors):
                        if checkpoint_2 == checkpoint_1 :
                            index_to = i
                            index_from = k 
                            break
                #print(index_to,index_from)
                self.path = self.path[:(index_to+1)] + path_from_anchors[(index_from+1):]
            print(self.path)
            self.path_index = 0

  

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


def node_classification(reachable_points,displacement):
    c = [[[[] for i in range(80)] for i in range(80)] for i in range(80)]

    for point, orientation in zip(reachable_points,displacement):
        i = int((point[0]+2)/0.05) 
        if i == 80 :
            i = 79
        j = int((point[1]+2)/0.05) 
        if j == 80 :
            j = 79
        k = int((point[2]+2)/0.05) 
        if k == 80 :
            k = 79 
        c[i][j][k].append(orientation)

    return c

def estimate_goalPose(goal,c):
    solutions = []
    solution = []
    i = int((goal[0][0]+2)/0.05) 
    if i == 80 :
        i = 79
    j = int((goal[0][1]+2)/0.05) 
    if j == 80 :
        j = 79
    k = int((goal[0][2]+2)/0.05) 
    if k == 80 :
        k = 79

    for potential_solution in c[i][j][k]:
        minE = 10
        E = calculate_mse(potential_solution[2],goal[1])
        if E >= 0.2 :
            continue
        elif E < minE:
            solution = potential_solution
            minE = E
    return solution

def estimate_goalPosition(goal,c):
    solutions = []
    solution = []
    i = int((goal[0]+2)/0.05) 
    if i == 80 :
        i = 79
    j = int((goal[1]+2)/0.05) 
    if j == 80 :
        j = 79
    k = int((goal[2]+2)/0.05) 
    if k == 80 :
        k = 79

    return c[i][j][k]

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


def string_to_float_multiarray(string):
    float_ = ''
    data = []
    for i in string:
        if i == ',' or i == ']':
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

def calculate_mse(pose,goalPose):
    MSE = 0
    for i,j in zip(pose,goalPose) :
        MSE += (i - j)**2
    MSE = MSE/len(pose)
    MSE = MSE**0.5
    return MSE



def main(args=None):

    node_list = []
    with open('/home/vb/control_robot_arm_with_node_graph/30_15_15_30_30/node.csv', mode ='r')as file:
        csvFile = csv.reader(file)
        for line in csvFile:
            for  i in line:
                node_list.append(string_to_float_multiarray(i))

    #--------------------------
    anchors = []
    with open('/home/vb/control_robot_arm_with_node_graph/30_15_15_30_30/anchors_30_15_15_30_30.csv', mode ='r')as file:
        csvFile = csv.reader(file)
        for line in csvFile:
            for  i in line:
                anchors.append(string_to_float_multiarray(i))
    #-------------------------
    pred_array = []
    with open('/home/vb/control_robot_arm_with_node_graph/30_15_15_30_30/pred_array_avoid_obstacles.csv', mode ='r')as file:
        csvFile = csv.reader(file)
        for lines in csvFile:
            data = []
            for i in lines : 
                if i == '':
                    data.append(None)
                else:
                    data.append(int(i))
            pred_array.append(data)
    print(len(pred_array))

    #-------------------------
    rclpy.init(args=args)

    testing_node= Testing_msg(node_list,pred_array,anchors)

    rclpy.spin(testing_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    testing_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()