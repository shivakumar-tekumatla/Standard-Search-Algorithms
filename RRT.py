# Standard Algorithm Implementation
# Sampling-based Algorithms RRT and RRT*

from dis import dis
from tabnanny import verbose
import matplotlib.pyplot as plt
import numpy as np
from sklearn import neighbors
from bresenham import bresenham

# Class for each tree node
class Node:
    def __init__(self, row, col):
        self.row = row        # coordinate
        self.col = col        # coordinate
        self.parent = None    # parent node
        self.cost = 0.0       # cost

# Class for RRT
class RRT:
    # Constructor
    def __init__(self, map_array, start, goal):
        self.map_array = map_array            # map array, 1->free, 0->obstacle
        # print(self.map_array)
        self.size_row = map_array.shape[0]    # map size
        self.size_col = map_array.shape[1]    # map size

        self.start = Node(start[0], start[1]) # start node
        self.goal = Node(goal[0], goal[1])    # goal node
        self.vertices = []                    # list of nodes
        self.found = False                    # found flag
        self.max_rows = len(self.map_array)   #bounds for rows 
        self.max_columns = len(self.map_array[0]) #bounds for columns 
        self.step_size = 0.025*(self.max_columns+self.max_rows)/2
        self.goal_bias = 0.1

    def init_map(self):
        '''Intialize the map before each search
        '''
        self.found = False
        self.vertices = []
        self.vertices.append(self.start)

    
    def dis(self, node1, node2):
        '''Calculate the euclidean distance between two nodes
        arguments:
            node1 - node 1
            node2 - node 2

        return:
            euclidean distance between two nodes
        '''
        
        ### YOUR CODE HERE ###
        # print("node1",type(node1))
        # print("node2",type(node2))
        return np.sqrt((node1.row-node2.row)**2+(node1.col-node2.col)**2)

    
    def check_collision(self, node1, node2):
        '''Check if the path between two nodes collide with obstacles
        arguments:
            node1 - node 1
            node2 - node 2

        return:
            True if the new node is valid to be connected
        '''
        ### YOUR CODE HERE ###
        """Using Bresenham algorithm to check the collision """
        y1,x1 = node1.row,node1.col #Because the row is same as y axis 
        y2,x2 = node2.row,node2.col
        
        bresenham_line = list(bresenham(x1,y1,x2,y2))
        # print(bresenham_line)
        for point in bresenham_line:
            x,y = point
            if self.map_array[y][x] ==0:
                return False
                break
            else:
                return True 

    def get_new_point(self, goal_bias):
        '''Choose the goal or generate a random point
        arguments:
            goal_bias - the possibility of choosing the goal instead of a random point

        return:
            point - the new point
        '''
        ### YOUR CODE HERE ###
        self.goal_bias = goal_bias
        
        if np.random.rand()>self.goal_bias:
            point = Node(np.random.randint(0,self.max_rows),np.random.randint(0,self.max_columns))
        else:
            point = self.goal
        return point

    
    def get_nearest_node(self, point):
        '''Find the nearest node in self.vertices with respect to the new point
        arguments:
            point - the new point

        return:
            the nearest node
        '''
        ### YOUR CODE HERE ###
        all_distances=[]
        # Finding distance to all the vertices 
        for vertex in self.vertices:
            all_distances.append(self.dis(point,vertex))
        # self.vertices[np.argmin(all_distances)]
        return self.vertices[np.argmin(all_distances)]


    def get_neighbors(self, new_node, neighbor_size):
        '''Get the neighbors that are within the neighbor distance from the node
        arguments:
            new_node - a new node
            neighbor_size - the neighbor distance

        return:
            neighbors - a list of neighbors that are within the neighbor distance 
        '''
        ### YOUR CODE HERE ###
        neighbors=[]
        for vertex in self.vertices:
            if self.dis(new_node,vertex) <=neighbor_size:
                neighbors.append(vertex)
        return neighbors


    def rewire(self, new_node, neighbors):
        '''Rewire the new node and all its neighbors
        arguments:
            new_node - the new node
            neighbors - a list of neighbors that are within the neighbor distance from the node

        Rewire the new node if connecting to a new neighbor node will give least cost.
        Rewire all the other neighbor nodes.
        '''
        ### YOUR CODE HERE ###
        distances =[]
        for neighbor in neighbors:
            distances.append(neighbor.cost+self.dis(new_node,neighbor))
        best_neighbor = neighbors[np.argmin(distances)] #Finding the best neighbor based on lowest distance
        new_node.parent = best_neighbor 
        new_node.cost = best_neighbor.cost+self.dis(new_node,best_neighbor)
        neighbors.remove(best_neighbor)
        #Trying to rewire all the neighbors again with the newly formed point 
        for neighbor in neighbors:
            self.vertices.remove(neighbor)
            distance_newNode = new_node.cost + self.dis(new_node,neighbor)
            if neighbor.cost > distance_newNode:
                neighbor.parent = new_node 
                neighbor.cost = distance_newNode
            self.vertices.append(neighbor)
        self.vertices.append(new_node)

    def draw_map(self):
        '''Visualization of the result
        '''
        # Create empty map
        fig, ax = plt.subplots(1)
        img = 255 * np.dstack((self.map_array, self.map_array, self.map_array))
        ax.imshow(img)

        # Draw Trees or Sample points
        for node in self.vertices[1:-1]:
            plt.plot(node.col, node.row, markersize=3, marker='o', color='y')
            plt.plot([node.col, node.parent.col], [node.row, node.parent.row], color='y')
        
        # Draw Final Path if found
        if self.found:
            cur = self.goal
            while cur.col != self.start.col and cur.row != self.start.row:
                plt.plot([cur.col, cur.parent.col], [cur.row, cur.parent.row], color='b')
                cur = cur.parent
                plt.plot(cur.col, cur.row, markersize=3, marker='o', color='b')

        # Draw start and goal
        plt.plot(self.start.col, self.start.row, markersize=5, marker='o', color='g')
        plt.plot(self.goal.col, self.goal.row, markersize=5, marker='o', color='r')

        # show image
        plt.show()


    def RRT(self, n_pts=1000):
        '''RRT main search function
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        In each step, extend a new node if possible, and check if reached the goal
        '''
        # Remove previous result
        self.init_map()
        # print(self.map_array)
        ### YOUR CODE HERE ###
        
        itr=0  #keeping track of iterations 
        while itr<=n_pts: # In each step,
            point = self.get_new_point(self.goal_bias) # get a new point, 
            nearest_node = self.get_nearest_node(point)# get its nearest node, 

            distance = self.dis(point,nearest_node)
            # print(distance)
            if distance ==0:continue 
            t = self.step_size/distance
            next_point = Node(int((1-t)*nearest_node.row+t*point.row),int((1-t)*nearest_node.col+t*point.col))
            # extend the node and check collision to decide whether to add or drop,
            not_collision = self.check_collision(next_point, nearest_node)
            if not_collision:
                itr+=1
                self.vertices.append(next_point)
                next_point.parent = nearest_node
                next_point.cost = next_point.parent.cost +self.dis(next_point.parent ,next_point)

                if self.dis(next_point,self.goal)<self.step_size and self.check_collision(next_point, self.goal):  # if added, check if reach the neighbor region of the goal
                    self.found= True
                    self.goal.parent = next_point
                    self.goal.cost = next_point.cost +self.dis(next_point ,self.goal)
                    break
            else:
                pass
            # print("Vertices",self.vertices)

        # Output
        if self.found:
            steps = len(self.vertices) - 2
            length = self.goal.cost
            print("It took %d nodes to find the current path" %steps)
            print("The path length is %.2f" %length)
        else:
            print("No path found")
        
        # Draw result
        self.draw_map()


    def RRT_star(self, n_pts=1000, neighbor_size=20):
        '''RRT* search function
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points
            neighbor_size - the neighbor distance
        
        In each step, extend a new node if possible, and rewire the node and its neighbors
        '''
        # Remove previous result
        self.init_map()

        ### YOUR CODE HERE ###
        itr = 0
        while itr<=n_pts: # In each step,
            point = self.get_new_point(self.goal_bias)# get a new point, 
            nearest_node = self.get_nearest_node(point) # get its nearest node, 
            distance = self.dis(point,nearest_node)
            t = self.step_size/distance
            next_point = Node(int((1-t)*nearest_node.row+t*point.row),int((1-t)*nearest_node.col+t*point.col))
            # extend the node and check collision to decide whether to add or drop,
            not_collision = self.check_collision(next_point, nearest_node)
            if not_collision:
                itr+=1
                neighbors = self.get_neighbors(next_point, neighbor_size)
                # print(neighbors)
                # if added, rewire the node and its neighbors,
                self.rewire(next_point,neighbors)
                # and check if reach the neighbor region of the goal if the path is not found.
                if self.dis(next_point,self.goal)<self.step_size and self.check_collision(next_point, self.goal):  # if added, check if reach the neighbor region of the goal
                    self.found= True
                    self.goal.parent = next_point
                    self.goal.cost = next_point.cost +self.dis(next_point ,self.goal)
                    break
            else:
                pass
        # Output
        if self.found:
            steps = len(self.vertices) - 2
            length = self.goal.cost
            print("It took %d nodes to find the current path" %steps)
            print("The path length is %.2f" %length)
        else:
            print("No path found")

        # Draw result
        self.draw_map()
