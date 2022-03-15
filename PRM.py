# Standard Algorithm Implementation
# Sampling-based Algorithms PRM

import matplotlib.pyplot as plt
import numpy as np
import networkx as nx
from bresenham import bresenham 
from scipy.spatial import KDTree

# Class for PRM
class PRM:
    # Constructor
    def __init__(self, map_array):
        self.map_array = map_array            # map array, 1->free, 0->obstacle
        self.size_row = map_array.shape[0]    # map size
        self.size_col = map_array.shape[1]    # map size

        self.samples = []                     # list of sampled points
        self.graph = nx.Graph()               # constructed graph
        self.path = []                        # list of nodes of the found path
        self.k_neighbors = 8                 # Number of neighbors for connecting to start and goal positions  
    def check_collision(self, p1, p2):
        '''Check if the path between two points collide with obstacles
        arguments:
            p1 - point 1, [row, col]
            p2 - point 2, [row, col]

        return:
            True if there are obstacles between two points
        '''
        ### YOUR CODE HERE ###
        """Using Bresenham algorithm to check the collision """
        
        y1,x1 = p1[0],p1[1] #Because the row is same as y axis 
        y2,x2 = p2[0],p2[1] 

        bresenham_line = list(bresenham(x1,y1,x2,y2))
        for point in bresenham_line:
            x,y = point
            if self.map_array[y][x] ==0:
                return True
        return False 

    def dis(self, point1, point2):
        '''Calculate the euclidean distance between two points
        arguments:
            p1 - point 1, [row, col]
            p2 - point 2, [row, col]

        return:
            euclidean distance between two points
        '''
        ### YOUR CODE HERE ###
        return np.sqrt((point1[0]-point2[0])**2+(point1[1]-point2[1])**2)

    def uniform_sample(self, n_pts):
        '''Use uniform sampling and store valid points
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        check collision and append valide points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        '''
        # Initialize graph
        self.graph.clear()

        ### YOUR CODE HERE ###
        # self.samples.append((0, 0))
        n_sqrt = np.ceil(np.sqrt(n_pts))
        row_step = np.ceil(self.size_row/n_sqrt)
        col_step = np.ceil(self.size_col/n_sqrt)
        rows_grid = np.arange(0,self.size_row,row_step)
        cols_grid = np.arange(0,self.size_col,col_step)
        for row in rows_grid:
            for col in cols_grid:
                if self.map_array[int(row)][int(col)]==1:
                    self.samples.append((int(row), int(col)))

    def random_sample(self, n_pts):
        '''Use random sampling and store valid points
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        check collision and append valide points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        '''
        # Initialize graph
        self.graph.clear()

        ### YOUR CODE HERE ###
        # self.samples.append((0, 0))
        rows_list = np.random.uniform(0,self.size_row,n_pts)
        cols_list = np.random.uniform(0,self.size_col,n_pts)
        rows = map(np.floor,rows_list)
        cols = map(np.floor,cols_list)
        for val in zip(rows,cols):
            row,col = val 
            if self.map_array[int(row)][int(col)]==1:
                self.samples.append((int(row), int(col)))

    def gaussian_sample(self, n_pts):
        '''Use gaussian sampling and store valid points
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        check collision and append valide points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        '''
        # Initialize graph
        self.graph.clear()

        ### YOUR CODE HERE ###
        # self.samples.append((0, 0))
        mean = 1
        std  = 10
        #generating the first random point
        itr=0
        while itr<n_pts:
            
            p1_row = np.random.randint(0,self.size_row)
            p1_col = np.random.randint(0,self.size_col)
            p2_row = int(np.ceil(p1_row)+np.random.normal(loc=mean,scale=std))
            p2_col = int(np.ceil(p1_col)+np.random.normal(loc=mean,scale=std))
            
            if p2_col>=self.size_col or p2_row>=self.size_row:continue

            p1_not_collision = bool(self.map_array[p1_row][p1_col])
            p2_not_collision = bool(self.map_array[p2_row][p2_col])
            add_or_not = p1_not_collision^p2_not_collision #XOR operation 
            # print(p1_row,p1_col,p1_not_collision, p2_row,p2_col,p2_not_collision)
            if add_or_not:
                
                if p1_not_collision:
                    self.samples.append((p1_row,p1_col))
                else:
                    self.samples.append((p2_row,p2_col))
            itr+=1

    def bridge_sample(self, n_pts):
        '''Use bridge sampling and store valid points
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        check collision and append valide points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        '''
        # Initialize graph
        self.graph.clear()

        ### YOUR CODE HERE ###
        # self.samples.append((0, 0))
        mean =1 
        std = 20
        itr=0
        # n_pts =5000
        while itr<n_pts:
            p1_row = np.random.randint(0,self.size_row)
            p1_col = np.random.randint(0,self.size_col)
            p1_not_collision = bool(self.map_array[p1_row][p1_col])
            if not p1_not_collision:

                p2_row = int(np.ceil(p1_row)+np.random.normal(loc=mean,scale=std))
                p2_col = int(np.ceil(p1_col)+np.random.normal(loc=mean,scale=std))
            
                if p2_col>=self.size_col or p2_row>=self.size_row:continue
                p2_not_collision = bool(self.map_array[p2_row][p2_col])
                if not p2_not_collision:
                    p_row = (p1_row+p2_row)//2
                    p_col = (p1_col+p2_col)//2
                    # print(p_row,p_col)
                    if bool(self.map_array[p_row][p_col]):
                        self.samples.append((p_row,p_col))
            itr+=1
        # print(self.samples)
    def draw_map(self):
        '''Visualization of the result
        '''
        # Create empty map
        fig, ax = plt.subplots()
        img = 255 * np.dstack((self.map_array, self.map_array, self.map_array))
        ax.imshow(img)

        # Draw graph
        # get node position (swap coordinates)
        node_pos = np.array(self.samples)[:, [1, 0]]
        pos = dict( zip( range( len(self.samples) ), node_pos) )
        pos['start'] = (self.samples[-2][1], self.samples[-2][0])
        pos['goal'] = (self.samples[-1][1], self.samples[-1][0])
        
        # draw constructed graph
        nx.draw(self.graph, pos, node_size=3, node_color='y', edge_color='y' ,ax=ax)

        # If found a path
        if self.path:
            # add temporary start and goal edge to the path
            final_path_edge = list(zip(self.path[:-1], self.path[1:]))
            nx.draw_networkx_nodes(self.graph, pos=pos, nodelist=self.path, node_size=8, node_color='b')
            nx.draw_networkx_edges(self.graph, pos=pos, edgelist=final_path_edge, width=2, edge_color='b')

        # draw start and goal
        nx.draw_networkx_nodes(self.graph, pos=pos, nodelist=['start'], node_size=12,  node_color='g')
        nx.draw_networkx_nodes(self.graph, pos=pos, nodelist=['goal'], node_size=12,  node_color='r')

        # show image
        plt.axis('on')
        ax.tick_params(left=True, bottom=True, labelleft=True, labelbottom=True)
        plt.show()


    def sample(self, n_pts=1000, sampling_method="uniform"):
        '''Construct a graph for PRM
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points
            sampling_method - name of the chosen sampling method

        Sample points, connect, and add nodes and edges to self.graph
        '''
        # Initialize before sampling
        self.samples = []
        self.graph.clear()
        self.path = []

        # Sample methods
        if sampling_method == "uniform":
            self.uniform_sample(n_pts)
        elif sampling_method == "random":
            self.random_sample(n_pts)
        elif sampling_method == "gaussian":
            self.gaussian_sample(n_pts)
        elif sampling_method == "bridge":
            self.bridge_sample(n_pts)
            self.k_neighbors =20
            

        ### YOUR CODE HERE ###
        self.kd_tree = KDTree(self.samples)# Find the pairs of points that need to be connected
        pairs = []
        # #          (p_id1, p_id2, weight_12) ...]

        # Use sampled points and pairs of points to build a graph.
        # To add nodes to the graph, use
        # self.graph.add_nodes_from([p_id0, p_id1, p_id2 ...])
        # To add weighted edges to the graph, use
        # self.graph.add_weighted_edges_from([(p_id0, p_id1, weight_01), 
        #                                     (p_id0, p_id2, weight_02), 
        #                                     (p_id1, p_id2, weight_12) ...])
        # 'p_id' here is an integer, representing the order of 
        # current point in self.samples
        # For example, for self.samples = [(1, 2), (3, 4), (5, 6)],
        # p_id for (1, 2) is 0 and p_id for (3, 4) is 1.
        self.kdtree = KDTree(self.samples)
        distances,indices  = self.kdtree.query(self.samples,self.k_neighbors)
        for index_list in indices:
            sample_id = index_list[0]
            for i in range(1,len(index_list)):
                if not self.check_collision(self.samples[sample_id],self.samples[index_list[i]]):
                    pairs.append((sample_id,index_list[i],distances[sample_id][i]))
        nodes_indices = [i for i in range(len(self.samples))]
        self.graph.add_nodes_from(nodes_indices)
        self.graph.add_weighted_edges_from(pairs)

        # Print constructed graph information
        n_nodes = self.graph.number_of_nodes()
        n_edges = self.graph.number_of_edges()
        print("The constructed graph has %d nodes and %d edges" %(n_nodes, n_edges))


    def search(self, start, goal):
        '''Search for a path in graph given start and goal location
        arguments:
            start - start point coordinate [row, col]
            goal - goal point coordinate [row, col]

        Temporary add start and goal node, edges of them and their nearest neighbors
        to graph for self.graph to search for a path.
        '''
        # Clear previous path
        self.path = []

        # Temporarily add start and goal to the graph
        self.samples.append(start)
        start_id = 'start'#len(self.samples)-1
        goal_id  ='goal'# len(self.samples)-1
        self.samples.append(goal)
        # start and goal id will be 'start' and 'goal' instead of some integer
        self.graph.add_nodes_from(['start', 'goal'])

        ### YOUR CODE HERE ###
        start_distances,start_indices=self.kd_tree.query(start,self.k_neighbors)
        goal_distances,goal_indices=self.kd_tree.query(goal,self.k_neighbors)
        # Find the pairs of points that need to be connected
        # and compute their distance/weight.
        # You could store them as
        # start_pairs = [(start_id, p_id0, weight_s0), (start_id, p_id1, weight_s1), 
        #                (start_id, p_id2, weight_s2) ...]
        start_pairs = []
        goal_pairs = []
        for i in range(0,len(start_indices)):
            if not self.check_collision(start,self.samples[start_indices[i]]):
                start_pairs.append((start_id,start_indices[i],start_distances[i]))
        for i in range(0,len(goal_indices)):
            if not self.check_collision(goal,self.samples[goal_indices[i]]):
                goal_pairs.append((goal_id,goal_indices[i],goal_distances[i]))
        # Add the edge to graph
        self.graph.add_weighted_edges_from(start_pairs)
        self.graph.add_weighted_edges_from(goal_pairs)
        # Seach using Dijkstra
        try:
            self.path = nx.algorithms.shortest_paths.weighted.dijkstra_path(self.graph,'start', 'goal')
            path_length = nx.algorithms.shortest_paths.weighted.dijkstra_path_length(self.graph,'start', 'goal')
            print("The path length is %.2f" %path_length)
        except nx.exception.NetworkXNoPath:
            print("No path found")
        
        # Draw result
        self.draw_map()

        # Remove start and goal node and their edges
        self.samples.pop(-1)
        self.samples.pop(-1)
        self.graph.remove_nodes_from(['start', 'goal'])
        self.graph.remove_edges_from(start_pairs)
        self.graph.remove_edges_from(goal_pairs)
        