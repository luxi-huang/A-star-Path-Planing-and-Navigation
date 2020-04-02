import matplotlib.pyplot as plt
import numpy as np
import math as math
from matplotlib.collections import LineCollection


class A_star (object):

    # initial the values
    def __init__(self,dims,start_point,goal,value):
        self.landmark = np.loadtxt( 'ds1_Landmark_Groundtruth.dat' )
        self.landmark_x = list([])
        self.landmark_y = list([])
        self.matrix = np.zeros(dims)
        self.dims = dims
        self.start_point = start_point
        self.start_point_matrix_position = list([])

        self.goal = goal
        self.goal_point_matrix_position = list([])
        self.closelist = list([])

        self.parent = list([])
        self.openlist_g = np.zeros(dims)
        self.openlist_f = np.zeros(dims)
        self.current_point = list([])

        self.matrix_g = np.zeros(dims)
        self.matrix_h = np.zeros(dims)
        self.neighbor_g = np.zeros((3,3))
        self.matrix_f = np.zeros(dims)
        self.first_value = list([])
        self.first_positive_value_position = list([0,0])
        self.path=list([])
        self.start_x = list([])
        self.start_y = list([])
        self.goal_x = list([])
        self.goal_y = list([])
        self. line_collection = list([])
        self.value = value

        #self.neighbor_h = np.zeros((3,3))
    def parent_matrix(self):
        for i in range (self.dims[0]):
        #for i in range (2):
            self.parent.append([])
            for j in range (self.dims[1]):
                    self.parent[i].append([0,0])
        return self.parent

    # find the landmark
    def landmarkpoint(self):
        for i in range (len(self.landmark)):
            self.landmark_x. append(self.landmark[i,1])
            self.landmark_y. append(self.landmark[i,2])
        return (self.landmark_x,self.landmark_y)

    # mark landmark on the world
    def landmark_block(self):
        for i in range(len(self.landmark)):
        #for i in range(2):
            x = int(math.floor(self.landmark_x[i]))+2
            y = int(math.floor(self.landmark_y[i]))+6
            self.matrix[y,x] = 3
        return self.matrix

    # mark goal and  start point on the matrix, inital current_point and parents to start_point position on the world
    def mark_start_goal_block(self):
        self.start_x = int(math.floor(self.start_point[0]))+2
        self.start_y = int(math.floor(self.start_point[1]))+6
        self.matrix[self.start_y,self.start_x] = 1
        self.start_point_matrix_position = [self.start_y,self.start_x]
        self.current_point = self.start_point_matrix_position
        #self.parent.append(self.start_point_matrix_position)
        self.closelist.append(self.current_point)


        self.goal_x = int(math.floor(self.goal[0]))+2
        self.goal_y = int(math.floor(self.goal[1]))+6
        self.matrix[self.goal_y,self.goal_x] = 2
        self.goal_point_matrix_position = [self.goal_y,self.goal_x]

        return self.matrix

    def heuristic(self):
        for i in range (self.dims[0]):
            for j in range(self.dims[1]):
                if self.matrix[i][j] == 3:
                    self.matrix_h [i][j] = 1000;
                else:
                    self.matrix_h[i][j] = (((i - self.goal_point_matrix_position[0])**2) +((j - self.goal_point_matrix_position[1])**2))**(0.5)
        return self.matrix_h

    # update whole world g value and open list g value:
    def neighbor(self):
        matrix_neighbor_g = np.zeros((3,3))
        for i in range (3):
            for j in range(3):
                matrix_neighbor_g [i,j]= self.matrix_g [self.current_point[0],self.current_point[1]] + 1

        if self.current_point[0]-1 >= 0 and  self.current_point[1]-1 >= 0:
            if self.matrix_g[self.current_point[0]-1,self.current_point[1]-1] > matrix_neighbor_g[0,0] or self.matrix_g[self.current_point[0]-1,self.current_point[1]-1]== 0:
                self.matrix_g[self.current_point[0]-1,self.current_point[1]-1] = matrix_neighbor_g[0,0]
                self.openlist_g[self.current_point[0]-1,self.current_point[1]-1] = matrix_neighbor_g[0,0]
                self.parent[self.current_point[0]-1][self.current_point[1]-1][0] = self.current_point[0]
                self.parent[self.current_point[0]-1][self.current_point[1]-1][1] = self.current_point[1]

        if self.current_point[0]-1 >= 0 :
            if self.matrix_g[self.current_point[0]-1,self.current_point[1]] > matrix_neighbor_g[0,1] or self.matrix_g[self.current_point[0]-1,self.current_point[1]] == 0:
                self.matrix_g[self.current_point[0]-1,self.current_point[1]] = matrix_neighbor_g[0,1]
                self.openlist_g[self.current_point[0]-1,self.current_point[1]] = matrix_neighbor_g[0,1]
                self.parent[self.current_point[0]-1][self.current_point[1]][0] = self.current_point[0]
                self.parent[self.current_point[0]-1][self.current_point[1]][1] = self.current_point[1]

        if self.current_point[0]-1 >= 0 and self.current_point[1]+1 < 7:
            if self.matrix_g[self.current_point[0]-1,self.current_point[1]+1] > matrix_neighbor_g[0,2] or self.matrix_g[self.current_point[0]-1,self.current_point[1]+1] == 0:
                self.matrix_g[self.current_point[0]-1,self.current_point[1]+1] = matrix_neighbor_g[0,2]
                self.openlist_g[self.current_point[0]-1,self.current_point[1]+1] = matrix_neighbor_g[0,2]
                self.parent[self.current_point[0]-1][self.current_point[1]+1][0] = self.current_point[0]
                self.parent[self.current_point[0]-1][self.current_point[1]+1][1] = self.current_point[1]


        if self.current_point[1]-1 >= 0:
            if self.matrix_g[self.current_point[0],self.current_point[1]-1] > matrix_neighbor_g[1,0] or self.matrix_g[self.current_point[0],self.current_point[1]-1] == 0:
                self.matrix_g[self.current_point[0],self.current_point[1]-1] = matrix_neighbor_g[1,0]
                self.openlist_g[self.current_point[0],self.current_point[1]-1] = matrix_neighbor_g[1,0]
                self.parent[self.current_point[0]][self.current_point[1]-1][0] = self.current_point[0]
                self.parent[self.current_point[0]][self.current_point[1]-1][1] = self.current_point[1]


        if self.current_point[1]+1<7:
            if self.matrix_g[self.current_point[0],self.current_point[1]+1] > matrix_neighbor_g[1,2] or self.matrix_g[self.current_point[0],self.current_point[1]+1] == 0:
                self.matrix_g[self.current_point[0],self.current_point[1]+1] = matrix_neighbor_g[1,2]
                self.openlist_g[self.current_point[0],self.current_point[1]+1] = matrix_neighbor_g[1,2]
                self.parent[self.current_point[0]][self.current_point[1]+1][0] = self.current_point[0]
                self.parent[self.current_point[0]][self.current_point[1]+1][1] = self.current_point[1]


        if self.current_point[0]+1 <12 and self.current_point[1]-1 >=0:
            if self.matrix_g[self.current_point[0]+1,self.current_point[1]-1] > matrix_neighbor_g[2,0] or self.matrix_g[self.current_point[0]+1,self.current_point[1]-1] == 0:
                self.matrix_g[self.current_point[0]+1,self.current_point[1]-1] = matrix_neighbor_g[2,0]
                self.openlist_g[self.current_point[0]+1,self.current_point[1]-1] = matrix_neighbor_g[2,0]
                self.parent[self.current_point[0]+1][self.current_point[1]-1][0] = self.current_point[0]
                self.parent[self.current_point[0]+1][self.current_point[1]-1][1] = self.current_point[1]

        if self.current_point[0]+1 < 12:
            if self.matrix_g[self.current_point[0]+1,self.current_point[1]] > matrix_neighbor_g[2,1] or self.matrix_g[self.current_point[0]+1,self.current_point[1]] == 0:
                self.matrix_g[self.current_point[0]+1,self.current_point[1]] = matrix_neighbor_g[2,1]
                self.openlist_g[self.current_point[0]+1,self.current_point[1]] = matrix_neighbor_g[2,1]
                self.parent[self.current_point[0]+1][self.current_point[1]][0] = self.current_point[0]
                self.parent[self.current_point[0]+1][self.current_point[1]][1] = self.current_point[1]

        if self.current_point[0]+1 <12 and self.current_point[1]+1 <7 :
            if self.matrix_g[self.current_point[0]+1,self.current_point[1]+1] > matrix_neighbor_g[2,2] or self.matrix_g[self.current_point[0]+1,self.current_point[1]+1] == 0:
                self.matrix_g[self.current_point[0]+1,self.current_point[1]+1] = matrix_neighbor_g[2,2]
                self.openlist_g[self.current_point[0]+1,self.current_point[1]+1] = matrix_neighbor_g[2,2]
                self.parent[self.current_point[0]+1][self.current_point[1]+1][0] = self.current_point[0]
                self.parent[self.current_point[0]+1][self.current_point[1]+1][1] = self.current_point[1]

        self.openlist_g[self.current_point[0],self.current_point[1]] = 0
        #print self.openlist_g
        #print (self.current_point)
        self.openlist_g[self.start_y,self.start_x] = 0
        return (self.matrix_g,self.openlist_g)

    def f_value(self):
        for i in range (self.dims[0]):
            for j in range(self.dims[1]):
                if self. matrix_g[i][j] != 0:
                    self.matrix_f[i][j]=self.matrix_g[i][j]+self.matrix_h[i][j]

                if self. openlist_g[i][j] != 0:
                    self.openlist_f[i][j]=self.openlist_g[i][j]+self.matrix_h[i][j]

        self.openlist_f[self.current_point[0],self.current_point[1]] = 0
        return self.matrix_f


    def find_first_nonzero_value(self):
        for i in range (self.dims[0]):
            for j in range(self.dims[1]):
                if self.openlist_f[i,j] != 0:
                    self.first_value = self.openlist_f[i,j]
                    self.first_positive_value_position[0] = i
                    self.first_positive_value_position[1] = j
                    return self.first_value

    def update_current_point_and_closelist(self):
        self.first_positive_value_position[1]
        min_value = self.first_value
        for i in range (self.dims[0]):
            for j in range(self.dims[1]):
                if self.openlist_f[i,j] != 0 and self.openlist_f[i,j] < min_value:
                    min_value = self.openlist_f[i,j]
                    self.first_positive_value_position[0] = i
                    self.first_positive_value_position[1] = j


        for i in range (self.dims[0]):
            for j in range(self.dims[1]):
                if self.openlist_f[i,j] == min_value:
                    if self.openlist_g[i,j] < self.openlist_g[self.first_positive_value_position[0],self.first_positive_value_position[1]]:
                        self.first_positive_value_position[0] = i
                        self.first_positive_value_position[1] = j

        #self.parent.append(self.current_point)
        self.current_point[0] = self.first_positive_value_position[0]
        self.current_point[1] = self.first_positive_value_position[1]
        self.closelist.append(self.current_point)
        return self.current_point

    def pathway(self):
        new_parent = list([])
        self.path.append(self.current_point)
        new_parent = self.parent[self.current_point[0]][self.current_point[1]]
        two_point = [self.current_point,new_parent]
        self.line_collection.append(two_point)
        while (1):
            self.path.append(new_parent)
            if new_parent[0] == self.start_y and new_parent[1] == self.start_x:
                break
            else:
                old_parent = new_parent
                new_parent = self.parent[new_parent[0]][new_parent[1]]
                two_point = [old_parent,new_parent]
                #self.line_collection.append(two_point)

        return (self.path)

    def main(self):
        self.parent_matrix()
        landmark = self.landmarkpoint()

        self.landmark_block()
        world = self.mark_start_goal_block()

        self.heuristic()

        #if self.value > 2:
        #    self.openlist_g = np.zeros(self.dims)
        #    self.openlist_f = np.zeros(self.dims)

        while (1):


            self.neighbor()

            self.f_value()
            self.find_first_nonzero_value()
            #if self.value > 5:
            #     currentpoint = self.update_current_point_and_closelist_real()

            currentpoint = self.update_current_point_and_closelist()

            if currentpoint[0] == self.goal_y and currentpoint[1] == self.goal_x:
                break

        pathway = self.pathway()

        for i in range (len(pathway)):
            world[pathway[i][0],pathway[i][1]] = 4



        fig = plt.figure()
        ax = fig.add_subplot(111)
    #ax.add_collection(lc)
        ax.matshow(world,extent = [-2,5,-6,6],origin="lower")
        ax.scatter(landmark[0],landmark[1], np.sqrt(100),color='black')
        ax.grid(which='major', axis='both', linestyle='-', color='w', linewidth=1)
        ax.set_xticks(np.arange(-2, 6, 1));
        ax.set_yticks(np.arange(-6, 7, 1));
        # ax.arrow (0,0,1,1)



if __name__ == '__main__':
    for i in range (3):

        # point for question 1-6
        S = [[0.5,-1.5],[4.5,3.5],[-0.5,5.5]]
        G = [[0.5, 1.5],[4.5,-1.5],[1.5,-3.5]]

        astar = A_star((12,7),S[i%3],G[i%3],i)
        astar.main()

    plt.show()
    #except rospy.ROSInterruptException:
