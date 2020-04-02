import matplotlib.pyplot as plt
import numpy as np
import math as math
from matplotlib.collections import LineCollection


class A_star (object):

    # initial the values
    def __init__(self,dims,start_point,goal):
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
        self.last_current_point = list([[1,1]])
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
        self.value = list([0])



        # robot path varales:
        self. block_numbers = list([0])
        self.theta= list([])
        self.heading = list([-math.pi/2])
        self.theta_collect =list([])
        self.x_collect = list([])
        self.y_collect = list([])
        self.current_x = list([])
        self.current_y = list([])
        self.delta_x = list([])
        self.delta_y = list([])
        self.goal_block_x = list([])
        self.goal_block_y = list([])


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


    def landmark_block_real(self):
        for i in range(len(self.landmark)):
        #for i in range(2):
            x = int(math.floor(self.landmark_x[i]*10))+20
            y = int(math.floor(self.landmark_y[i]*10))+60
            for i in range (x-3,x+4):
                for j in range (y-3,y+4):
                    self.matrix[j,i] = 3
            #self.matrix[y,x] = 3

        return self.matrix

    # mark goal and  start point on the matrix, inital current_point and parents to start_point position on the world
    def mark_start_goal_block_real(self):
        self.start_x = int(math.floor(self.start_point[0]*10))+20
        self.start_y = int(math.floor(self.start_point[1]*10))+60
        self.matrix[self.start_y,self.start_x] = 1
        self.start_point_matrix_position = [self.start_y,self.start_x]
        self.current_point = self.start_point_matrix_position
        #self.last_current_point = [1,1]
        #self.parent.append(self.start_point_matrix_position)
        self.closelist.append(self.current_point)


        self.goal_x = int(math.floor(self.goal[0]*10))+20
        self.goal_y = int(math.floor(self.goal[1]*10))+60
        self.matrix[self.goal_y,self.goal_x] = 2
        self.goal_point_matrix_position = [self.goal_y,self.goal_x]

        return self.matrix

    def heuristic(self):
        for i in range (self.dims[0]):
            for j in range(self.dims[1]):
                if self.matrix[i][j] == 3:
                    self.matrix_h [i][j] = 10000;
                else:
                    self.matrix_h[i][j] = (((i - self.goal_point_matrix_position[0])**2) +((j - self.goal_point_matrix_position[1])**2))**(0.5)
        return self.matrix_h

    # update whole world g value and open list g value:

    def neighbor_real(self):
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

        if self.current_point[0]-1>= 0 :
            if self.matrix_g[self.current_point[0]-1,self.current_point[1]] > matrix_neighbor_g[0,1] or self.matrix_g[self.current_point[0]-1,self.current_point[1]] == 0:
                self.matrix_g[self.current_point[0]-1,self.current_point[1]] = matrix_neighbor_g[0,1]
                self.openlist_g[self.current_point[0]-1,self.current_point[1]] = matrix_neighbor_g[0,1]
                self.parent[self.current_point[0]-1][self.current_point[1]][0] = self.current_point[0]
                self.parent[self.current_point[0]-1][self.current_point[1]][1] = self.current_point[1]

        if self.current_point[0]-1 >= 0 and self.current_point[1]+1 < 70:
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


        if self.current_point[1]+1<70:
            if self.matrix_g[self.current_point[0],self.current_point[1]+1] > matrix_neighbor_g[1,2] or self.matrix_g[self.current_point[0],self.current_point[1]+1] == 0:
                self.matrix_g[self.current_point[0],self.current_point[1]+1] = matrix_neighbor_g[1,2]
                self.openlist_g[self.current_point[0],self.current_point[1]+1] = matrix_neighbor_g[1,2]
                self.parent[self.current_point[0]][self.current_point[1]+1][0] = self.current_point[0]
                self.parent[self.current_point[0]][self.current_point[1]+1][1] = self.current_point[1]


        if self.current_point[0]+1 <120 and self.current_point[1]-1 >=0:
            if self.matrix_g[self.current_point[0]+1,self.current_point[1]-1] > matrix_neighbor_g[2,0] or self.matrix_g[self.current_point[0]+1,self.current_point[1]-1] == 0:
                self.matrix_g[self.current_point[0]+1,self.current_point[1]-1] = matrix_neighbor_g[2,0]
                self.openlist_g[self.current_point[0]+1,self.current_point[1]-1] = matrix_neighbor_g[2,0]
                self.parent[self.current_point[0]+1][self.current_point[1]-1][0] = self.current_point[0]
                self.parent[self.current_point[0]+1][self.current_point[1]-1][1] = self.current_point[1]

        if self.current_point[0]+1 < 120:
            if self.matrix_g[self.current_point[0]+1,self.current_point[1]] > matrix_neighbor_g[2,1] or self.matrix_g[self.current_point[0]+1,self.current_point[1]] == 0:
                self.matrix_g[self.current_point[0]+1,self.current_point[1]] = matrix_neighbor_g[2,1]
                self.openlist_g[self.current_point[0]+1,self.current_point[1]] = matrix_neighbor_g[2,1]
                self.parent[self.current_point[0]+1][self.current_point[1]][0] = self.current_point[0]
                self.parent[self.current_point[0]+1][self.current_point[1]][1] = self.current_point[1]

        if self.current_point[0]+1 <120 and self.current_point[1]+1 <70 :
            if self.matrix_g[self.current_point[0]+1,self.current_point[1]+1] > matrix_neighbor_g[2,2] or self.matrix_g[self.current_point[0]+1,self.current_point[1]+1] == 0:
                self.matrix_g[self.current_point[0]+1,self.current_point[1]+1] = matrix_neighbor_g[2,2]
                self.openlist_g[self.current_point[0]+1,self.current_point[1]+1] = matrix_neighbor_g[2,2]
                self.parent[self.current_point[0]+1][self.current_point[1]+1][0] = self.current_point[0]
                self.parent[self.current_point[0]+1][self.current_point[1]+1][1] = self.current_point[1]

        if self.value[0] > 2:
            a = self.value[0]
            self.openlist_g[self.last_current_point[a-2][0]][self.last_current_point[a-2][1]] = self.openlist_g[self.last_current_point[a-2][0]][self.last_current_point[a-2][1]] + 1

        self.openlist_g[self.current_point[0],self.current_point[1]] = 0
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
    def robot_path(self):
        self.current_y =  self.start_point[1]
        self.current_x = self.start_point[0]
        self.x_collect.append(self.current_x)
        self.y_collect.append(self.current_y)
        dx = 1*math.cos(self.heading[0])
        dy = 1*math.sin(self.heading[0])
        self.delta_x.append(dx)
        self.delta_y.append(dy)


        total_block = len(self.path)/2
        for i in range (total_block):
        #for i in range (1):
            bl_number = i*2
        #for i in range (1):
            #bl_number = i
            self.block_angle(bl_number)
            self.theta_list()
            self.linear_list()




    def block_angle(self,bl_number):

        end_block = self.path[bl_number]
        self.goal_block_x = float((end_block[1] - 20))/10+0.05
        self.goal_block_y = float((end_block[0] - 60))/10+0.05
        #theta = math.atan2(end_block[0]-start_block[0],end_block[1]-start_block[1])
        theta = math.atan2(self.goal_block_y-self.current_y,self.goal_block_x - self.current_x)
        if theta > - math.pi and theta < 0:
            theta = theta + 2* math.pi
        if self.heading[0] > -math.pi and self.heading[0] < 0:
            self.heading[0] = self.heading[0] + 2*math.pi
        self.theta = theta - self.heading[0]
        #print self.theta
        self.theta_collect.append(self.heading[0])



    def linear_list(self):
        global new_x
        global new_y
        x_distance = (self.current_x - self.goal_block_x)**2
        y_distance = (self.current_y - self.goal_block_y)**2
        distance = (x_distance + y_distance)**0.5
        time = (distance/0.288)**0.5
        time = round(time,1)
        time_range = int(time*10)



        # first part
        for i in range (time_range):

            i = (float(i)+1)/10
            new_x = self.current_x + 0.5*(0.288*math.cos(self.heading[0]))*(i**2)
            new_y = self.current_y + 0.5*(0.288*math.sin(self.heading[0]))*(i**2)

            self.theta_collect.append(self.heading[0])
            self.theta_collect.append(self.heading[0])
            dx = 1*math.cos(self.heading[0])
            dy = 1*math.sin(self.heading[0])
            self.x_collect.append(new_x)
            self.y_collect.append(new_y)
            self.delta_x.append(dx)
            self.delta_y.append(dy)

        self.current_x = new_x
        self.current_y = new_y

        # middle_part
        rest = distance - (0.5*(0.288)*(time**2))*2
        v_max = 0.288*time
        if v_max != 0 :
            t_max_speed = round(rest/v_max,1)
        else:
            t_max_speed = 0
        new_x = self.current_x + t_max_speed * v_max * math.cos(self.heading[0])
        new_y = self.current_y + t_max_speed * v_max * math.sin(self.heading[0])

        self.theta_collect.append(self.heading[0])
        self.theta_collect.append(self.heading[0])
        dx = 1*math.cos(self.heading[0])
        dy = 1*math.sin(self.heading[0])
        self.x_collect.append(new_x)
        self.y_collect.append(new_y)
        self.delta_x.append(dx)
        self.delta_y.append(dy)
        self.current_x = new_x
        self.current_y = new_y

        # last_part
        for i in range (time_range):
            i = (float(i)+1)/10
            s = v_max*i -  0.5*(0.288)*(i**2)
            new_x = self.current_x + s * math.cos(self.heading[0])
            new_y = self.current_y + s * math.sin(self.heading[0])

            self.theta_collect.append(self.heading[0])
            self.theta_collect.append(self.heading[0])
            dx = 1*math.cos(self.heading[0])
            dy = 1*math.sin(self.heading[0])
            self.x_collect.append(new_x)
            self.y_collect.append(new_y)
            self.delta_x.append(dx)
            self.delta_y.append(dy)

        self.current_x = new_x
        self.current_y = new_y






        return (self.current_x,self.current_y,self.heading)




    def theta_list(self):
        if self.theta < - math.pi:
            time = ((self.theta + 2*math.pi)/5.579) ** (0.5)
            #print self.theta +2*math.pi
            #print time
        elif self.theta  > math.pi:
            time = (abs(self.theta - 2*math.pi)/5.579) ** (0.5)
        else:
            time = (abs(self.theta)/5.579)**(0.5)
        #print time
        time = round(time,1)
        time_range = int(time*10)
        if self.theta > 0 and self.theta < math.pi:
            self.clockwise(time_range,time)
        elif self.theta < - math.pi :
            self.clockwise(time_range,time)
        else:
            self.counterclockwise(time_range,time)



    def clockwise(self,time_range,time):
        global my_heading1
        # first part
        for i in range (time_range):
            i = (float(i)+1)/10
            my_heading1 = self.heading[0] +  0.5*(5.579)*(i**2)
            if my_heading1 > 2* math.pi:
                my_heading1 = my_heading1 - 2*math.pi
            #append value
            self.theta_collect.append(my_heading1)
            self.theta_collect.append(my_heading1)
            dx = 1*math.cos(my_heading1)
            dy = 1*math.sin(my_heading1)
            self.x_collect.append(self.current_x)
            self.y_collect.append(self.current_y)
            self.delta_x.append(dx)
            self.delta_y.append(dy)
        self.heading[0] = my_heading1

            # middle part
        if self.theta < - math.pi:
            rest = self.theta + 2*math.pi - (0.5*(5.579)*(time**2))*2
        elif self.theta >  math.pi:
            rest = abs(self.theta - 2*math.pi) - (0.5*(5.579)*(time**2))*2
        else:
            rest = abs(self.theta) - (0.5*(5.579)*(time**2))*2

        v_max = 5.579*time
        if v_max != 0:
            t_max_speed = round(rest/v_max,1)
        else:
            t_max_speed = 0
        my_heading1 = my_heading1 + t_max_speed * v_max

        if my_heading1 > 2* math.pi:
            my_heading1 = my_heading1 - 2*math.pi

        self.theta_collect.append(my_heading1)
        self.x_collect.append(self.current_x)
        self.y_collect.append(self.current_y)
        dx = 1*math.cos(my_heading1)
        dy = 1*math.sin(my_heading1)
        self.delta_x.append(dx)
        self.delta_y.append(dy)

        self.heading[0] = my_heading1


            # last_half
        for i in range (time_range):
            i = (float(i)+1)/10
            s = v_max*i -  0.5*(5.579)*(i**2)
            my_heading1 = self.heading[0] + s
            if my_heading1 > 2* math.pi:
                my_heading1 = my_heading1 - 2*math.pi
            self.theta_collect.append(my_heading1)
            dx = 1*math.cos(my_heading1)
            dy = 1*math.sin(my_heading1)
            self.x_collect.append(self.current_x)
            self.y_collect.append(self.current_y)
            self.delta_x.append(dx)
            self.delta_y.append(dy)
        self.heading[0] = my_heading1


    def counterclockwise(self,time_range,time):
        global my_heading
        # first part
        for i in range (time_range):
            i = (float(i)+1)/10

            my_heading = self.heading[0] -  0.5*(5.579)*(i**2)
            if my_heading < 0:
                my_heading = my_heading + 2*math.pi
            #append value
            self.theta_collect.append(my_heading)
            self.theta_collect.append(my_heading)
            dx = 1*math.cos(my_heading)
            dy = 1*math.sin(my_heading)
            self.x_collect.append(self.current_x)
            self.y_collect.append(self.current_y)
            self.delta_x.append(dx)
            self.delta_y.append(dy)
        global my_heading
        self.heading[0] = my_heading

            # middle part
        if self.theta < - math.pi:
            rest = self.theta + 2*math.pi - (0.5*(5.579)*(time**2))*2
        elif self.theta >  math.pi:
            rest = abs(self.theta - 2*math.pi) - (0.5*(5.579)*(time**2))*2
        else:
            rest = abs(self.theta) - (0.5*(5.579)*(time**2))*2


        v_max = 5.579*time
        if v_max != 0:
            t_max_speed = round(rest/v_max,1)
        else:
            t_max_speed = 0
        my_heading = my_heading - t_max_speed * v_max

        if my_heading < 0:
            my_heading = my_heading + 2*math.pi

        self.theta_collect.append(my_heading)
        self.x_collect.append(self.current_x)
        self.y_collect.append(self.current_y)
        dx = 1*math.cos(my_heading)
        dy = 1*math.sin(my_heading)
        self.delta_x.append(dx)
        self.delta_y.append(dy)

        self.heading[0] = my_heading


            # last_half
        for i in range (time_range):
            i = (float(i)+1)/10
            s = v_max*i -  0.5*(5.579)*(i**2)
            my_heading = self.heading[0] - s
            if my_heading < 0:
                my_heading = my_heading + 2*math.pi
            self.theta_collect.append(my_heading)
            dx = 1*math.cos(my_heading)
            dy = 1*math.sin(my_heading)
            self.x_collect.append(self.current_x)
            self.y_collect.append(self.current_y)
            self.delta_x.append(dx)
            self.delta_y.append(dy)
        self.heading[0] = my_heading





    def main(self):
        self.parent_matrix()
        landmark = self.landmarkpoint()

        self.landmark_block_real()
        world = self.mark_start_goal_block_real()
        self.heuristic()
        while (1):
        #for i in range (4):
            self.value[0] = self.value[0]+1
            self.neighbor_real()

            self.f_value()
            self.find_first_nonzero_value()

            currentpoint = self.update_current_point_and_closelist()
            self.last_current_point.append(currentpoint)

            pathway = self.pathway()
            for i in range (len(pathway)):
                world[pathway[i][0],pathway[i][1]] = 4
            if currentpoint[0] == self.goal_y and currentpoint[1] == self.goal_x:
                break

            else:
                self.start_x = currentpoint[1]
                self.start_y = currentpoint[0]
                self.start_point_matrix_position = [self.start_y,self.start_x]
                self.openlist_g [self.start_y,self.start_x]
                self.current_point = self.start_point_matrix_position
                self.closelist = list([])
                self.parent = list([])
                self.parent_matrix()
                self.openlist_g = np.zeros(self.dims)
                self.openlist_f = np.zeros(self.dims)
                self.matrix_g = np.zeros(self.dims)
                self.neighbor_g = np.zeros((3,3))
                self.matrix_f = np.zeros(self.dims)
                self.first_value = list([])
                self.first_positive_value_position = list([0,0])
                #self.path=list([])
        #self.block_angle()
        #self.theta_list()
        #self.linear_list()
        self.robot_path()


        fig = plt.figure()
        ax = fig.add_subplot(111)

        for i in range (len(self.x_collect)):
            ax.arrow(self.x_collect[i],self.y_collect[i],self.delta_x[i]/2,self.delta_y[i]/2, width=0.0001,edgecolor = 'red' ,head_width=0.05,facecolor="red", overhang = 0)

    #ax.add_collection(lc)

        ax.matshow(world,extent = [-2,5,-6,6],origin="lower")
        ax.scatter(landmark[0],landmark[1], np.sqrt(100),color='black')
        #ax.scatter(self.start_point[0],self.start_point[1],np.sqrt(10), color = 'pink')
        #ax.scatter(self.start_point[0],self.start_point[1], color = 'yellow')
        ax.scatter(self.start_point[0],self.start_point[1],color = 'red', marker = '*')
        ax.scatter(self.goal[0],self.goal[1],color = 'red', marker = '^')
        # ax.text(self.start_point[0]-1.5,self.start_point[1]-0.5,'Start',color = 'yellow',fontstyle="oblique")
        # ax.text(self.goal[0]-1.5,self.goal[1],'Goal',color = 'yellow',fontstyle="oblique")
        ax.grid(which='major', axis='both', linestyle='-', color='w', linewidth=1)
        ax.set_xticks(np.arange(-2, 6, 1));
        ax.set_yticks(np.arange(-6, 7, 1));

if __name__ == '__main__':
    for i in range (3):
        #point for question 7
        S1 = [[2.45,-3.55],[4.95,-0.05],[-0.55,1.45]]
        G1 = [[0.95, -1.55],[2.45,0.25],[1.95,3.95]]

        astar = A_star((120,70),S1[i%3],G1[i%3])
        astar.main()
    plt.show()
    #except rospy.ROSInterruptException:
