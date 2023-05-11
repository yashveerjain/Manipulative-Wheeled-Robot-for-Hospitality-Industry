import pygame
import numpy as np
import os
import sys
import math
import random
import scipy
from collections import defaultdict 
import cv2
import time
# class rrt_map():
#     def __init__(self, start, goal, mapDimension, obsdim, obsnum) -> None:
#         self.start = start
#         self.goal = goal
#         self.MapDimensions = mapDimension
#         self.MapH, self.MapW = self.MapDimensions

#         self.MapWindowName = "RRT path planning"
#         pygame.display.set_caption(self.MapWindowName)
#         self.map = pygame.display.set_mode((self.MapW, self.MapH))
#         self.map.fill((255,255,255))
#         self.nodeRad=4
#         self.nodeThickness = 0
#         self.edgeThickness = 1

#         self.obstacles = []
#         self.obsdim = obsdim
#         self.obsNumber = obsnum

#         # color
#         self.grey = (70,70,70)
#         self.Blue = (0, 0, 255)
#         self.Green = (0, 255, 0)
#         self.Red = (255, 0, 0)
#         self.white = (255,255,255)

#     def drawMap(self):
#         pygame.draw.circle(self.map, self.Green, self.start, self.nodeRad, 0)
#         pygame.draw.circle(self.map, self.Red, self.goal,self.nodeRad+10,1)
#         self.drawObs()

#     def drawObs(self):
#         self.makeObs()
#         obstacleList = self.obstacles.copy()
#         while len(obstacleList)>0:
#             obs = obstacleList.pop(0)
#             pygame.draw.rect(self.map, self.grey, obs)


#     def makeRandomRect(self):
#         uppercornerx = int(random.uniform(0,self.MapW))
#         uppercornery = int(random.uniform(0,self.MapH))

#         return (uppercornerx, uppercornery)

#     def makeObs(self):
#         obs = []

#         for i in range(0, self.obsNumber):
#             rectang=None
#             startGoalcol = True

#             while startGoalcol:
#                 upper = self.makeRandomRect()
#                 rectang = pygame.Rect(upper,(self.obsdim,self.obsdim))
#                 if rectang.collidepoint(self.start) or rectang.collidepoint(self.goal):
#                     startGoalcol = True
#                 else:
#                     startGoalcol=False
#             obs.append(rectang)

#         self.obstacles = obs.copy()
#         # print(self.obstacles)
#         return obs

#     def drawPath(self,path, smooth_nodes):
#         print("len path : ",len(path))
#         # print("smooth path : ",len(smooth_nodes)//5)
#         for node in path:
#             pygame.draw.circle(self.map, color = self.Red, center = node, radius = self.nodeRad)    
#         for smooth_node_idx in range(1, len(smooth_nodes)):
#             # if smooth_node_idx%5==0:
#                 # print("path : ",path[smooth_node_idx//5])
#                 # print(smooth_nodes[smooth_node_idx-1],smooth_nodes[smooth_node_idx])
#             pygame.draw.line(self.map, self.Blue,smooth_nodes[smooth_node_idx-1],smooth_nodes[smooth_node_idx],2)

            

class Map():
    def __init__(self,start_pos,clearance, map_dim, unit="cm",pygame_interact = False):
        self.unit = {
            "mm" : 1000,
            "cm" : 100
        } # w.r.t meter
        self.unit_rate = self.unit[unit]
        self.wheel_radius = .102*self.unit_rate # mm -> unit
        self.wheel_dist = .762*self.unit_rate # m -> unit
        self.time_step = 0.1 # sec
        self.robot_radius = .539*self.unit_rate #m -> unit
        self.total_cost = 0

        self.clearance=round(clearance+self.robot_radius)

        # Physical turtle bot robot limitation 
        self.MAX_LINEAR_VELOCITY = 22   # cm/s - Safety margin
        self.MAX_ANGULAR_VELOCITY = 2.84   # rad/s


        print("clearance : ",self.clearance)
        print("")

        #robot_radius = 1
        # self.clearance = 5 + self.robot_radius
        # print(start_pos, goal_pos)

        # print(self.start_pos, self.goal_pos)
        self.map_dim = map_dim # H,W


        # self.rect_coords = [
        #     [70,38,85,53],
        #     [145,0,160,15],
        #     [220,38,235,53],
        #     [70,93,85,108],
        #     [220,93,235,108],
        #     [145,125,160,140],
        #     [70,148,85,163],
        #     [220,148,235,163],
        #     [145,185,160,200],
        #     [145,60,160,75]
        #     ]

        self.rect_coords = [[3.000,2.000,3.800,3.000],
                            [5.800,2.000,6.600,3.000],
                            [3.000,7.000,3.800, 8.000],
                            [5.800,7.000,6.600,8.000],
                            [2.825,4.825,3.175, 5.350],
                            [4.025,2.325,4.375, 2.675],
                            [4.025,7.325,4.375, 7.675]]
        rect_coords = []
        for coord in self.rect_coords:
            rect_coords.append(list(map(lambda x : int(round(x*self.unit_rate)),coord)))
        self.rect_coords = rect_coords
        # self.rect_coords = map(lambda x: round())

        self.map = self.Create_Map() 
        self.map_size = self.map.shape
        self.goal_node_idx = None

        self.edit_start_pos = self.process_pos2unit(start_pos)
        #goal_pos = [550,220] # (x,y) user input
        # # because index start from 0, so handling edge cases when the pos is same as the height or width of the map

        self.edit_goal_pos = None
        # self.edit_goal_pos = list(map(int,(self.goal_pos[0],self.map_size[0]-self.goal_pos[1]-1))) # edit to make it according to array index which is top left as origin to bottom left as origin
        
        print("start pos : ",self.edit_start_pos)
        # print("goal pos : ",self.edit_goal_pos)

        self.get_table_poses()
        print("Table Pose : ", self.tablePoses)

        self.grey = (70,70,70)
        self.Blue = (255, 0, 0)
        self.Green = (0, 255, 0)
        self.Red = (0, 0, 255)
        self.white = (255,255,255)

        self.nodeRad=4
        self.nodeThickness = 0
        self.edgeThickness = 1


        #pygame map for better visulisation
        self.pygame_interact = pygame_interact
        self.pygame_map = None
        if self.pygame_interact:
            self.MapWindowName = "RRT path planning"
            pygame.display.set_caption(self.MapWindowName)
            self.MapH, self.MapW = self.map_dim
            self.pygame_map = pygame.display.set_mode((self.MapW, self.MapH))
            self.pygame_map.fill((255,255,255))
            self.Create_MapPygame()

    def set_goal_pos(self, goal_pos):
        self.edit_goal_pos = goal_pos
        if not self.check_nodes():
            # Exit if goal and start position is not satisfying certain condition
            # sys.exit(0)
            return False
        return True

    def draw_map(self, start_pos, goal_pos):
        cv2.circle(self.map, (start_pos[0], start_pos[1]), 5, (0,255,0), 2)
        cv2.circle(self.map, (goal_pos[0], goal_pos[1]), 5, (0, 0, 255), 2)
        cv2.imwrite("map.jpg", self.map)


    def process_pos2unit(self,pos):
        # converting from m to mm
        ang= pos[2]
        pos = tuple(map(lambda x:int(x*self.unit_rate), pos))
        if pos[1] >= self.map_size[0]:
            pos[1] = self.map_size[0]-1
        if pos[0] >= self.map_size[1]:
            pos[0] = self.map_size[1]-1
        pos = list(map(int,(pos[0],self.map_size[0]-pos[1]-1, ang))) # edit to make it according to array index which is top left as origin to bottom left as origi
        return pos

    def process_pos2m(self,pos):
        # converting from cm to m
        ang= pos
        pos = list(map(lambda x:(x/self.unit_rate), pos))
        pos[-1] = ang
        return pos

    def drawRectangles(self,canvas):
        for x1,y1,x2,y2 in self.rect_coords:
            canvas = cv2.rectangle(canvas, pt1=(x1,y1), pt2=(x2,y2), color=(0,255,0), thickness=-1)
        return canvas

    def addClearance(self, x, y, canvas, clearance):
        R,C,_ = canvas.shape
        for x1,y1,x2,y2 in self.rect_coords:
            y_top = max(y1-clearance,0)
            x_top = max(x1-clearance,0)
            x_bot = min(x2+clearance,C)
            y_bot = min(y2+clearance,R)
            if y_top<y and y_bot>y and x_top<x and x_bot>x:
                canvas[y,x,0]=255 
                break
        return canvas



    def check_nodes(self):
        column_limit = None
        row_limit = None
        Flag = True
        for c in range(self.clearance+1, self.map.shape[1]-self.clearance):
            su = np.sum(self.map[:,c,0]>0)
            # print(su, c)
            if su == self.map.shape[0]:
                column_limit = c

        for r in range(self.clearance+1,self.map.shape[0]-self.clearance):
            su = np.sum(self.map[r,:,0]>0)
            if su == self.map.shape[1]:
                row_limit = r

        if column_limit:
            if self.edit_goal_pos[0] > column_limit and self.edit_start_pos[0] < column_limit:
                Flag = False
                print("please enter node again, not able to reach the goal")
            elif self.edit_goal_pos[0] < column_limit and self.edit_start_pos[0] > column_limit:
                print("please enter node again, not able to reach the goal")    
                Flag=False
        if row_limit:
            if self.edit_goal_pos[1] > row_limit and self.edit_start_pos[1] < row_limit:
                Flag = False
                print("please enter node again, not able to reach the goal")
            elif self.edit_goal_pos[1] < row_limit and self.edit_start_pos[1] > row_limit:
                print("please enter node again, not able to reach the goal")    
                Flag = False
        if self.map[self.edit_goal_pos[1], self.edit_goal_pos[0],0] > 0 or self.map[self.edit_start_pos[1],self.edit_start_pos[0],0] > 0:
            print("please enter node again, as it is coinciding with the obstacles")
            Flag = False
        return Flag
    
    def Create_Map(self):
        # in cm
        canvas = np.zeros((*self.map_dim,3))

        clearance = self.clearance# cm

        canvas = self.drawRectangles(canvas)

        ## Adding clearance
        for y in range(canvas.shape[0]):
            for x in range(canvas.shape[1]):
                
                canvas = self.addClearance(x, y, canvas, clearance)
  
                if clearance>=y or canvas.shape[0] - clearance<=y:
                    canvas[y,x,0]=255  

        return canvas
    
    def Create_MapPygame(self):
        R,C = self.map_dim
        self.pygame_obs = []
        for x1,y1,x2,y2 in self.rect_coords:
            y_top = max(y1-self.clearance,0)
            x_top = max(x1-self.clearance,0)
            x_bot = min(x2+self.clearance,C)
            y_bot = min(y2+self.clearance,R)
            upper = (x_top,y_top)
            box_dim = (x_bot-x_top,y_bot-y_top)
            rectang = pygame.Rect(upper,box_dim)
            pygame.draw.rect(self.pygame_map, self.grey, rectang)
            self.pygame_obs.append(rectang)
        
        # top boundary
        upper = (0,0)
        box_dim = (self.clearance,self.clearance)
        rectang = pygame.Rect(upper, box_dim)
        pygame.draw.rect(self.pygame_map, self.grey, rectang)
        self.pygame_obs.append(rectang)

        upper = (self.MapW-1-self.clearance,self.MapH-self.clearance-1)
        box_dim = (self.clearance,self.clearance)
        rectang = pygame.Rect(upper, box_dim)
        pygame.draw.rect(self.pygame_map, self.grey, rectang)
        self.pygame_obs.append(rectang)

    def get_table_poses(self):
        # this are center position of the table
        # robot will always goto the front of the table and front of the
        # table is facing the origin, so after adding clearance the point will put here
        
        R,C,_ = self.map_size 
        self.tablePoses = {} 
        for idx, rect_coord in enumerate(self.rect_coords):
            x1,y1,x2,y2 = rect_coord
            y_top = max(y1-self.clearance,0)
            x_top = max(x1-self.clearance,0)
            x_bot = min(x2+self.clearance,C)
            y_bot = min(y2+self.clearance,R)
            
            # for front facing position the x (top left with clearance) 
            # will remain same but y will be center left with clearance
            center_y  = y1+(y2-y1)//2
            self.tablePoses[str(idx)] = self.tablePoses.get(str(idx), (x_top, center_y, 0))

        return self.tablePoses


    def draw_path(self, path, smooth_path):
        # def record_video(self,s2g_poses):
        new_canvas = self.map.copy().astype(np.uint8)

        FPS = len(path)//5
        FPS = FPS if FPS > 0 else 60
        
        size = (new_canvas.shape[1],new_canvas.shape[0])
        # Below VideoWriter object will create
        # a frame of above defined The output 
        # is stored in 'filename.avi' file.
        print(os.path.expanduser('~'))
        home_dir = os.path.expanduser('~')
        result = cv2.VideoWriter(os.path.join(home_dir,'filename.avi'), 
                                cv2.VideoWriter_fourcc(*'MJPG'),
                                FPS, size)


        for pos_idx in range(1,len(path)):
            new_pos = path[pos_idx]
            prev_pos = path[pos_idx-1]

            new_canvas = cv2.circle(new_canvas, new_pos, 2, self.Blue,2)
            new_canvas = cv2.line(new_canvas,prev_pos, new_pos, self.grey,2)
            # new_canvas = cv2.arrowedLine(new_canvas, (int(prev_pos[0]), int(prev_pos[1])),(int(new_pos[0]),int(new_pos[1])),(255,255,255), thickness=1,tipLength=0.5) 
            # new_canvas = plot_curve(prev_pos[0],prev_pos[1], prev_pos[2],action[0], action[1], new_canvas)
            result.write(new_canvas)
            #print(pos)
            # Display the frame
            # saved in the file
            cv2.imshow('Frame', new_canvas)

            # Press S on keyboard 
            # to stop the process
            if cv2.waitKey(10) & 0xFF == ord('s'):
                break
            # plt.show()
        cv2.imwrite("RRt_image.png",new_canvas)
        for pos_idx in range(1,len(smooth_path)):
            new_pos = (int(smooth_path[pos_idx][0]),int(smooth_path[pos_idx][1]))
            prev_pos = (int(smooth_path[pos_idx-1][0]),int(smooth_path[pos_idx-1][1]))
            # new_canvas[int(new_pos[1]),int(new_pos[0]),2] = 255
            # for action in self.actions:
            # print("Next Position : ",new_pos)
 
            new_canvas = cv2.line(new_canvas,prev_pos, new_pos,self.Red,2)
            # new_canvas = cv2.arrowedLine(new_canvas, (int(prev_pos[0]), int(prev_pos[1])),(int(new_pos[0]),int(new_pos[1])),(255,255,255), thickness=1,tipLength=0.5) 
            # new_canvas = plot_curve(prev_pos[0],prev_pos[1], prev_pos[2],action[0], action[1], new_canvas)
            result.write(new_canvas)
            #print(pos)
            # Display the frame
            # saved in the file
            cv2.imshow('Frame', new_canvas)

            # Press S on keyboard 
            # to stop the process
            if cv2.waitKey(10) & 0xFF == ord('s'):
                break
            # plt.show()
        cv2.imwrite("RRt_smooth_image.png",new_canvas)
        result.release()

            
        # Closes all the frames
        cv2.destroyAllWindows()

class RRTStarGraph():
    def __init__(self, start, mapDimension, map,unit):
        self.unit = {
            "mm" : 1000,
            "cm" : 100
        } # w.r.t meter
        self.unit_rate = self.unit[unit]
        
        (x,y,ang) = start
        # self.goal = goal
        self.start = (x,y)

        self.x = []
        self.y = []
        self.parent = []

        # initialize the tree
        self.x.append(x)
        self.y.append(y)
        self.parent.append(0)

        self.path_smooth = defaultdict(list)
        self.interpolate_step_size = 5
        self.interpolation_factor = 1


        # backtracking and stopping the search
        self.goalState = None
        self.path = []
        self.goalFlag = False

        # Map details
        self.MapDimensions = mapDimension
        self.MapH, self.MapW = self.MapDimensions
        self.map = map.map
        self.pygame_map = map.pygame_map
        self.map_class = map

        # self.obstacles= obstacles

        # set step size m -> unit
        self.dmax = 0.5 * self.unit_rate

        # set goal region m -> unit
        self.goalRadius = .5 * self.unit_rate

        self.rewire_nodes_nearby_radius = .200 * self.unit_rate #mm
        self.cost = [0]

    def add_node(self, n, x, y):
        # n is the index
        # x and y are position
        self.x.insert(n,int(x))
        self.y.insert(n,int(y))
    
    def remove_node(self, n):
        self.x.pop(n)
        self.y.pop(n)


    def add_edge(self, parent, child):
        # child is index of new position
        # parent is also index of the prev position
        self.parent.insert(child, parent)


    def remove_edge(self, n):
        self.parent.pop(n)


    def number_of_nodes(self):
        return len(self.x)
    
    def distance(self, n1, n2):
        # n1 node index
        # n2 another node index
        x1, y1 = (self.x[n1], self.y[n1])
        x2, y2 = (self.x[n2], self.y[n2])

        dist=  math.sqrt(math.pow(x1-x2,2)+math.pow(y1-y2,2))
        
        return dist
    
    def sample_envir(self):
        # generate random node from the environment
        x = int(random.uniform(0,self.MapW))
        y = int(random.uniform(0,self.MapH))
        return x, y
    
    def nearest(self, n):
        # check whether which previous node is nearest to new node for connection
        dmin = self.distance(0,n)
        nnear = 0
        for i in range(0,n):
            if self.distance(i, n)<dmin:
                dmin = self.distance(i, n)
                nnear = i
        return nnear
    
    def nnearest(self, n):
        # check whether which previous node is nearest to new node for connection
        Lnear = []
        dmin = self.rewire_nodes_nearby_radius
        for n_prev in range(self.number_of_nodes()-1):
            if self.distance(n_prev, n)<dmin:
                Lnear.append(n_prev)

        return Lnear

    def generate_nearGoalNode(self):
        x = int(random.uniform(self.goal[0]-self.goalRadius,self.goalRadius+self.goal[0]))
        y = int(random.uniform(self.goal[1]-self.goalRadius,self.goalRadius+self.goal[1]))
        return x, y

    def step(self,nnear, nstep):
        # Taking the step towards the node which has distance more than step size

        d = self.distance(nnear, nstep)
        if d>self.dmax:
            (xnear, ynear) = (self.x[nnear], self.y[nnear])
            (xstep,ystep) = (self.x[nstep],self.y[nstep])
            (px,py) = (xstep-xnear,ystep-ynear)
            theta = math.atan2(py,px)
            x = xnear + self.dmax*math.cos(theta)
            y = ynear + self.dmax*math.sin(theta)
            self.remove_node(nstep)

            if abs(x-self.goal[0])<self.goalRadius and abs(y-self.goal[1])<self.goalRadius:
                self.add_node(nstep,self.goal[0],self.goal[1])
                self.goalState=nstep
                self.goalFlag = True
                # print("Found goal")
                # (x1,y1) = (self.x[nnear],self.y[nnear])
                # (x2,y2) = (self.goal[0],self.goal[1])

                # if not self.crossObstacle(x1,y1,x2,y2):
                #     self.goalState=nstep
                #     self.goalFlag = True
                    
            else:
                self.add_node(nstep,x,y)
        # return False

    def bias(self,):
        # bias the step towards goal (move towards goal) not generating random node
        n = self.number_of_nodes()
        self.add_node(n, self.goal[0], self.goal[1])
        nnear = self.nearest(n)
        self.step(nnear, n)
        # flag=False
        # if self.goalFlag:
        #     flag = True
        isConnected = self.connect(nnear, n)
        # while not isConnected and flag:
        #     x,y = self.generate_nearGoalNode()
        #     self.remove_node(n)
        #     self.add_node(n,x,y)
        #     nnear = self.nearest(n)
        #     self.step(nnear, n)
        #     isConnected = self.connect(nnear, n)
        #     if self.goalFlag:
        #         flag = True
        #     else:
        #         flag = False
        if isConnected and not self.goalFlag:
            Lnear = self.nnearest(n)
            self.connect_shortest_valid(n, Lnear)
            self.rewire(Lnear, n)

        return self.x, self.y, self.parent
    
    def expand(self):
        # more of exploration and randomly generating the node
        x,y = self.sample_envir()
        n = self.number_of_nodes()
        self.add_node(n,x,y)
        if self.isFree(n):
            nnear = self.nearest(n)
            self.step(nnear, n)
            isConnected = self.connect(nnear, n)
            if isConnected and not self.goalFlag:
                Lnear = self.nnearest(n)
                self.connect_shortest_valid(n,Lnear)
                self.rewire(Lnear, n)

        return self.x, self.y, self.parent


    def isFree(self, n):
        # n is node index
        n = self.number_of_nodes()-1
        (x,y) = self.x[n],self.y[n]

        if self.map[y,x,0]!=0:
            self.remove_node(n)
            return False
        return True
    
    def crossObstacle(self,x1,y1,x2,y2):
        # check if there is an obstacle between 2 points
        # For each obstacle we will perform interpolation

        for i in range(0,101):
            u = i/100
            x = x1*u+x2*(1-u)
            y = y1*u+y2*(1-u)
            if self.map[round(y),round(x),0]!=0:
                return True
        return False
    
        # obs = self.map_class.pygame_obs.copy()
        # # For each obstacle we will perform interpolation
        # while (len(obs)>0):
        #     rectang=obs.pop(0)
        #     for i in range(0,101):
        #         u = i/100
        #         x = x1*u+x2*(1-u)
        #         y = y1*u+y2*(1-u)
        #         if rectang.collidepoint(x,y):
        #             return True
        # return False


    def connect(self,n1,n2):
        # connect 2 nodes
        (x1,y1) = (self.x[n1],self.y[n1])
        (x2,y2) = (self.x[n2],self.y[n2])

        if self.crossObstacle(x1,y1,x2,y2):
            if self.goalFlag:
                # print("not adding edge for goal : ",n2, " on ",n1)
                # print("because coord has obs : ",x1,y1,x2,y2)
                self.goalState= None
                self.goalFlag = False
            self.remove_node(n2)
            return False
        else:

            self.add_edge(n1,n2)
            return True
        
    def path_to_goal(self):

        if self.goalFlag:
            print("got the goal : ", self.goalState)
            print("len : ", len(self.parent))
            self.path = []
            self.path.append(self.goalState)
            parent_n = self.parent[self.goalState]
            while parent_n!=0:
                self.path.append(parent_n)
                parent_n =self.parent[parent_n]
            self.path.append(0)
        return self.goalFlag

    def convertpos2realworld(self,pos):
        # in real world higher the y, robot will move in upwards while in map its opposite because origin are opposite
        (x,y) = pos
        newy = self.MapH-y-1
        newx = x
        return (newx,newy)
    
    def getPathCoords(self):
        pathCoords = []
        smooth_nodes = []
        rw_pathCoords = []
        rw_shortenedPath = []
        rw_smoothNodes =[]
        for n in self.path:
            (x,y) = (self.x[n], self.y[n])
            pathCoords.append((x,y))
            rw_pathCoords.append(self.convertpos2realworld((x,y)))

        # remove the unecessary points
        s2g_pathCoords =  list(reversed(pathCoords))
        shortened_path = [s2g_pathCoords.pop(0)]
        p = shortened_path[0]
        rw_shortenedPath.append(self.convertpos2realworld(p))
        stored_np = None
        while len(s2g_pathCoords)>0:
            np = s2g_pathCoords.pop(0)
            (x1,y1) = p
            (x2,y2) = np
            if not self.crossObstacle(x1,y1,x2,y2):
                stored_np = np
            else:
                # rw_smoothNodes.append
                smooth_nodes.append(self.interpolate(p, stored_np))
                shortened_path.append(stored_np)
                rw_shortenedPath.append(self.convertpos2realworld(stored_np))
                rw_smoothNodes.append(self.interpolate(self.convertpos2realworld(p), self.convertpos2realworld(stored_np)))
                p = stored_np
                s2g_pathCoords.insert(0, np)
        smooth_nodes.append(self.interpolate(p, stored_np))
        rw_smoothNodes.append(self.interpolate(self.convertpos2realworld(p), self.convertpos2realworld(stored_np)))             
        shortened_path.append(stored_np)
        rw_shortenedPath.append(self.convertpos2realworld(stored_np))

        smooth_nodes = [coords for smooth_node in smooth_nodes for coords in smooth_node ]
        rw_smooth_nodes = [coords for smooth_node in rw_smoothNodes for coords in smooth_node ]

        return (pathCoords, smooth_nodes, shortened_path), (rw_pathCoords,rw_shortenedPath,rw_smooth_nodes)  #smooth_nodes
    

    def compute_cost(self, n_new):
        p_idx = self.parent[n_new]
        self.cost.insert(n_new, self.cost[p_idx]+self.distance(p_idx, n_new))

    def update_cost(self, n_new, new_cost):
        self.cost[n_new] = new_cost
    
    def rewire(self, Lnear, n_new):
        # Lnear is the list containing the nodes near to the new node
        for n in Lnear:
            curr_cost = self.cost[n]
            cost = self.distance(n_new, n)+self.cost[n_new]
            if cost < curr_cost:
                # curr_cost = cost
                # best_parent_node = n
                (x1,y1) = (self.x[n],self.y[n])
                (x2,y2) = (self.x[n_new],self.y[n_new])

                if not self.crossObstacle(x1,y1,x2,y2):
                    curr_cost = cost
                    best_parent_node = n_new

                    self.remove_edge(n)
                    self.step(best_parent_node, n)
                    self.connect(best_parent_node, n)
                    self.update_cost(n, curr_cost)

        # pass
    def connect_shortest_valid(self, n_new, Lnear):
        self.compute_cost(n_new)
        curr_cost = self.cost[n_new]
        best_parent_node = self.parent[n_new]
        for n in Lnear:
            cost = self.distance(n_new, n)+self.cost[n]
            if cost < curr_cost :
                # curr_cost = cost
                # best_parent_node = n
                (x1,y1) = (self.x[n],self.y[n])
                (x2,y2) = (self.x[n_new],self.y[n_new])

                if not self.crossObstacle(x1,y1,x2,y2):
                    curr_cost = cost
                    best_parent_node = n


        if self.parent[n_new]==best_parent_node:
            # no new connections formed or no rewiring
            return False
        else:

            self.remove_edge(n_new)
            self.step(best_parent_node, n_new)
            if not self.connect(best_parent_node, n_new):
                print("issue")
            self.update_cost(n_new, curr_cost)
            return True

    def interpolate(self, coord1, coord2):
        # parent_node = self.parent[child_node]
        x1,y1 = coord1
        x2,y2 = coord2
        dist = math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
        num_steps = self.interpolate_step_size #int(dist / self.interpolate_step_size)
        x_step = (x2 - x1) / num_steps
        y_step = (y2 - y1) / num_steps
        
        # Create a list of points between the two nodes
        points = []
        for i in range(num_steps):
            x = x1 + i * x_step
            y = y1 + i * y_step
            points.append((x, y))
        points.append((x2, y2))
        
        # Apply cubic interpolation to smooth the points
        t = [i / (len(points) - 1) for i in range(len(points))]
        x = [p[0] for p in points]
        y = [p[1] for p in points]
        tnew = np.linspace(0, 1, num=(len(points)-1)*self.interpolation_factor+1)
        xnew = scipy.interpolate.interp1d(t, x, kind='cubic')(tnew)
        ynew = scipy.interpolate.interp1d(t, y, kind='cubic')(tnew)
        
        # Pack the smoothed points into a list of tuples
        smoothed_points = [(xnew[i], ynew[i]) for i in range(len(tnew))]
        
        return smoothed_points

    def search(self, goal):
        i =0 
        self.goal = goal
        # while i<Iterations:
        print("Searching for best path ----------")
        start_time = time.perf_counter()
        while not self.path_to_goal():
            try:
                if i%10==0:
                    # probability of going towards goal 10%
                    x,y,parent = self.bias()
                
                else:
                    # probability of exploring the map 90%
                    x,y,parent = self.expand()
                
                if self.pygame_map:
                    pygame.draw.circle(self.pygame_map, color = self.map_class.Blue, center = (x[-1],y[-1]), radius = self.map_class.nodeRad)
                    parent_n = graph.parent[-1]
                    pygame.draw.line(self.pygame_map, color= self.map_class.grey, start_pos=(x[-1],y[-1]), end_pos=(x[parent_n], y[parent_n]),width=self.map_class.edgeThickness)
                
                    i+=1
                    if i%5==0:
                        # update the map to show
                        pygame.display.update()
            except Exception as e:
                print("Error : ",e)
                return False        
            i+=1
        print("Got the best path-----------")
        print("time taken : ", time.perf_counter()-start_time," sec")
        return True


if __name__=="__main__":
    unit = {"cm" : 100, "mm":1000} #w.r.t m

    desired_unit = "cm"
    unit_rate = unit[desired_unit] 
    dimensions = (10*unit_rate,10*unit_rate)
    start = [0,5,0]
    goal = [5.15,7.5,0]
    # goal = [2,2.5,0]
    obsdim = 30
    obsnum = 50
    Iterations= 500
    clearance = .1 * unit_rate
    pygame_interact = False
    if pygame_interact:
        pygame.init()

    rest_map = Map(start_pos=start,clearance=clearance,map_dim=dimensions,unit=desired_unit, pygame_interact = pygame_interact)
    

    start_pos = rest_map.process_pos2unit(start)
    goal_pos = rest_map.process_pos2unit(goal)
    ret = rest_map.set_goal_pos(goal_pos)

    rest_map.draw_map(start_pos, goal_pos)
    if not ret:
        print("Obstacles in goal")
        sys.exit(0)
    goal_pos = rest_map.edit_goal_pos
    rest_map.draw_map(start_pos, goal_pos)
    # started search

    graph = RRTStarGraph(start_pos, mapDimension=rest_map.map_dim, map=rest_map, unit=desired_unit)
    print("Started Searching --------")
    graph.search(goal_pos)    

    if pygame_interact:
        pygame.display.update()
        pygame.event.clear()
        pygame.event.wait(0)

    print("get path")
    (path, smooth_path, short_path),RW_coords_paths = graph.getPathCoords()
    rest_map.draw_path(path, smooth_path)
