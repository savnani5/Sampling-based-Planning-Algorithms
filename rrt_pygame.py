import math
import random

import matplotlib.pyplot as plt
import numpy as np
import pygame
import sys
import time
show_animation = True


class RRT:
    """
    Class for RRT planning
    """

    class Node:
        """
        RRT Node
        """
        def __init__(self, x, y):
            self.x = x
            self.y = y
            self.path_x = []
            self.path_y = []
            self.parent = None
            
        def __repr__(self):
            return f'X: {self.x},Y: {self.y}'

    def __init__(self,start,goal,rand_area,expand_dis=0.1,path_resolution=0.05, goal_sample_rate = 1, max_iter=3000):
        """
        Setting Parameter
        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y,size],...]
        randArea:Random Sampling Area [min,max]
        """
        self.start = self.Node(start[0], start[1])
        self.end = self.Node(goal[0], goal[1])
        self.min_rand = rand_area[0]
        self.max_rand = rand_area[1]
        self.expand_dis = expand_dis
        self.path_resolution = path_resolution
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.node_list = []

    def planning(self, animation=True):
        """
        rrt path planning
        animation: flag for animation on or off
        """

        pygame.init()
        screen = pygame.display.set_mode((100*10, 100*10))
        counter = 0
        path = None

        while(1):
            screen.fill((0,0,0))
            pygame.draw.circle(screen, (0,0, 255), (100*2, 100*8), 100*1)
            pygame.draw.polygon(screen, (0,0, 255), ((100*0.25, 100*4.25), (100*1.75, 100*4.25), (100*1.75, 100*5.75) ,(100*0.25, 100*5.75)))
            pygame.draw.polygon(screen, (0,0, 255), ((100*3.75, 100*4.25),(100*6.25,100*4.25),(100*6.25,100*5.75) ,(100*3.75, 100*5.75)))
            pygame.draw.circle(screen, (0,0, 255), (100*2, 100*2), 100*1)
            pygame.draw.polygon(screen, (0,0, 255), ((100*7.25, 100*6), (100*8.75, 100*6), (100*8.75, 100*8), (100*7.25, 100*8)))
            
            # Goal threshold
            pygame.draw.circle(screen, (0,255,0), (100*self.end.x, 100*10-(100*self.end.y)), 20)
        
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()

            if counter == 0:

                self.node_list = [self.start]
                pygame.draw.circle(screen, (255,255,0), (100*self.start.x, 100*10-(100*self.start.y)), 5)
            
                for i in range(self.max_iter):
                            
                    rnd_node = self.get_random_node()
                    nearest_ind = self.get_nearest_node_index(self.node_list, rnd_node)
                    nearest_node = self.node_list[nearest_ind]
                    new_node = self.steer(nearest_node, rnd_node, self.expand_dis)
                
                    if self.check_collision(new_node):
                        self.node_list.append(new_node)

                        # Drawing the nodes and edges
                        pygame.draw.circle(screen, (0,255,255), (100*new_node.x, 100*10 - 100*new_node.y), 2)
                        pygame.draw.line(screen, (255,255,255), (100*nearest_node.x, 100*10 - 100*nearest_node.y), (100*new_node.x, 100*10 - 100*new_node.y))

                    if self.calc_dist_to_goal(self.node_list[-1].x, self.node_list[-1].y) <= self.expand_dis:
                        print('reached!')
                        final_node = self.steer(self.node_list[-1], self.end, self.expand_dis)
                        
                        if self.check_collision(final_node):
                            path = self.generate_path(len(self.node_list) - 1)
                    
                    # time.sleep(0.02)
                    pygame.display.update()
                    
                if path is None:
                    print("Cannot find path")
                else:
                    for point in path:
                        pygame.draw.circle(screen, (255,0,0), (100*point[0], 10*100-100*point[1]), 5)

                    print("found path!!")
                pygame.display.update()
            
            counter +=1


    # def steer(self, from_node, to_node, extend_length=float("inf")):
    def steer(self, from_node, to_node, extend_length=float("inf")):

        new_node = self.Node(from_node.x, from_node.y)
        d, theta = self.calc_distance_and_angle(new_node, to_node)

        new_node.path_x = [new_node.x]
        new_node.path_y = [new_node.y]

        if extend_length > d:
            extend_length = d

        n_expand = math.floor(extend_length / self.path_resolution)

        for _ in range(n_expand):
            new_node.x += self.path_resolution * math.cos(theta)
            new_node.y += self.path_resolution * math.sin(theta)
            new_node.path_x.append(new_node.x)
            new_node.path_y.append(new_node.y)

        d, _ = self.calc_distance_and_angle(new_node, to_node)
        if d <= self.path_resolution:
            new_node.path_x.append(to_node.x)
            new_node.path_y.append(to_node.y)
            new_node.x = to_node.x
            new_node.y = to_node.y

        new_node.parent = from_node

        return new_node

    def generate_path(self, goal_ind):
        path = [[self.end.x, self.end.y]]
        node = self.node_list[goal_ind]
        while node.parent is not None:
            path.append([node.x, node.y])
            node = node.parent
        path.append([node.x, node.y])
        return path

    def calc_dist_to_goal(self, x, y):
        dx = x - self.end.x
        dy = y - self.end.y
        return math.hypot(dx, dy)

    def get_random_node(self):
        if random.randint(0, 100) > self.goal_sample_rate:
            rnd = self.Node(
                random.uniform(0, 10),
                random.uniform(0, 10))
        else:  # goal point sampling
            rnd = self.Node(self.end.x, self.end.y)
        return rnd

    @staticmethod
    def get_nearest_node_index(node_list, rnd_node):
        dlist = [(node.x - rnd_node.x)**2 + (node.y - rnd_node.y)**2
                 for node in node_list]
        minind = dlist.index(min(dlist))

        return minind


    @staticmethod
    def check_collision(node):

        if node is None:
            return False
        x = node.x
        y = node.y
        
        if node.path_x:
            # Boundary condition
            if (x < 0) or (x > 10) or (y < 0) or (y > 10): 
                
                return False
            
            # Obstacle 1 (Circle Up)
            
            elif (x-2)**2 + (y-8)**2 - (1)**2 <= 0: 
                print('Circle Up!')

                return False
            
            # Obstacle 2 (Square) 
            elif x >= 0.25 and x <= 1.75 and y >= 4.25 and y <= 5.75: 
                print('Square!')
     
                return False
            
            # Obstacle 3 (Rectangle Up)
            elif x >= 3.75 and x <= 6.25 and y >= 4.25 and y <= 5.75: 
                print('Rectangle Up!')
         
                return False
            
              # Obstacle 4 (Circle Down)
            elif (x-2)**2 + (y-2)**2 - (1)**2 <= 0:
                print('Circle Down!')
  
                return False
            
            # Obstacle 3 (Rectangle Down)
            elif x >= 7.25 and x <= 8.75 and y >= 2 and y <= 4: 
                print('Rectangle Down!')
    
                return False
            
            for (x, y) in zip(node.path_x, node.path_y):
                
                
                # Boundary condition
                if (x < 0) or (x > 10) or (y < 0) or (y > 10): 
                    
                    return False
                
                # Obstacle 1 (Circle Up)
                
                elif (x-2)**2 + (y-8)**2 - (1)**2 <= 0: 
                    print('Circle Up!')

                    return False
                
                # Obstacle 2 (Square) 
                elif x >= 0.25 and x <= 1.75 and y >= 4.25 and y <= 5.75: 
                    print('Square!')
         
                    return False
                
                # Obstacle 3 (Rectangle Up)
                elif x >= 3.75 and x <= 6.25 and y >= 4.25 and y <= 5.75: 
                    print('Rectangle Up!')
             
                    return False
                
                  # Obstacle 4 (Circle Down)
                elif (x-2)**2 + (y-2)**2 - (1)**2 <= 0:
                    print('Circle Down!')
      
                    return False
                
                # Obstacle 3 (Rectangle Down)
                elif x >= 7.25 and x <= 8.75 and y >= 2 and y <= 4: 
                    print('Rectangle Down!')
        
                    return False
                
                else:
                    # Node in Freespace
                    return True 
        return True
        

    @staticmethod
    def calc_distance_and_angle(from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        d = math.hypot(dx, dy)
        theta = math.atan2(dy, dx)
        return d, theta


def main(gx=6.0, gy=10.0):
    print("start " + __file__)

    # ====Search Path with RRT====
   
    # Set Initial parameters
    rrt = RRT(
        start=[0, 0],
        goal=[gx, gy],
        rand_area=[0, 100*10],)
    
    rrt.planning(animation=show_animation)

if __name__ == '__main__':
    main()