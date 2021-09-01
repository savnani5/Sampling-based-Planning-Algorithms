import math
import time
import os
import sys
import pygame
import matplotlib.pyplot as plt

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../RRT/")

try:
    from rrt_pygame import RRT
except ImportError:
    print('x')
    raise

show_animation = True


class RRTStar(RRT):
    """
    Class for RRT Star planning
    """

    class Node(RRT.Node):
        def __init__(self, x, y):
            super().__init__(x, y)
            self.cost = 0.0

    def __init__(self,start,goal,rand_area,expand_dis=0.1,path_resolution=0.05,goal_sample_rate= 10, max_iter=3000, connect_circle_dist=3.0,search_until_max_iter=True):
        """
        Setting Parameter
        start:Start Position [x,y]
        goal:Goal Position [x,y]
        randArea:Random Sampling Area [min,max]
        """
        super().__init__(start, goal, rand_area, expand_dis,
                         path_resolution, goal_sample_rate, max_iter)
        self.connect_circle_dist = connect_circle_dist
        self.goal_node = self.Node(goal[0], goal[1]) 
        self.search_until_max_iter = search_until_max_iter

    def planning(self, animation=True):
        """
        rrt star path planning
        animation: flag for animation on or off .
        """

        pygame.init()
        screen = pygame.display.set_mode((100*10, 100*10))
        counter = 0
        path = None

        while(1):

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()            


            if counter == 0:
                self.node_list = [self.start]

                
                for i in range(self.max_iter):

                            
                    print("Iter:", i, ", Number of nodes:", len(self.node_list))
                    rand = self.get_random_node()
                    print(rand)
                    nearest_ind = self.get_nearest_node_index(self.node_list, rand)
                    nearest_node = self.node_list[nearest_ind]
                    new_node = self.steer(nearest_node, rand,
                                        self.expand_dis)
                    print(new_node)
                    new_node.cost = nearest_node.cost + \
                        math.hypot(new_node.x-nearest_node.x,
                                new_node.y-nearest_node.y)
                    pygame.draw.circle(screen, (0,255,255), (100*new_node.x, 100*10 - 100*new_node.y), 2)
                    pygame.display.update()

                    if self.check_collision(new_node):
                        near_nodes = self.find_near_nodes(new_node)
                        node_with_updated_parent = self.choose_parent(new_node, near_nodes)        

                        if node_with_updated_parent:
                            print("in if node")

                            self.rewire(node_with_updated_parent, near_nodes)
                            self.node_list.append(node_with_updated_parent)
                            
                        else:
                            self.node_list.append(new_node)

                    if animation:
                        screen.fill((0,0,0))
                        pygame.draw.circle(screen, (0,0, 255), (100*2, 100*8), 100*1)
                        pygame.draw.polygon(screen, (0,0, 255), ((100*0.25, 100*4.25), (100*1.75, 100*4.25), (100*1.75, 100*5.75) ,(100*0.25, 100*5.75)))
                        pygame.draw.polygon(screen, (0,0, 255), ((100*3.75, 100*4.25),(100*6.25,100*4.25),(100*6.25,100*5.75) ,(100*3.75, 100*5.75)))
                        pygame.draw.circle(screen, (0,0, 255), (100*2, 100*2), 100*1)
                        pygame.draw.polygon(screen, (0,0, 255), ((100*7.25, 100*6), (100*8.75, 100*6), (100*8.75, 100*8), (100*7.25, 100*8)))
                        
                        # Goal threshold
                        pygame.draw.circle(screen, (0,255,0), (100*self.end.x, 100*10-(100*self.end.y)), 20)
                        pygame.draw.circle(screen, (255,255,0), (100*self.start.x, 100*10-(100*self.start.y)), 5)                        

                        for node in self.node_list:
                            if node.parent:
                                # for (x,y) in zip(node.path_x, node.path_y):
                                pygame.draw.circle(screen, (0,255,255), (100*node.x, 100*10 - 100*node.y), 2)
                                pygame.draw.line(screen, (255,255,255), (100*node.x, 100*10 - 100*node.y), (100*node.parent.x, 100*10 - 100*node.parent.y))

                        pygame.display.update()
                        # time.sleep(0.02)
                    if ((not self.search_until_max_iter) and new_node):  # if reaches goal
                        last_index = self.search_best_goal_node()
                        if last_index is not None:
                            path =  self.generate_path(last_index)
                    
                    
                print("reached max iteration")

                last_index = self.search_best_goal_node()
                if last_index is not None:
                    path = self.generate_path(last_index)
                
                if path is None:
                    print("Cannot find path")
                else:
                    prev = None
                    for point in path:
                    
                        pygame.draw.circle(screen, (255,0,0), (100*point[0], 10*100-100*point[1]), 5)
                        if prev:
                            pygame.draw.line(screen, (255,0,0), (100*point[0], 10*100-100*point[1]), (100*prev[0], 100*10 - 100*prev[1]))
                        prev = point
                    print("found path!!")
                pygame.display.update()
            
            counter +=1

              

    # def choose_parent(self, new_node, near_inds):
    def choose_parent(self, new_node, near_nodes):
        """
        Computes the cheapest point to new_node contained in the list
        near_inds and set such a node as the parent of new_node.
            Arguments:
            --------
                new_node, Node
                    randomly generated node with a path from its neared point
                    There are not coalitions between this node and the tree.
                near_inds: list
                    Indices of indices of the nodes what are near to new_node
            Returns.
            ------
                Node, a copy of new_node
        """


        if not near_nodes:
            return None

        # search nearest cost in near_inds
        costs = []
        for near_node in near_nodes:

            t_node = self.steer(near_node, new_node)
            if t_node and self.check_collision(t_node):    # does this check only edge?
                costs.append(self.calc_new_cost(near_node, new_node))
            else:
                costs.append(float("inf"))  # the cost of collision node
        min_cost = min(costs)

        if min_cost == float("inf"):
            print("There is no good path.(min_cost is inf)")
            return None

        new_parent = near_nodes[costs.index(min_cost)]
        new_node = self.steer(new_parent, new_node)
        new_node.cost = min_cost


        return new_node

    def search_best_goal_node(self):

        dist_to_goal_list = [self.calc_dist_to_goal(n.x, n.y) for n in self.node_list]
        goal_inds = [
            dist_to_goal_list.index(i) for i in dist_to_goal_list
            if i <= self.expand_dis
        ]

        safe_goal_inds = []
        for goal_ind in goal_inds:
            t_node = self.steer(self.node_list[goal_ind], self.goal_node)
            if self.check_collision(t_node):
                safe_goal_inds.append(goal_ind)

        if not safe_goal_inds:
            return None

        min_cost = min([self.node_list[i].cost for i in safe_goal_inds])
        for i in safe_goal_inds:
            if self.node_list[i].cost == min_cost:
                return i

        return None

    def find_near_nodes(self, new_node):
        """
        1) defines a ball centered on new_node
        2) Returns all nodes of the tree that are inside this ball
            Arguments:
            ---------
                new_node: Node
                    new randomly generated node, without collisions between
                    its nearest node
            Returns:
            -------
                list
                    List with the indices of the nodes inside the ball of
                    radius r
        """
        nnode = len(self.node_list) + 1                                        # +1 ??
        r = self.connect_circle_dist * math.sqrt((math.log(nnode) / nnode))
        # print("Radius", r)
        # if expand_dist exists, search vertices in a range no more than
        # expand_dist
        # if hasattr(self, 'expand_dis'):
        #     r = min(r, self.expand_dis)
        dist_list = [(node.x - new_node.x)**2 + (node.y - new_node.y)**2 for node in self.node_list]
        near_inds = [dist_list.index(i) for i in dist_list if i <= r**2]
        near_nodes = [self.node_list[i] for i in near_inds]
        # return near_inds
        return near_nodes

  
    def rewire(self, new_node, near_nodes):
        """
            For each node in near_inds, this will check if it is cheaper to
            arrive to them from new_node.
            In such a case, this will re-assign the parent of the nodes in
            near_inds to new_node.
            Parameters:
            ----------
                new_node, Node
                    Node randomly added which can be joined to the tree
                near_inds, list of uints
                    A list of indices of the self.new_node which contains
                    nodes within a circle of a given radius.
            Remark: parent is designated in choose_parent.
        """
       
                
        for near_node in near_nodes:
            # i = self.node_list.index(near_node)
            # near_node = self.node_list[i]

            edge_node = self.steer(new_node, near_node)
            if not edge_node:                                                  # try commenting
                continue
            edge_node.cost = self.calc_new_cost(new_node, near_node)

            no_collision = self.check_collision(edge_node)
            improved_cost = near_node.cost > edge_node.cost
  
            if no_collision and improved_cost:
                near_node.x = edge_node.x
                near_node.y = edge_node.y
                near_node.cost = edge_node.cost
                near_node.path_x = edge_node.path_x
                near_node.path_y = edge_node.path_y
                near_node.parent = edge_node.parent
                self.propagate_cost_to_leaves(new_node)
                

    def calc_new_cost(self, from_node, to_node):
        d, _ = self.calc_distance_and_angle(from_node, to_node)
        return from_node.cost + d

    def propagate_cost_to_leaves(self, parent_node):

        for node in self.node_list:
            if node.parent == parent_node:
                node.cost = self.calc_new_cost(parent_node, node)
                self.propagate_cost_to_leaves(node)



def main():
    print("Start " + __file__)

    # ====Search Path with RRT====
    
    # Set Initial parameters
    rrt_star = RRTStar(
        start=[0, 0],
        goal=[6, 10],
        rand_area=[0, 10*100])
    
    rrt_star.planning(animation=show_animation)


if __name__ == '__main__':
    main()