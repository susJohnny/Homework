from operator import ne
from typing import final
from typing_extensions import Self
from xml.dom.minicompat import NodeList
import numpy as np
import queue
from math import sqrt

from pyrfc3339 import generate

class Astar:
    def __init__(self):
        # graph node: x, y, cost, parent
        self.cur_vis = []   # current visited node list
        self.next_vis = []  # next visit node list

    def find_path(self, graph, start_pos, goal_pos):
        
        print('start to Astar search') 

        frontier = queue.PriorityQueue()  # priority queue for algorithm to explore current points.
        frontier.put((0, start_pos))  # put the priority and position

        self.cur_vis.append([start_pos.x, start_pos.y])
        self.next_vis.append([start_pos.x, start_pos.y])
        
        cost_so_far = np.zeros((graph.width, graph.height)) # cost matrix from start point to this point
        
        final_node = None
        cnt=0;
        while not frontier.empty():
            cnt+=1
            element=frontier.get()
            curNode=element[1];
            self.cur_vis.append([curNode.x,curNode.y])
            if curNode.x==goal_pos.x and curNode.y==goal_pos.y:
                final_node=curNode
                break
            nextList= graph.neighbors(curNode)
            for node in nextList:
                if [node.x,node.y] in self.next_vis:
                    if self.changeMap(cost_so_far,node):
                        frontier.put((self.final_cost(node,goal_pos),node))
                else:
                    self.next_vis.append([node.x,node.y])
                    # cost_so_far[node.x][node.y]=node.cost+cost_so_far[node.parent.x][node.parent.y]
                    frontier.put((self.final_cost(node,goal_pos),node))
                          

            
            # print('you should complete this part to find the path')
            # please complete this part for the homework question1 
            # each node has four parts: x position, y position, the cost so far, the parent node. Utilize the parent node, the path can be generated
            # parts: (1) get current node with priority (using frontier.get()[1])  
            # (2) check whether current node is the goal node (using graph.node_equal) 
            # (3) explore the neighbors of current node (using graph.neighbors)
            # (4) if the neighbor node is not in the next_vis (and cur_vis): put that in the frontier with priority and append that in the next_vis.  (priority= cost_so_far + heuristic)
            # (5) if the neighbor node is in the next_vis: check whether cost so far is less than that in the next_vis to determine whether put it in the frontier
            # (6) return the node when it is in the goal position.
            # print(start_pos)
            # print(cost_so_far)   
            # final_node=start_pos
            # nextList=graph.neighbors(start_pos)
            # for node in nextList:
            #     self.cur_vis.append([node.x,node.y])
            #     frontier.put((node.cost,node))
            # print(frontier.get()[0])
            # break
            
        print(cnt)
        print('search done')

        return final_node, self.cur_vis

    def heuristic(self, node1, node2, coefficient=1):
        # please complete the heuristic function for the homework question1  (related to the distance to the goal)
        # print('You should complete the heuristic function')
        # pass
        distance=coefficient*sqrt((node1.x-node2.x)**2+(node1.y-node2.y)**2)
        return distance;
        
    def final_cost(self,node,goal):
        return self.heuristic(node,goal,2)+node.cost

    def changeMap(self,map,node):
        if node.cost<map[node.x][node.y]:
            map[node.x][node.y]=node.cost
            return True
        return False

    def generate_path(self, final_node):
        # utilize the node to generate the path. 

        path = [ [final_node.x, final_node.y] ]
        
        while final_node.parent is not None:
            path.append( [final_node.parent.x, final_node.parent.y] )
            final_node = final_node.parent
        
        return path





    

