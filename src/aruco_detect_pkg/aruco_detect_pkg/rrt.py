import math
import cv2
import random
import numpy as np
# import time

class Nodes:

    def __init__(self,x,y):
        
        self.x = x
        self.y = y
        self.parent_x = []
        self.parent_y = []


class RRT:

    def __init__(self, start_point, end_point, stepsize , iterlimit, bounding_boxes, img):

        self.nodeslist = []
        self.start_point = start_point
        self.end_point = end_point 

        self.obstacle_points = self.find_obstacle_points(bounding_boxes)
        self.stepsize = stepsize
        self.iterlimit = iterlimit

        self.nodeslist.append(Nodes(start_point[0],start_point[1]))
        self.nodeslist[0].parent_x.append(start_point[0])
        self.nodeslist[0].parent_y.append(start_point[1])

        self.img = img
        self.l, self.h , _ = img.shape
        

    def planning(self):

   
        if (self.end_point[0],self.end_point[1]) in self.obstacle_points or (self.end_point[1]<0 or self.end_point[1]>self.l or self.end_point[0]<0 or self.end_point[0]>self.h):
            print("Path Not Available!")
            return []
        
        self.draw_circle(self.start_point[0],self.start_point[1])
        self.draw_circle(self.end_point[0],self.end_point[1])
        
        
  
        dirConnection = self.check_collision(self.start_point[0],self.start_point[1],self.end_point[0],self.end_point[1])
        if not dirConnection:
            self.draw_line(self.start_point[0],self.start_point[1],self.end_point[0],self.end_point[1], thickness= 5)
            return [(self.start_point[0],self.start_point[1]),(self.end_point[0],self.end_point[1])]

        
        pathfound = False
        i = 1
        while (pathfound == False) and (i<self.iterlimit):

            new_x, new_y = self.rand_point()

            nearest_node_index = self.nearest_node(new_x,new_y)
            node_x,node_y = self.nodeslist[nearest_node_index].x , self.nodeslist[nearest_node_index].y

            x,y, nodeEdge, directEdge = self.check_connection(node_x,node_y,new_x,new_y)


            # If direct conncetion exists..
            if directEdge and nodeEdge:

                print(f"Node {i} can connect directly with end")
                self.nodeslist.append(Nodes(x,y))

                self.nodeslist[i].parent_x = self.nodeslist[nearest_node_index].parent_x.copy()
                self.nodeslist[i].parent_y = self.nodeslist[nearest_node_index].parent_y.copy()
                self.nodeslist[i].parent_x.append(x)
                self.nodeslist[i].parent_y.append(y)
                
                self.draw_circle(x,y)
                self.draw_line(x,y,self.nodeslist[nearest_node_index].x,self.nodeslist[nearest_node_index].y)
                self.draw_line(x,y,self.end_point[0],self.end_point[1], thickness= 5)

                print("Path has been found")

                for j in range(len(self.nodeslist[i].parent_x)-1):                    
                    self.draw_line(self.nodeslist[i].parent_x[j],self.nodeslist[i].parent_y[j],self.nodeslist[i].parent_x[j+1],self.nodeslist[i].parent_y[j+1], thickness = 5)
                
                
                pathfound = True            

                
                return self.path_cordinates(i)

            elif nodeEdge:

                print(f"Nodes {i-1} and {i} connected")
                
                self.nodeslist.append(Nodes(x,y))
                self.nodeslist[i].parent_x = self.nodeslist[nearest_node_index].parent_x.copy()
                self.nodeslist[i].parent_y = self.nodeslist[nearest_node_index].parent_y.copy()
                
                self.nodeslist[i].parent_x.append(x)
                self.nodeslist[i].parent_y.append(y)

                i += 1
                
               

                self.draw_circle(x,y)
                self.draw_line(x,y,self.nodeslist[nearest_node_index].x,self.nodeslist[nearest_node_index].y)

             

                continue

            else:
                print("No direct connection is poosible.. Generating new random position..")
                continue

        print("Path is not found")
        return []


    def path_cordinates(self,i):


        path_points = []
        for j in range(len(self.nodeslist[i].parent_x)):

            x = self.nodeslist[i].parent_x[j]
            y = self.nodeslist[i].parent_y[j]
            path_points.append((x,y))
        
        path_points.append((self.end_point[0],self.end_point[1]))
        return path_points

            

    def find_obstacle_points(self,obstacles):

       
        
        obstacle_points = []


        for obstacle in obstacles:
            
            obstacle = list(map(int,obstacle))

            x1 = min(obstacle[0],obstacle[6])
            x2 = max(obstacle[2],obstacle[4])
            y1 = min(obstacle[1],obstacle[3])
            y2 = max(obstacle[5],obstacle[7])

            for x in range(x1-1,x2+1):
                for y in range(y1-1,y2+1):
                    obstacle_points.append((x,y))


        return obstacle_points


    def check_collision(self,x1,y1,x2,y2):

        
        if x1 == x2:              
            return True

        if (x2,y2) in self.obstacle_points:
            return True        

        if x2>x1:
            xpoints = [x for x in range(x1,x2+1)]
        else:
            xpoints = [x for x in range(x2,x1+1)]

        slope = (y2-y1)/(x2-x1)

        for x in xpoints:

            y = int(slope*(x-x1) + y1)
            if (x,y) in self.obstacle_points:
                return True

        return False


    def rand_point(self):
        rand_x = random.randint(0,self.h)
        rand_y = random.randint(0,self.l)
        return (rand_x, rand_y)
    
    def dist_and_angle(self,x1,y1,x2,y2):
        dist = math.sqrt(((x1-x2)**2)+((y1-y2)**2))
        angle = math.atan2(y2-y1, x2-x1)
        return(dist,angle)

    def nearest_node(self,x,y):

       
        
        dis_list = []
        
        for node in self.nodeslist:
            dis, _ = self.dist_and_angle(node.x,node.y,x,y)
            dis_list.append(dis)

        return dis_list.index(min(dis_list))


    def check_connection(self,x1,y1,x2,y2):
       
        
        _,angle = self.dist_and_angle(x1,y1,x2,y2)
        x = int(x1 + self.stepsize*np.cos(angle))
        y = int(y1 + self.stepsize*np.sin(angle))

        # x and x1 should be different..
        if x == x1 :
            return (0,0,False,False)
       
       
      
        if y<0 or y>self.l or x<0 or x>self.h:
            directEdge = False
            nodeEdge = False

        else:
            
            
            if self.check_collision(x,y,self.end_point[0],self.end_point[1]):
                directEdge = False
            else:
                directEdge = True

          
            if self.check_collision(x1,y1,x,y):
                nodeEdge = False
            else:
                nodeEdge = True

        return (x,y,nodeEdge,directEdge)

    def draw_circle(self, x, y):
        self.img = cv2.circle(self.img, (x,y), 5 ,(0,0,255), thickness=3, lineType=8)

    def draw_line(self, x1,y1,x2,y2, thickness = 1):
        self.img = cv2.line(self.img, (x1,y1), (x2,y2), (0,255,0), thickness=thickness, lineType=8)



if __name__ == "__main__":


    img = cv2.imread("/home/siddy/Downloads/aruco-markers-examples.jpg")

    obstacle_list = [[1162.0, 212.0, 1538.0, 212.0, 1538.0, 589.0, 1162.0, 589.0], 
                    [240.0, 212.0, 616.0, 212.0, 616.0, 589.0, 240.0, 589.0]]

    start_point = (0,0)
    end_point = (700,436)
    

    step_size = 100
    iter_limit = 100

    rrt_planner = RRT(start_point,end_point, step_size ,iter_limit ,obstacle_list,img)
    print(rrt_planner.planning())

    cv2.imshow("Image",rrt_planner.img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()



    