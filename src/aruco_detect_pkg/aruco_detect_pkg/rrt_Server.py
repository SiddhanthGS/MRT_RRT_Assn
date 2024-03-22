import rclpy
from rclpy.node import Node
from tutorial_interfaces.srv import ArUcoPath
from .rrt import RRT
import random
import numpy as np
import cv_bridge
import math
import cv2

# class RRT:
#     def __init__(self,width,height,obstacles):
#         self.width=width
#         self.height=height
#         self.obstacles=obstacles
#         self.tree=[]

#     def random_point(self):
#         return (random.uniform(0,self.width),random.uniform(0,self.height))
    
#     def nearest_neighbour(self,point):
#         nearest_dist=float('inf')
#         nearest_point=None

#         for node in self.tree:
#             dist=np.linalg.norm(np.array(point)-np.array(node))
#             if dist<nearest_dist:
#                 nearest_dist=dist
#                 nearest_point=node
#         return nearest_point
    
#     def steer(self,from_point,to_point,max_distance):
#         diff=np.array(to_point)-np.array(from_point)
#         diff_norm=np.linalg.norm(diff)
#         if diff_norm<max_distance:
#             return to_point
#         else:
#             return [from_point + max_distance*(diff/diff_norm)]
        
#     def is_collision_free(self,from_point,to_point):
#         for obstacle in self.obstacles:
#             if self.intersects(from_point,to_point,obstacle):
#                 return False
        
#         return True
    
#     def intersects(self,point1,point2,obstacle):
#         x1,y1=obstacle[0],obstacle[1]
#         x2,y2=obstacle[2],obstacle[3]
#         x,y=point1
#         u,v=point2
#         return x1<=x<=x2 and y1<=y<=y2 or x1<=u<=x2 and y1<=v<=y2 or \
#                (x<=x1 and u>=x2 and y<=y1 and v>=y2) or (x<=x1 and u>=x2 and y<=y2 and v>=y1) or \
#                (x<=x1 and u>=x2 and y<=y1 and v>=y1) or (x<=x1 and u>=x1 and y<=y2 and v>=y2) or \
#                (x<=x2 and u>=x2 and y<=y2 and v>=y1) or (x<=x2 and u>=x2 and y<=y1 and v>=y2)
    
#     def compute_rrt(self,start_pose,goal_pose,max_iter=1000,max_distance=10):
#         self.tree[start_pose]=None

#         for i in range(max_iter):
#             random_point=self.random_point()
#             nearest_point=self.nearest_neighbour(random_point)
#             new_point=self.steer(nearest_point,random_point,max_distance)
#             if self.is_collision_free(nearest_point,new_point):
#                 self.tree[new_point]=nearest_point
#                 if np.linalg.norm(np.array(new_point)-np.array(goal_pose))<max_distance:
#                     self.tree[goal_pose]=new_point
#                     break
    
#         else:
#             raise Exception("rrt could not find a path")
        
#         path=[]
#         current_point =goal_pose
#         while current_point is not None:
#             path.append(current_point)
#             current_point=self.tree[current_point]

#         path.reverse()
#         return path
class Nodes:
    def __init__(self,x,y):
        self.x=x
        self.y=y
        self.parent_x=[]
        self.parent_y=[]  
class RRT:
    def __init__(self,start_point=(0,0),end_point=(500,500),stepsize=5,iterlimit=1000):
        self.nodeslist=[]
        self.start_point=start_point
        self.end_point=end_point
        self.stepsize=stepsize
        self.iterlimit=iterlimit
        self.nodeslist.append(Nodes(start_point[0],start_point[1]))
        self.nodeslist[0].parent_x.append(start_point[0])
        self.nodeslist[0].parent_y.append(start_point[1])
          
    def planning(self):

        if(self.end_point[0],self.end_point[1]) in self.obstacle_points or(self.end_point[1]<0 or self.end_point[0]<0):
            print('path unavailable')
            return[]
        self.draw_circle()

        dirConnection=self.check_collision(self.start_point[0],self.start_point[1],self.end_point[0],self.end_point[1])
        if not dirConnection:
            self.draw_line



class RRTServer(Node):
    def __init__(self):
        super().__init__('rrt_server')
        self.srv=self.create_service(ArUcoPath,'rrt_path_form',self.aruco_path_callback)
        self.cv_bridge=cv_bridge.CvBridge()
        self.counter=1
        self.img=None


    def aruco_path_callback(self,request,response):
        self.get_logger().info(f"reveiving image: {self.counter}")
        self.counter +=1
        aruco_corners=request.aruco_corners
        start_pose=request.start_pose
        goal_pose=request.goal_pose
        image_width=1920
        image_height=1080
    
      
        rrt_planner=RRT(image_width,image_height,aruco_corners)
        rrt_path=rrt_planner.compute_rrt(start_pose,goal_pose)

        response.rrt_path=rrt_path
        self.get_logger().info('RRT path computed and sent')
        return response


def main(args=None):
    rclpy.init(args=args)
    rrt_server=RRTServer()
    rclpy.spin(rrt_server)
    rrt_server.destroy_node()
    rclpy.shutdown()
    
if __name__=='__main__':
    main()    