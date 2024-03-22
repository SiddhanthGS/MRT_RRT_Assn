from tutorial_interfaces.srv import ArUcoPath
import rclpy
from rclpy.node import Node
import cv_bridge
import cv2 
import numpy as np
from std_msgs.msg import Int64MultiArray
import threading

# from rrt import RRT      # For development only
from .rrt import RRT          # Uncomment for package



class RRTServer(Node):

    def __init__(self):
        super().__init__('RRTServer')
        self.srv = self.create_service(ArUcoPath, 'plan_path', self.rrt_path_callback)

        #Initalize the cv bridge.
        self.cv_bridge = cv_bridge.CvBridge()

        self.counter = 1
        # self.rrt_planrer = RRT()

        # Image attribute..
        self.img = None

        # Start a thread for updating the image window..
        display_thread = threading.Thread(target=self.update_window)
        display_thread.start()


    def rrt_path_callback(self, request, response):

        # Log the receieved image counter..
        self.get_logger().info(f"Receiving Image:  {self.counter}")
        self.counter += 1

        img = self.cv_bridge.imgmsg_to_cv2(request.image)
        coordinates = request.coordinates.data
        info = request.info.data

        num_rows = len(coordinates) // 8
        coordinates= [coordinates[i*8:(i+1)*8].tolist() for i in range(num_rows)]

        
        rrt_planner = RRT(start_point = (info[0],info[1]), end_point = (info[2],info[3]), stepsize = info[4], iterlimit = info[5], bounding_boxes = coordinates, img = img)
        path_points = rrt_planner.planning()
        
        if  path_points:
            response.coordinates = self.get_array_msg([coordinate for point in path_points for coordinate in point ])

        else:
            response.coordinates = self.get_array_msg([])

        self.display_image(rrt_planner.img)

        # print(corners)
        return response
    
    def display_image(self,img):

        self.img = img

    def update_window(self):
        
        while True:

            if self.img is not None:
                cv2.imshow('Path', self.img)
                cv2.waitKey(1)
            


    def get_array_msg(self,msg):
        msg = list(map(int,msg))
        array_msg = Int64MultiArray(data = msg)
        return array_msg

        
def main():
    rclpy.init()

    rrt_server = RRTServer()  
    rclpy.spin(rrt_server)

    rclpy.shutdown()



if __name__ == '__main__':
    main()