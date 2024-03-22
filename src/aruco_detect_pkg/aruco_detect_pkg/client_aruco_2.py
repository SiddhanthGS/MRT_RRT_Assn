
import rclpy
from rclpy.node import Node
import cv_bridge
import cv2 
from tutorial_interfaces.srv import ArucoDetect,ArUcoPath
from tutorial_interfaces.msg import Float64,Float64MultiArray,Image
#from aruco_corners.srv import ArUcoPath
import argparse
import time
from std_msgs.msg import Int64MultiArray

class InfoClient(Node):

    def __init__(self):
        
        super().__init__('client')
        self.cli = self.create_client(ArucoDetect, 'identify_aruco')
 
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Info Server not available, waiting again...')
 
        self.req = ArucoDetect.Request()

        #Initalize the cv bridge.
        self.cv_bridge = cv_bridge.CvBridge()

         


    def send_request(self,source):

        ret, frame = source.read()

        
        if not ret:
            return None , None  

        self.req.image = self.get_image_msg(frame)
        self.future = self.cli.call_async(self.req)
        
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result() , frame
        

    def get_image_msg(self,img):
        imgmsg = self.cv_bridge.cv2_to_imgmsg(img)
        return imgmsg     


       
class RRTClient(Node):

    def __init__(self):
        
        super().__init__('client')
        self.cli = self.create_client(ArUcoPath, 'plan_path')
 
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('RRT Server not available, waiting again...')
 
        self.req =ArUcoPath.Request()

        #Initalize the cv bridge.
        self.cv_bridge = cv_bridge.CvBridge()


    def send_request(self,img,coordinates,info):
        
        self.req.image = self.get_image_msg(img)
        self.req.coordinates = self.get_array_msg(coordinates)
        self.req.info = self.get_array_msg(info)
        
        self.future = self.cli.call_async(self.req)
        
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
    def get_image_msg(self,img):

        imgmsg = self.cv_bridge.cv2_to_imgmsg(img)
        return imgmsg  

    def get_array_msg(self,msg):
        msg = list(map(int,msg))
        array_msg = Int64MultiArray(data = msg)
        return array_msg





def main():

    # Create Server Node...
    rclpy.init()
    info_client = InfoClient()


    # Parser to parse required things...
    # parser = argparse.ArgumentParser(description='Video')

    # # Add argument for end point
    # parser.add_argument('filename', type=str, help='Path to the file')


    # # Add arguments for the start point
    # parser.add_argument('start_x', type= int, help='x Cordinate of the start point')
    # parser.add_argument('start_y', type= int, help='y Cordinate of the start point')
    
    # # Add arguments for end point
    # parser.add_argument('end_x', type=int, help='x Cordinate of the end point')
    # parser.add_argument('end_y', type= int, help='y Cordinate of the end point')
    

    # # Parse the command-line arguments
    # args = parser.parse_args()

    # Open the video file
    try:
        cap = cv2.VideoCapture('/home/siddy/Videos/DSC_1798.MOV')
    except Exception:
        pass

    start_x=0
    start_y=0
    end_x=1920
    end_y=1080
    
    if not cap.isOpened():
        print("Error: Unable to open the video file.")
        source = cv2.VideoCapture(0)        # Get frames from webcam..
    else:
        source = cap             # Get frames from video..



    try:

        start_point = (start_x,start_y)          # These can be taken from user also..
        end_point = (end_x,end_y)
        plan_path = True
    
    except Exception:

        plan_path = False
        pass

    
    
    if plan_path :
        rrt_client = RRTClient()




    counter = 1
    while True:

        # Make request using the video as path given by user..
        response, image = info_client.send_request(source)

        if image is None:
            break

        print(f"Sending Frame : {counter}")
        counter +=1
        
        # Ids-list processing...
        ids_list = response.ids.tolist()
        
        # 2-d array processing... (Need to be improved)
        coordinates = response.coordinates.data
        num_rows = len(coordinates) // 8
        coordinates_array = [coordinates[i*8:(i+1)*8].tolist() for i in range(num_rows)]
    
        if ids_list == []:
            info_client.get_logger().info("No ArUco Found in this Frame!!")
        else:
            info_client.get_logger().info(f"ArUco Markers in this Frame have tag ids : {str(ids_list)} and the coordinates of the bounding boxes are: {str(coordinates_array)}")


        # Plann path..
        if plan_path:

            step_size = 100
            iter_limit = 1000

            rrt_response = rrt_client.send_request(image, coordinates, info = [start_point[0],start_point[1],end_point[0],end_point[1],step_size,iter_limit])
            path_points = rrt_response.coordinates.data
            if path_points:

                num_rows = len(path_points) // 2
                path_points = [tuple(path_points[i*2:(i+1)*2]) for i in range(num_rows)]
                print(f"The path for the given start and end point include points: {path_points}")
            
            else:
                print("Path not found :(")
            print("\n")
                
    
    if plan_path: rrt_client.destroy_node()
    info_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()