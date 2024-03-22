#!/bin/env python
import cv2 
import rclpy
import sys
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge,CvBridgeError
from tutorial_interfaces.srv import ArucoDetect
from tutorial_interfaces.srv import ArUcoPath
from std_msgs.msg import Int64MultiArray


#capture=cv2.VideoCapture(0)
#print(capture.isOpened())
#bridge=CvBridge()

class Image_Client(Node):
    def __init__(self):
        super().__init__("img_client_node")
        #self.cli=self.create_client(ArucoDetect,'aruco_detect')
        #while not self.cli.wait_for_service(timeout_sec=1.0):
         #   self.get_logger().info("service not availabe,continue waiting...")
        self.bridge=CvBridge()
        #self.req=ArucoDetect.Request()

        
        
       # self.req=Image.Request()
        #self.img=cv2.imread()

    def send_request(self):
        img='/home/siddy/Downloads/aruco-markers-examples.jpg'
        self.cli=self.create_client(ArucoDetect,'aruco_detect')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not availabe,continue waiting...")
        Bridge=CvBridge()
        ArgImg=cv2.imread(img)
        imgMsg=Bridge.cv2_to_imgmsg(ArgImg,encoding="bgr8")
        self.req=ArucoDetect.Request()
        self.req.image=imgMsg
        
        #capture=cv2.VideoCapture(0)
        #print(capture.isOpened())
        #bridge=CvBridge()
        self.future=self.cli.call_async(self.req)
        #while rclpy.ok():
         #   if self.future.done() and self.future.result(): 
        
        #        self.get_logger().info("send_request:exit")
         #       return self.future.result()
        #rclpy.spin_until_future_complete(self,self.future)
        return None
    def send_request_rrt(self,coordinates,info):
        self.cli=self.create_client(ArUcoPath,'rrt_path_form')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not availabe,continue waiting...")
        Bridge=CvBridge()
        self.req=ArUcoPath.Request()
        #coordinates=list(map(int,coordinates))
        #array_msg=Int64MultiArray(data=coordinates)
        self.req.coordinates=coordinates
        
        #info=list(map(int,info))
        #array_msg2=Int64MultiArray(data=info)
        self.req.info=info
        #self.req.start_pose=[10.0,10.0]
        #self.req.goal_pose=[600.0,400.0]
        img='/home/siddy/Downloads/aruco-markers-examples.jpg'
        self.req.image=img
        self.future=self.cli.call_async(self.req)
        return None
        

def main(args=None):
    
    rclpy.init(args=args)
    client_node=Image_Client()
    Bridge=CvBridge()
    client_node.send_request()
    while rclpy.ok():
        rclpy.spin_once(client_node)
        if client_node.future.done():
            try:
                response=client_node.future.result()
            except Exception as e:
                client_node.get_logger().info('service call failed'+e)
            else: 
                client_node.get_logger().info(str(response.id))#response.tl,response.tr,response.br,response.bl)
                client_node.get_logger().info(str(response.coordinates))        
        break
    boxes=response.coordinates
    #client_node.get_logger().info(response.id,response.corners)
    #rclpy.spin(client_node)
    client_node.destroy_node()

    client_node_rrt=Image_Client()
    img='/home/siddy/Downloads/Screenshot 2024-03-09 at 18-35-16 Online ArUco markers generator.png'
    info_msg=[img,0,0,400,400]
    client_node_rrt.send_request_rrt(boxes,info_msg)
    while rclpy.ok():
        rclpy.spin_once(client_node_rrt)
        if client_node_rrt.future.done():
            try:
                response=client_node_rrt.future.result()
            except Exception as e:
                client_node_rrt.get_logger().info('service call failed'+e)
            #else:
                #client_node_rrt.get_logger().info(str(response.))
    client_node_rrt.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
   main() 