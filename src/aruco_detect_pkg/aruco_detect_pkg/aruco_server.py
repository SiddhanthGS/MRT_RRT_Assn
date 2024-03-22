from tutorial_interfaces.srv import ArucoDetect
import rclpy
from rclpy.node import Node
import cv_bridge
import cv2 
import numpy as np
from std_msgs.msg import MultiArrayDimension, Float64MultiArray

class InfoServer(Node):

    def __init__(self):
        super().__init__('InfoServer')
        self.srv = self.create_service(ArucoDetect, 'identify_aruco', self.identify_aruco_callback)

        #Initalize the cv bridge.
        self.cv_bridge = cv_bridge.CvBridge()

        self.counter = 1
        self.arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        self.arucoParams = cv2.aruco.DetectorParameters()


    def identify_aruco_callback(self, request, response):

        # Log the receieved image counter..
        self.get_logger().info(f"Receiveing Image:  {self.counter}")
        self.counter += 1

        img = self.cv_bridge.imgmsg_to_cv2(request.image)
        ids,corners = self.identify_aruco(img)

        if ids is None:
            
            response.ids = []
            response.coordinates = Float64MultiArray()
            corners_data = []
        else :
            response.ids = np.array(ids).flatten().tolist()
            
            response.coordinates, corners_data = self.response_coordinates(corners)
            # _, corners_data = self.response_coordinates(corners)

            # corners = [[1,2,3,4],
            #            [4,5,6,7]]

            # response.coordinates = Float64MultiArray()
            # response.coordinates.data = corners

        self.display_image(img,corners_data, response.ids)


        # print(corners)
        return response
    

    def display_image(self,img, corners_data=None, ids = None):
        
        # Make a green box around identified ArUco-markers and mention its ID in Red Color..
        corners_data = list(map(int, corners_data))
        num_rows = len(corners_data) // 8
        corners_data = [corners_data[i*8:(i+1)*8] for i in range(num_rows)]

        for i in range(len(corners_data)):
            row = corners_data[i]
            img = cv2.rectangle(img, (row[0],row[1]), (row[4],row[5]), (0,255,0), 2) 
            img = cv2.putText(img, f"ID : {ids[i]}", (int((row[0]+row[4])/2),int((row[1]+row[5])/2)), cv2.FONT_HERSHEY_DUPLEX,  
                        1, (0,0,255), 2, cv2.LINE_AA) 


        cv2.imshow('Obstacles', img)
        cv2.waitKey(1)
        
    
    def identify_aruco(self,image):
        detector=cv2.aruco.ArucoDetector(self.arucoDict,self.arucoParams)
        (corners, ids, rejected) = detector.detectMarkers(image)
        return ids,corners
    
    def response_coordinates(self,corners):     # Improve this function..

        # 2-D array task is not completed yet!!!
        
        # Assign data to the float64 2D array
        coordinates_array = Float64MultiArray()
        corners_data = np.array(corners).flatten().tolist() 

        coordinates_array.layout.dim = [MultiArrayDimension(), MultiArrayDimension()]
        # coordinates_array.layout.dim[0].label = "rows"
        # coordinates_array.layout.dim[0].size = 3  # Number of rows
        coordinates_array.layout.dim[1].label = "columns"
        coordinates_array.layout.dim[1].size = 8  # Number of columns
        coordinates_array.data =  corners_data # Example data
        return coordinates_array , corners_data


def main():
    rclpy.init()

    info_server = InfoServer()  
    rclpy.spin(info_server)

    rclpy.shutdown()



if __name__ == '__main__':
    main()