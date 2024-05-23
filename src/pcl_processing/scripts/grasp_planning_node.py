#!/usr/bin/env python3

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image, RegionOfInterest

from object_detector_tensorflow_interfaces.srv import DetectObjects

from geometry_msgs.msg import Point
#from sensor_msgs.msg import PointCloud2
from point_transformation_interfaces.srv import PixelToPoint

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

class GraspPlanning(Node):
    """
    This class represents the GraspPlanning node.

    It subscribes to the '/stereo/depth' topic, calls the 'PixelToPoint' service provided by the 'point_transformation_node',
    and handles the response asynchronously.
    """

    def __init__(self):
        super().__init__('depth_image_sub')
        self.transform_service_group = MutuallyExclusiveCallbackGroup() #callback group controls the execution of the callback function 
                                                                        # MutuallyExclusiveCallbackGroup: Only one Callback in the group can be executed at a time.

        # Create a subscription to the '/stereo/depth' topic and specify the callback function
        self.image_subscriber = self.create_subscription(Image, '/stereo/depth', self.depth_image_callback, 10)
        self.get_logger().info("Started depth_image_sub")



        ## TO-DO ##
        # Create a client to call 'detect_object' service to retrieve the mask provided by ODTF
        self.ODTF_client = self.create_client(DetectObjects, 'detection_node/detect_objects', callback_group=self.transform_service_group)


        # Create a client to call the 'PixelToPoint' service provided by the 'point_transformation_node'
        self.PtP_client = self.create_client(
        PixelToPoint, 'point_transformation_node/pixel_to_point', callback_group=self.transform_service_group)

        # Wait for the 'PixelToPoint' service to become available
        while not self.PtP_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('PixelToPoint service not available, waiting again...')

        # Create a publisher to '/../points topic and 
        self.points_publisher = self.create_publisher(PointCloud2, 'points_topic', 10)

        

    def pixel_to_point_async(self, pixels: list, height=0, width=0, depth_image:Image=Image()):
        """
        Convert a list of pixels to a list of Point objects asynchronously.

        Args:
            pixels (list): List of pixels to convert.
            height (int): Height of the image.
            width (int): Width of the image.
            depth_image (Image): Depth image.

        Returns:
            transform_response (PixelToPoint.Response): Response from the 'PixelToPoint' service.
        """
        # Convert the list of pixels to a list of Point objects
        points_msg = []
        for pixel in pixels:
            point = Point()
            point.x = float(pixel[0])
            point.y = float(pixel[1])
            point.z = 0.0  # Set the z-coordinate to 0 for now
            points_msg.append(point)

        # Create a request message for the 'PixelToPoint' service
        tranform_request = PixelToPoint.Request()
        tranform_request.pixels = points_msg
        tranform_request.height = int(height)
        tranform_request.width = int(width)
        tranform_request.depth_image = depth_image
        tranform_request.camera_type = "roboception"

        # Call the 'PixelToPoint' service asynchronously
        transform_response: PixelToPoint.Response = self.PtP_client.call_async(tranform_request)
       
        return transform_response

    def pixel_to_point(self, pixels: list, height=0, width=0, depth_image:Image=Image()):
        """
        Convert a list of pixels to a list of Point objects.

        Args:
            pixels (list): List of pixels to convert.
            height (int): Height of the image.
            width (int): Width of the image.
            depth_image (Image): Depth image.

        Returns:
            response (PixelToPoint.Response): Response from the 'PixelToPoint' service.
        """
        # Call the 'pixel_to_point_async' method to make an asynchronous call to the 'PixelToPoint' service
        future = self.pixel_to_point_async(pixels, height, width, depth_image)
        self.get_logger().info('before spin_until_future_complete for PixelToPoint')

        # Wait until the future is complete and get the response
        rclpy.spin_until_future_complete(self, future)
        ptp_response = future.result()
        if ptp_response is not None:
            self.get_logger().info('Received response for PtP: %s' % str(future.result().points))
        else:
            self.get_logger().error('Exception while calling PtP service: %r' % future.exception())
        self.get_logger().info(str(ptp_response.points))
        return ptp_response.points
    

    def highest_probability_mask(self, odtf_response):
        # Select the mask with the highest probability     
        max_prob = -1
        best_detection = None
        
        for detection in odtf_response.detections.detections:
            if detection.probability > max_prob:
                max_prob = detection.probability
                best_detection = detection
            if best_detection is not None:
                self.get_logger().info('Detection with highest probability: %s' % str(best_detection.mask))
            else:
                self.get_logger().info('No detections found')
   
        ## To-Do: Position the mask correctly in the depth image frame based on the bounding box and center point of the bounding box ##

        width_bb = best_detection.mask.width
        height_bb = best_detection.mask.height

        x_offset = best_detection.center.x - width_bb/2
        y_offset = best_detection.center.y + height_bb/2     #sensor_msgs/Image has the origin at the top left corner

        best_mask = [(i, j) for i in range(x_offset, x_offset + width_bb) for j in range(y_offset - height_bb, y_offset)]  
       
        return best_mask

    ########################

    def retrieve_mask_async(self):

        mask_request = DetectObjects.Request()
        # Call the 'detect_objects' service asynchronously to retrieve the mask
        mask_response: DetectObjects.Response = self.ODTF_client.call_async(mask_request)

        return mask_response

    def retrieve_mask(self):

        future = self.retrieve_mask_async()
        self.get_logger().info('before spin_until_future_complete for Mask')

        rclpy.spin_until_future_complete(self, future)
        odtf_response = future.result()
        
        self.get_logger().info('after spin_until_future_complete for Mask')

        # Log the masks received from the 'detect_objects' service
        if odtf_response is not None:
            for i, detection in enumerate(odtf_response.detections.detections):
                self.get_logger().info('Received response for Mask %d: %s' % (i, str(detection.mask))) 
        else:
            self.get_logger().error('Exception while calling ODTF service: %r' % future.exception())
    
        for i, detection in enumerate(odtf_response.detections.detections):
            self.get_logger().info('Mask %d: %s' % (i, str(detection.mask)))
        
        if odtf_response is not None:
            next_object_mask = self.highest_probability_mask(odtf_response)
        else:
            self.get_logger().error('Exception while calling ODTF service: %r' % future.exception())   
        
        return next_object_mask

    #########################

#     def next_object_mask(self, odtf_response):
        
#   ##### To-Do: Select the correct mask based on the packing sequence from packing algorithm ##
#             # Subscribing to the 'packing_sequence' topic to get the packing sequence or client call to the packing algorithm
        
#         next_sequence_object= None

#         return next_sequence_object.mask
    
    #########################


    def depth_image_callback(self, img_msg):
        
        self.img_msg = img_msg

        # Call the 'pixel_to_point' method to convert the pixels to points
        #self.point = self.pixel_to_point([(5,10), (100,100)], img_msg.height, img_msg.width, img_msg)
        
        
        #### Temporary mask for testing
        # self.mask = [(i, j) for i in range(100, 105) for j in range(100, 105)]
        #self.mask = [[(5,10), (100,100)]]
        
        # Get the mask from the 'detect_objects' service response
        self.mask = self.retrieve_mask()
        self.get_logger().info("after ODTF Mask client call")

        # Call the 'pixel_to_point' method with the retrieved mask to convert the pixels to points
        self.point = self.pixel_to_point(self.mask, img_msg.height, img_msg.width, img_msg)
        self.get_logger().info("after PixelToPoint client call")


        ## To-Do: Create Publisher to /points 

        #points_msg = PointCloud2
        
        
        return


def main(args=None):
    rclpy.init(args=args)

    grasp_planning_node = GraspPlanning()

    try:
        rclpy.spin(grasp_planning_node)
        grasp_planning_node.get_logger().info("Started Node")

    except KeyboardInterrupt:
        grasp_planning_node.get_logger().info("Stopped Node")

    #####################

    grasp_planning_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()