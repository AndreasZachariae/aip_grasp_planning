#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Image

from point_transformation_interfaces.srv import PixelToPoint
from aip_grasp_planning_interfaces.srv import GraspObjectSurfaceNormal
from aip_grasp_planning_interfaces.srv import GraspPlanning
from aip_grasp_planning_interfaces.msg import CylinderCombination

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup


from aip_grasp_planning.cylinder_selection import CylinderSelection
import cv2
from cv_bridge import CvBridge


# import cv2
# from cv_bridge import CvBridge
# import numpy as np

class GraspPlanningNode(Node):
    """
    This class represents the GraspPlanning node.

    It subscribes to the '/stereo/depth' topic, calls the 'PixelToPoint' service provided by the 'point_transformation_node',
    and handles the response asynchronously.
    """
   # depth_image = request.depth_image

        # for mask in masks:
        # # Convert the mask image to a list of pixels
        #     pixels = self.convert_image_mask_to_pixel_indices(mask)

        #     # Call the 'pixel_to_point' method to convert the pixels to points
        #     points = self.pixel_to_point(pixels, depth_image.height, depth_image.width, depth_image)

        #     # Call the 'grasp_pose_client' service to get the grasp poses
        #     grasp_pose_request = GraspObjectSurfaceNormal.Request()
        #     grasp_pose_request.masked_points = points
        #     grasp_pose_response = self.grasp_pose_client.call(grasp_pose_request)

        #     # # Process the grasp pose response and extract the grasp poses
        #     # grasp_pose = grasp_pose_response.grasp_pose
    def __init__(self):
        super().__init__('grasp_planning_node')
        self.service_group = MutuallyExclusiveCallbackGroup() #callback group controls the execution of the callback function 
                                                                        # MutuallyExclusiveCallbackGroup: Only one Callback in the group can be executed at a time.
        self.get_logger().info("Started grasp_planning_server")

        # Create a server to handle the 'grasp_planning service requests
        self.grasp_planning_server = self.create_service(GraspPlanning, 'grasp_planning_node/grasp_planning', self.grasp_planning_logic)

        # Create a client to call the 'PixelToPoint' service provided by the 'point_transformation_node'
        self.PtP_client = self.create_client(PixelToPoint, 'point_transformation_node/pixel_to_point', callback_group=self.service_group)

        # Create a client to request the grasp pose normal vector
        self.grasp_pose_client = self.create_client(GraspObjectSurfaceNormal, 'grasp_object_surface_normal', callback_group=self.service_group)

        # Wait for the 'PixelToPoint' service to become available
        while not self.PtP_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('PixelToPoint service not available, waiting again...')

        # Wait for the 'GraspObjectSurfaceNormal' service to become available
        while not self.grasp_pose_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('GraspObjectSurfaceNormal service not available, waiting again...')


    def grasp_planning_logic(self, request, response): 
        
        self.get_logger().info('Received request for Grasp Planning')
    
        # Get all the masks from the odtf part of the request
        masks = []
        class_names = []
        for detection in request.detections.detections:
            masks.append(detection.mask)
            class_names.append(detection.class_name)

        # Get the reference image from the odtf part of the request        
        depth_image = request.depth_image        # sensor_msgs/Image reference_image as part of detection<detections<DetectObject.srv (-> devel Branch)

        # Get the package sequence from the request        
        packages = request.package_sequence.packages
        pack_sequence_class_names = []
        for package in packages:
            pack_sequence_class_names.append(package.class_name)

        grasp_poses = []
        for pack_sequence_class_name in pack_sequence_class_names:
            if pack_sequence_class_name not in class_names:
                self.get_logger().info(f"{pack_sequence_class_name} not found in the detected objects.")
                self.get_logger().info("Abort grasp planning for this pack sequence.")
                return

            mask_index = class_names.index(pack_sequence_class_name)
            mask = masks[mask_index]

            # Convert the mask image to a list of pixels
            pixels = self.convert_image_mask_to_pixel_indices(mask, depth_image.width, depth_image.height)

            # Call the 'pixel_to_point' method to convert the pixels to points
            points = self.pixel_to_point(pixels, depth_image.height, depth_image.width, depth_image)

            self.get_logger().info("Converted masks to pixel indices and to points.")

            # Call the 'grasp_pose_client' servicefilterCloud to get the grasp poses
            grasp_pose_request = GraspObjectSurfaceNormal.Request()
            grasp_pose_request.masked_points = points       #request type = geometry_msgs/Point[]
            future = self.grasp_pose_client.call_async(grasp_pose_request)  #response type = geometry_msgs/Pose surface_normal_to_grasp
            rclpy.spin_until_future_complete(self, future)
            grasp_pose_response = future.result()
            self.get_logger().info("Received grasp pose from point cloud processing node.")
            self.get_logger().info("Grasp Pose: " + str(grasp_pose_response.surface_normal_to_grasp))

            grasp_poses.append(grasp_pose_response.surface_normal_to_grasp)

        ### Cylinder Selection ###

        self.get_logger().info("Extracting package information for cylinder selection.")
        
        # Extract the weight of the packages
        packages_weights = []
        for package in packages:
            packages_weights.append(package.weight)

        # Extract the dimensions of the packages
        #packages_dimensions = request.package_sequence.packages.dimensions       #geometry_msgs/Vector3 dimensions
        packages_length = []
        for package in packages:
            packages_length.append(package.dimensions.x)
        
        packages_width = []
        for package in packages:
            packages_width.append(package.dimensions.y)
        
        
        # Choose Cylinder per package based on the package dimensions and weight
        index_msgs, cylinder_ids_per_package, tcps_cylinder_offsets = CylinderSelection().choose_cylinder(packages_weights, packages_length, packages_width) #, packages_height)
        self.get_logger().info("Cylinder IDs per package: " + str(cylinder_ids_per_package))
        self.get_logger().info("TCP offsets per package: " + str(tcps_cylinder_offsets))

        response.cylinder_ids = cylinder_ids_per_package

        # Fixed response for now
        cylinder_combination = CylinderCombination()
        cylinder_combination.cylinder_ids = [1, 2]
        response.cylinder_ids = [cylinder_combination]


        ## Grasp Pose with added TCP offsets ###
        # new_grasp_pose = grasp_pose_response * tcp_offset
        # self.get_logger().info("New Grasp Pose including offsets: " + str(new_grasp_pose))

        # Fixed cylinder offset index for now         
        tcp_offset = tcps_cylinder_offsets[0]
        self.get_logger().info("TCP Offset with Index 0: " + str(tcp_offset))

        # For now: Mock up with fixed grasp poses
        # Return the response with the calculated grasp poses
        response.grasp_pose = grasp_poses


        ### Place Pose Definition ###
        # ToDo: Add Logic to decide which place poses to use
        
        # Container corner reference 
        container_corner = Point(x=0.75, y=0.1, z=0.95)   #container size = 0.585, 0.392, 0.188 -> see packing_algorithm docker container 

        # Retrieve target orientation and place coordinates 
        packages_target_orientations = [] #uint32 rotation_index from Package.msg
        for package in packages:
            packages_target_orientations.append(package.rotation_index) #possible values: 10, 30

        # package_target_place_coordinates = []
        # for package in package_sequence.packages:
        #     package_target_place_coordinates.append(package.place_coordinates)

        # # Retrieve package height // length + width + weight are already extracted above
        # packages_height = []
        # for package in package_sequence.packages:
        #     packages_height.append(package.dimensions.z)


        # Calculate the place pose based on the target orientation and place coordinates
        place_poses = []
        for package in packages:
            self.get_logger().info("Package: " + str(package))
            # orientation_pick =                            #match coresponding orientation from the detection with correct index
            # orientation_pack = package.rotation_index
            place_pose = Pose(
                            position=Point(x=container_corner.x + package.place_coordinates.x, 
                                           y=container_corner.y - package.place_coordinates.y, 
                                           z=container_corner.z + package.dimensions.z), 
                            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
            place_poses.append(place_pose)

        response.place_pose = place_poses

        # # FIXED Place Pose response for now
        # response.place_pose = [
        #     Pose(position=Point(x=1.05, y=0.46, z=1.55), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)),
        #     Pose(position=Point(x=0.65, y=0.55, z=1.60), orientation=Quaternion(x=0.0, y=-0.05, z=0.05, w=1.0)),
        # ] #             Pose(position=Point(x=0.65, y=0.55, z=1.60), orientation=Quaternion(x=0.0, y=-0.05, z=0.05, w=1.0)),

        return response

    def convert_image_mask_to_pixel_indices(self, mask, depth_width, depth_height):
        pixels = []
        bridge = CvBridge()
        self.get_logger().info(f"Mask shape: {mask.height} x {mask.width}")
        # Konvertieren von sensor_msgs/Image zu einem OpenCV-Bild
        cv_image = bridge.imgmsg_to_cv2(mask, desired_encoding='passthrough')
        # Konvertieren des OpenCV-Bildes in ein Numpy-Array
        self.get_logger().info(f"Mask shape: {cv_image.shape}")
        resized_image = cv2.resize(cv_image, (depth_height, depth_width))

        for i in range(depth_height):
            for j in range(depth_width):
                if resized_image[j, i] == 1:
                    pixels.append((j, i))
        return pixels
        

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
            self.get_logger().debug('Received response for PtP: %s' % str(future.result().points))
        else:
            self.get_logger().error('Exception while calling PtP service: %r' % future.exception())
        self.get_logger().info(str(ptp_response.points))
        return ptp_response.points
    

# Choose the mask with the highest probability

    # def highest_probability_mask(self, odtf_response):
    #     # Select the mask with the highest probability     
    #     max_prob = -1
    #     best_detection = None
        
    #     for detection in odtf_response.detections.detections:
    #         if detection.probability > max_prob:
    #             max_prob = detection.probability
    #             best_detection = detection
    #         if best_detection is not None:
    #             self.get_logger().info('Detection with highest probability: %s' % str(best_detection.mask))
    #         else:
    #             self.get_logger().info('No detections found')
   
    #     ## To-Do: Position the mask correctly in the depth image frame based on the bounding box and center point of the bounding box ##

    #     width_bb = int(best_detection.mask.width)
    #     height_bb = int(best_detection.mask.height)

    #     x_offset = int(best_detection.center.x - width_bb/2)
    #     y_offset = int(best_detection.center.y + height_bb/2)     #sensor_msgs/Image has the origin at the top left corner  

    #     best_mask = [(i, j) for i in range(x_offset, x_offset + width_bb) for j in range(y_offset - height_bb, y_offset)]  
       
    #     return best_mask

    ########################

    # def retrieve_mask_async(self):

    #     mask_request = DetectObjects.Request()
    #     # Call the 'detect_objects' service asynchronously to retrieve the mask
    #     mask_response: DetectObjects.Response = self.ODTF_client.call_async(mask_request)

    #     return mask_response

    # def retrieve_mask(self):

    #     future = self.retrieve_mask_async()
    #     self.get_logger().info('before spin_until_future_complete for Mask')

    #     rclpy.spin_until_future_complete(self, future)
    #     odtf_response = future.result()
        
    #     self.get_logger().info('after spin_until_future_complete for Mask')

    #     # Log the masks received from the 'detect_objects' service
    #     if odtf_response is not None:
    #         for i, detection in enumerate(odtf_response.detections.detections):
    #             self.get_logger().info('Received response for Mask %d: %s' % (i, str(detection.mask))) 
    #     else:
    #         self.get_logger().error('Exception while calling ODTF service: %r' % future.exception())
    
    #     for i, detection in enumerate(odtf_response.detections.detections):
    #         self.get_logger().info('Mask %d: %s' % (i, str(detection.mask)))
        
    #     if odtf_response is not None:
    #         next_object_mask = self.highest_probability_mask(odtf_response)
    #     else:
    #         self.get_logger().error('Exception while calling ODTF service: %r' % future.exception())   
        
    #     return next_object_mask

    #########################

#     def next_object_mask(self, odtf_response):
        
#   ##### To-Do: Select the correct mask based on the packing sequence from packing algorithm ##
#             # Subscribing to the 'packing_sequence' topic to get the packing sequence or client call to the packing algorithm
        
#         next_sequence_object= None

#         return next_sequence_object.mask
    
    #########################


def main(args=None):
    rclpy.init(args=args)

    grasp_planning_node = GraspPlanningNode()

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