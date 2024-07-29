#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import rclpy.time
import numpy as np
from sensor_msgs.msg import Image

from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Image
from aip_grasp_planning.utils_transform import Affine


from point_transformation_interfaces.srv import PixelToPoint
from aip_grasp_planning_interfaces.srv import GraspObjectSurfaceNormal
from aip_grasp_planning_interfaces.srv import GraspPlanning
from aip_grasp_planning_interfaces.msg import CylinderCombination
from geometry_msgs.msg import PoseArray


from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from tf2_ros import Buffer, TransformListener

from tf2_geometry_msgs import do_transform_pose

from aip_grasp_planning.cylinder_selection import CylinderSelection
import cv2
from cv_bridge import CvBridge

from scipy.spatial.transform import Rotation


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
        self.ptp_group = MutuallyExclusiveCallbackGroup()
        self.get_logger().info("Started grasp_planning_server")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # Create a server to handle the 'grasp_planning service requests
        self.grasp_planning_server = self.create_service(GraspPlanning, 'grasp_planning_node/grasp_planning', self.grasp_planning_logic)
        self.grasp_poses_publisher = self.create_publisher(PoseArray, '/grasp_poses', 10)
        # Create a client to call the 'PixelToPoint' service provided by the 'point_transformation_node'
        self.PtP_client = self.create_client(PixelToPoint, 'point_transformation_node/pixel_to_point', callback_group=self.ptp_group)

        # Create a client to request the grasp pose normal vector
        self.grasp_pose_client = self.create_client(GraspObjectSurfaceNormal, 'grasp_object_surface_normal', callback_group=self.service_group)

        # Wait for the 'PixelToPoint' service to become available
        while not self.PtP_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('PixelToPoint service not available, waiting again...')

        # Wait for the 'GraspObjectSurfaceNormal' service to become available
        while not self.grasp_pose_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('GraspObjectSurfaceNormal service not available, waiting again...')


    async def grasp_planning_logic(self, request, response): 
        
        self.get_logger().info('Received request for Grasp Planning')
    
        # Get all the masks from the odtf part of the request
        masks = []
        class_names = []
        orientations = []
        for detection in request.detections.detections:
            masks.append(detection.mask)
            class_names.append(detection.class_name)
            orientations.append(detection.orientation)

        # Get the reference image from the odtf part of the request        
        depth_image = request.depth_image        # sensor_msgs/Image reference_image as part of detection<detections<DetectObject.srv

        # Get the package sequence from the request        
        packages = request.package_sequence.packages


        ### Cylinder Selection + TCP Offset ###
        self.get_logger().info("Extracting package information for cylinder selection and TCP offsets.")
        
        # Extract the weight of the packages
        packages_weights = []
        for package in packages:
            packages_weights.append(package.weight)

        # Extract the dimensions of the packages
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


        pack_sequence_class_names = []
        for package in packages:
            pack_sequence_class_names.append(package.class_name)

        grasp_poses = []
        cylinder_ejection_offset = 0.15

        for idx, pack_sequence_class_name in enumerate(pack_sequence_class_names):
            # if pack_sequence_class_name not in class_names:
            #     self.get_logger().info(f"{pack_sequence_class_name} not found in the detected objects.")
            #     self.get_logger().info("Abort grasp planning for this pack sequence.")
            #     return

            mask_index = class_names.index(pack_sequence_class_name)
            mask = masks[mask_index]

            # Convert the mask image to a list of pixels
            pixels = self.convert_image_mask_to_pixel_indices(mask, depth_image.width, depth_image.height)

            # Call the 'pixel_to_point' method to convert the pixels to points
            future = self.pixel_to_point_async(pixels, depth_image.height, depth_image.width, depth_image)

            # Wait for the 'PixelToPoint' service to return the response
            await future
            point_response = future.result()
            

            self.get_logger().info("Converted masks to pixel indices and to points.")

            # Call the 'grasp_pose_client' servicefilterCloud to get the grasp poses
            grasp_pose_request = GraspObjectSurfaceNormal.Request()
            
            grasp_pose_request.masked_points = point_response.points       #request type = geometry_msgs/Point[]
            future = self.grasp_pose_client.call_async(grasp_pose_request)  #response type = geometry_msgs/Pose surface_normal_to_grasp
            
            await future
            grasp_pose_response = future.result()
            self.get_logger().info("Received grasp pose from point cloud processing node.")
            self.get_logger().info("Grasp Pose: " + str(grasp_pose_response.surface_normal_to_grasp))
            t = self.tf_buffer.lookup_transform("base_link", "camera", rclpy.time.Time())
            # tt = self.tf_buffer.lookup_transform("camera", "base_link", rclpy.time.Time())
            self.get_logger().info(f"Transform: {t.transform}")
            t_w = self.tf_buffer.lookup_transform("world", "base_link", rclpy.time.Time())
            orientation = orientations[mask_index]
            grasp_pose = Pose()
            grasp_pose.position.x = grasp_pose_response.surface_normal_to_grasp.position.x
            grasp_pose.position.y = grasp_pose_response.surface_normal_to_grasp.position.y
            grasp_pose.position.z = grasp_pose_response.surface_normal_to_grasp.position.z
            grasp_pose.orientation.x = orientation.x        
            grasp_pose.orientation.y = orientation.y
            grasp_pose.orientation.z = orientation.z
            grasp_pose.orientation.w = orientation.w
            self.get_logger().info(f"Grasp pose before transform: {grasp_pose}")

            grasp_pose = do_transform_pose(grasp_pose, t)
            self.get_logger().info(f"Grasp pose after transform base: {grasp_pose}")
            grasp_pose = do_transform_pose(grasp_pose, t_w)
            #turn by 90 to match y
            angle = np.arccos(grasp_pose.orientation.w) + np.pi/4
            grasp_pose.orientation.x = (grasp_pose.orientation.x / np.arccos(grasp_pose.orientation.w)) * np.sin(angle)
            grasp_pose.orientation.y = (grasp_pose.orientation.y / np.arccos(grasp_pose.orientation.w)) * np.sin(angle)
            grasp_pose.orientation.z = (grasp_pose.orientation.z / np.arccos(grasp_pose.orientation.w)) * np.sin(angle)
            grasp_pose.orientation.w = np.cos(angle)
            self.get_logger().info(f"Grasp pose after transform world WITHOUT TCP Offset: {grasp_pose}")

            # TODO for calculating the grasp pose with true surface normal vector
            # # Define a default homogeneous matrix
            # default_pose = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
            # # Convert the grasp pose to a homogeneous matrix
            # grasp_pose_matrix = default_pose * Affine()

            self.get_logger().info("TCP Offset with Index 0: " + str(tcps_cylinder_offsets[idx]))

            self.get_logger().info(f"TCP Offset with Index 0: {tcps_cylinder_offsets[idx].translation[0]}")
            self.get_logger().info(f"TCP Offset with Index 1: {tcps_cylinder_offsets[idx].translation[1]}")
            self.get_logger().info(f"TCP Offset with Index 2: {tcps_cylinder_offsets[idx].translation[2]}")

            grasp_pose.position.x = grasp_pose.position.x + tcps_cylinder_offsets[idx].translation[0]
            grasp_pose.position.y = grasp_pose.position.y + tcps_cylinder_offsets[idx].translation[1]
            grasp_pose.position.z = grasp_pose.position.z + tcps_cylinder_offsets[idx].translation[2] + cylinder_ejection_offset
            grasp_pose.orientation = grasp_pose.orientation

            self.get_logger().info("Grasp pose in WORLD WITH TCP Offset: " + str(grasp_pose))


            grasp_poses.append(grasp_pose)

        offset_pose_array = PoseArray()
        offset_pose_array.header.frame_id = "world"
        offset_pose_array.header.stamp = self.get_clock().now().to_msg()
        offset_pose_array.poses = grasp_poses
        self.grasp_poses_publisher.publish(offset_pose_array)        
            
        response.grasp_pose = grasp_poses



        ### PLACE Pose Definition ###
        # Container corner reference 
        container_corner = Point(x=0.75, y=0.1, z=0.95)   #container size = 0.585, 0.392, 0.188 -> see packing_algorithm docker container 
        place_poses = []

        # Calculate the place pose for each package
        for idx, package in enumerate(packages):
            self.get_logger().info("Package: " + str(package))

            # package.rotation_index = 3  #For TESTING

            # Check target orientation per package from the packing plan and calculate the place pose
            if package.rotation_index == 1: #long tcp side across to long package side 
                place_pose = Pose(
                            position=Point(x=container_corner.x + package.place_coordinates.x + tcps_cylinder_offsets[idx].translation[0], 
                                           y=container_corner.y - package.place_coordinates.y + tcps_cylinder_offsets[idx].translation[1], 
                                           z=container_corner.z + package.dimensions.z + tcps_cylinder_offsets[idx].translation[2] + cylinder_ejection_offset), 
                            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
                
            elif package.rotation_index == 3: #long tcp side parallel to long package side
                place_pose = Pose(
                            position=Point(x=container_corner.x + package.place_coordinates.x + tcps_cylinder_offsets[idx].translation[0], 
                                           y=container_corner.y - package.place_coordinates.y + tcps_cylinder_offsets[idx].translation[1], 
                                           z=container_corner.z + package.dimensions.z + tcps_cylinder_offsets[idx].translation[2] + cylinder_ejection_offset),
                            orientation=Quaternion(x=0.0, y=0.0, z=-np.sin(angle / 2), w=np.cos(angle / 2))
                        )

            place_poses.append(place_pose)

        response.place_pose = place_poses

        # # FIXED Place Pose response for now
        # response.place_pose = [
        #     Pose(position=Point(x=1.05, y=0.46, z=1.55), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)),
        #     Pose(position=Point(x=0.65, y=0.55, z=1.60), orientation=Quaternion(x=0.0, y=-0.05, z=0.05, w=1.0)),
        # ] #             Pose(position=Point(x=0.65, y=0.55, z=1.60), orientation=Quaternion(x=0.0, y=-0.05, z=0.05, w=1.0)),
        self.get_logger().info("Finished grasp planning logic.")
        response.cylinder_ids = [CylinderCombination(cylinder_ids=[1, 2])]
        response.grasp_pose = [Pose(position=Point(x=0.5, y=0.5, z=0.5), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))]
        response.place_pose = [Pose(position=Point(x=0.5, y=0.5, z=0.5), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))]
        return response

    def convert_image_mask_to_pixel_indices(self, mask, depth_width, depth_height):
        pixels = []
        bridge = CvBridge()
        self.get_logger().info(f"Mask shape: {mask.height} x {mask.width}")
        # Konvertieren von sensor_msgs/Image zu einem OpenCV-Bild
        cv_image = bridge.imgmsg_to_cv2(mask, desired_encoding='passthrough')
        self.get_logger().info(f"Mask shape: {cv_image.shape}")
        resized_image = cv2.resize(cv_image, (depth_width, depth_height))

        for i in range(depth_height):
            for j in range(depth_width):
                if resized_image[i, j] == 1:
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
        future = self.PtP_client.call_async(tranform_request)

        return future

def main(args=None):
    rclpy.init(args=args)
    grasp_planning_node = GraspPlanningNode()

    try:
        rclpy.spin(grasp_planning_node)
        grasp_planning_node.get_logger().info("Started Node")

    except KeyboardInterrupt:
        grasp_planning_node.get_logger().info("Stopped Node")

    # grasp_planning_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()