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

from aip_grasp_planning.cylinder_selection import choose_cylinder
import cv2
from cv_bridge import CvBridge

from scipy.spatial.transform import Rotation
import copy


class GraspPlanningNode(Node):
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

        self.grasp_poses_with_offset_publisher = self.create_publisher(PoseArray, '/grasp_poses_with_offset', 10)        


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

        # for package in packages:
        #     self.get_logger().info("PACKAGE " + str(package.class_name) + " with dimensions: " + str(package.dimensions) + " and weight: " + str(package.weight))

        # Choose Cylinder per package based on the package dimensions and weight
        _, cylinder_ids_per_package, tcps_cylinder_offsets = choose_cylinder(packages_weights, packages_length, packages_width) 
        self.get_logger().info("Cylinder IDs per package: " + str(cylinder_ids_per_package))
        self.get_logger().info("TCP offsets per package: " + str(tcps_cylinder_offsets))
        response.cylinder_ids = cylinder_ids_per_package

        pack_sequence_class_names = []
        for package in packages:
            pack_sequence_class_names.append(package.class_name)

        cylinder_ejection_offset = 0.14
        used_mask_indices = []

        grasp_pose_array = PoseArray()
        grasp_pose_array.header.frame_id = "world"
        grasp_pose_array.header.stamp = self.get_clock().now().to_msg()
    
        grasp_poses_with_offsets_array = PoseArray()
        grasp_poses_with_offsets_array.header.frame_id = "world"
        grasp_poses_with_offsets_array.header.stamp = self.get_clock().now().to_msg()
        
        start_idx = 0
               
        grasp_poses_with_offset = []
        
        for idx in range(len(pack_sequence_class_names)):

            self.get_logger().info("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXx")
            self.get_logger().info(f"Grasp planning for package with CLASS NAME: {pack_sequence_class_names[idx]} and INDEX: {idx}")

            satisfied = False
            mask_id_detections = start_idx
            while not satisfied:
                mask_id_detections = class_names.index(pack_sequence_class_names[idx], start_idx)
                if mask_id_detections not in used_mask_indices:
                    satisfied = True
                    used_mask_indices.append(mask_id_detections)
                else:
                    start_idx = mask_id_detections + 1
            mask = masks[mask_id_detections]
            self.get_logger().info(f"Class recognized: {class_names[mask_id_detections]}")

            # Convert the mask image to a list of pixels
            pixels = self.convert_image_mask_to_pixel_indices(mask, depth_image.width, depth_image.height)

            # Call the 'pixel_to_point' method to convert the pixels to points
            if idx == 0:
                reset_viz = True
            else:
                reset_viz = False
            future = self.pixel_to_point_async(pixels, depth_image.height, depth_image.width, depth_image, reset_viz)
            await future
            point_response = future.result()

            # Call the 'grasp_pose_client' servicefilterCloud to get the grasp poses
            grasp_pose_request = GraspObjectSurfaceNormal.Request()
            grasp_pose_request.reset_viz = reset_viz
            grasp_pose_request.masked_points = point_response.points       #request type = geometry_msgs/Point[]
            future = self.grasp_pose_client.call_async(grasp_pose_request)  #response type = geometry_msgs/Pose surface_normal_to_grasp
            await future
            grasp_pose_response = future.result()

            self.get_logger().info("Grasp Pose from GraspObjectSURFACENormal: " + str(grasp_pose_response.surface_normal_to_grasp))
            grasp_pose = Pose()
            grasp_pose.position.x = grasp_pose_response.surface_normal_to_grasp.position.x
            grasp_pose.position.y = grasp_pose_response.surface_normal_to_grasp.position.y
            grasp_pose.position.z = grasp_pose_response.surface_normal_to_grasp.position.z

            # Transform the grasp pose from camera to the base_link frame
            t = self.tf_buffer.lookup_transform("base_link", "camera", rclpy.time.Time())
            grasp_pose = do_transform_pose(grasp_pose, t)
            self.get_logger().info(f"Grasp pose after transform base_link: {grasp_pose}")
            # Transform the grasp pose from base_link to the world frame
            t_w = self.tf_buffer.lookup_transform("world", "base_link", rclpy.time.Time())
            grasp_pose = do_transform_pose(grasp_pose, t_w)

            # Calculate the orientation of the grasp pose by using the orientation of the surface normal to the grasp pose
            orientation = orientations[mask_id_detections]
            q1 = Rotation.from_quat([-grasp_pose_response.surface_normal_to_grasp.orientation.x, grasp_pose_response.surface_normal_to_grasp.orientation.y, grasp_pose_response.surface_normal_to_grasp.orientation.z, -grasp_pose_response.surface_normal_to_grasp.orientation.w])
            q2 = Rotation.from_quat([orientation.x, orientation.y, orientation.z, orientation.w])
            deg = q2.as_euler('zyx')[0]
            self.get_logger().info("Orientation: " + str(deg))
            q2 = Rotation.from_euler('z', deg)

            # Combine the two rotations
            q_combined = q1 * q2

            # Set the orientation of the grasp pose
            grasp_pose.orientation.x = q_combined.as_quat()[0]
            grasp_pose.orientation.y = q_combined.as_quat()[1]
            grasp_pose.orientation.z = q_combined.as_quat()[2]
            grasp_pose.orientation.w = q_combined.as_quat()[3]
            
            grasp_pose_array.poses.append(copy.deepcopy(grasp_pose))

            # Retrieve the TCP offsets for the cylinder corresponding to the package
            x_offset = tcps_cylinder_offsets[idx].translation[0]
            y_offset = tcps_cylinder_offsets[idx].translation[1]
            z_offset = tcps_cylinder_offsets[idx].translation[2] + cylinder_ejection_offset # Add ejection offset for pneumatic cylinder
            
            # Multiply the offsets with the rotation matrix
            offsets = np.array([x_offset, y_offset, z_offset])
            offsets_rotated = np.dot(q_combined.as_matrix(), offsets)
               
            self.get_logger().info("TCP Offsets: " + str(offsets))
           
            # Update the grasp pose with the rotated offsets
            grasp_pose_with_offset = copy.deepcopy(grasp_pose)
            grasp_pose_with_offset.position.x +=  offsets_rotated[0]
            grasp_pose_with_offset.position.y += offsets_rotated[1]
            grasp_pose_with_offset.position.z += offsets_rotated[2]         
            
            self.get_logger().info("Grasp pose in WORLD WITH TCP Offset: " + str(grasp_pose))
            grasp_poses_with_offset.append(grasp_pose_with_offset)
            grasp_poses_with_offsets_array.poses.append(grasp_pose_with_offset)          

        # Publish the grasp poses with/without offsets to visualize in RViz
        self.grasp_poses_publisher.publish(grasp_pose_array)
        self.grasp_poses_with_offset_publisher.publish(grasp_poses_with_offsets_array)
        
        response.grasp_pose = grasp_poses_with_offset

        ### PLACE Pose Definition ###
        # Container corner reference 
        container_corner = Point(x=0.75, y=0.1, z=0.95)   #container size = 0.585, 0.392, 0.188 -> see packing_algorithm docker container 
        place_poses = []

        # Calculate the place pose for each package
        for idx, package in enumerate(packages):
            self.get_logger().info("PLACE Pose for package with CLASS NAME: " + str(package.class_name) + " with INDEX: " + str(idx))

            #package.rotation_index = 3  #For TESTING

            # Check target orientation per package from the packing plan and calculate the place pose
            if package.rotation_index == 1: #long tcp side across to long package side 
                place_pose = Pose(
                            position=Point(x=container_corner.x + package.place_coordinates.y + tcps_cylinder_offsets[idx].translation[0], 
                                           y=container_corner.y + package.place_coordinates.x + tcps_cylinder_offsets[idx].translation[1], 
                                        #    z=container_corner.z + package.dimensions.z + tcps_cylinder_offsets[idx].translation[2] + cylinder_ejection_offset),  # TODO: Change to allow multi layer placements
                                           z=container_corner.z + package.place_coordinates.z + tcps_cylinder_offsets[idx].translation[2] + cylinder_ejection_offset),  # TODO: Change to allow multi layer placements
                            
                            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
                
            elif package.rotation_index == 3: #long tcp side parallel to long package side
                place_pose = Pose(
                            position=Point(x=container_corner.x + package.place_coordinates.x + tcps_cylinder_offsets[idx].translation[1], 
                                           y=container_corner.y + package.place_coordinates.y + tcps_cylinder_offsets[idx].translation[0], 
                                        #    z=container_corner.z + package.dimensions.z + tcps_cylinder_offsets[idx].translation[2] + cylinder_ejection_offset), # TODO: Change to allow multi layer placements
                                           z=container_corner.z + package.place_coordinates.z + tcps_cylinder_offsets[idx].translation[2] + cylinder_ejection_offset),  # TODO: Change to allow multi layer placements
                            orientation=Quaternion(x=0.0, y=0.0, z=-np.sin(np.pi / 4), w=np.cos(np.pi / 4))
                        )

            self.get_logger().info("PLACE Pose in WORLD WITH TCP Offset: " + str(place_pose))

            place_poses.append(place_pose)

        response.place_pose = place_poses

        self.get_logger().info("Cylinder Ids " + str(response.cylinder_ids))

        # Set the amount of objects to place
        amount_of_objects_to_place = len(packages)
        response.amount_of_objects_to_place = amount_of_objects_to_place

        self.get_logger().info("Finished grasp planning logic.")
        return response

    def convert_image_mask_to_pixel_indices(self, mask, depth_width, depth_height):
        pixels = []
        bridge = CvBridge()
        self.get_logger().info(f"Mask shape: {mask.height} x {mask.width}")

        # Convert sensor_msgs/Image to an OpenCV image
        cv_image = bridge.imgmsg_to_cv2(mask, desired_encoding='passthrough')
        self.get_logger().info(f"Mask shape: {cv_image.shape}")
        resized_image = cv2.resize(cv_image, (depth_width, depth_height))

        for i in range(depth_height):
            for j in range(depth_width):
                if resized_image[i, j] == 1:
                    pixels.append((j, i))
        return pixels
        
    def pixel_to_point_async(self, pixels: list, height=0, width=0, depth_image:Image=Image()):
        # Convert the list of pixels to a list of Point objects
        points_msg = []
        for pixel in pixels:
            point = Point()
            point.x = float(pixel[0])
            point.y = float(pixel[1])
            point.z = 0.0  # Set the z-coordinate to 0 for now
            points_msg.append(point)

        # Create a request message for the 'PixelToPoint' service
        transform_request = PixelToPoint.Request()
        transform_request.pixels = points_msg
        transform_request.height = int(height)
        transform_request.width = int(width)
        transform_request.depth_image = depth_image
        transform_request.camera_type = "roboception"

        # Call the 'PixelToPoint' service asynchronously
        future = self.PtP_client.call_async(transform_request)
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