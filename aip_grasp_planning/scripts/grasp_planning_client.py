#!/usr/bin/env python3

# This client is used to call a user defined GraspPlanning service with example messages.
# It can be used for debugging purposes or for hardware independent development.


import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from sensor_msgs.msg import Image, RegionOfInterest
from geometry_msgs.msg import Pose, Vector3, Point, Quaternion
from aip_grasp_planning_interfaces.srv import GraspPlanning
from aip_grasp_planning_interfaces.msg import CylinderCombination
from object_detector_tensorflow_interfaces.msg import Detections, Detection
import numpy as np
from aip_packing_planning_interfaces.msg import Package, PackageSequence
from std_msgs.msg import Header
import cv2

class GraspPlanningNode(Node):
    def __init__(self):
        super().__init__('grasp_planning_node')
        self.service_group = MutuallyExclusiveCallbackGroup()  # Only one callback in the group can be executed at a time
        self.get_logger().info("Started grasp_planning_client")

        # Create a client for the GraspPlanning service
        self.grasp_planning_client = self.create_client(GraspPlanning, 'grasp_planning_node/grasp_planning')

        # Wait for the service to be available
        while not self.grasp_planning_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('GraspPlanning service not available, waiting...')

    def call_grasp_planning_service(self, objects_to_pick, package_sequence, depth_image, detections):
        # Create a request object
        grasp_planning_request = GraspPlanning.Request()
        grasp_planning_request.objects_to_pick = objects_to_pick
        grasp_planning_request.package_sequence = package_sequence
        grasp_planning_request.depth_image = depth_image
        grasp_planning_request.detections = detections

        # Send the request
        future = self.grasp_planning_client.call_async(grasp_planning_request)

        return future


def main(args=None):
    rclpy.init(args=args)

    grasp_planning_node = GraspPlanningNode()

    # Example usage of the service call
    objects_to_pick = ["example_class"]
    package_sequence = PackageSequence()  # Populate with actual data
    
    depth_image = Image()  # Populate with actual data
    depth_image.width = 100
    depth_image.height = 100
    depth_image.encoding = "32FC1"
    depth_image.step = depth_image.width * 4
    # Convert the depth image to a numpy array
    depth_image_np = np.linspace(0.5, 0.7, depth_image.width * depth_image.height ,dtype=np.float32).reshape((depth_image.height, depth_image.width))
    # Transpose the depth image matrix
    depth_image_np1 = np.transpose(depth_image_np)
    
    depth_image_np = depth_image_np + depth_image_np1
    # Flatten the numpy array and assign it to depth_image.data
    depth_image.data = depth_image_np.flatten().tobytes()
    # Convert the depth image to a numpy array
    depth_image_np = np.frombuffer(depth_image.data, dtype=np.float32).reshape((depth_image.height, depth_image.width))

    # Display the depth image
    cv2.imshow("Depth Image", depth_image_np)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    image_header = Header()
    image_header.stamp = rclpy.time.Time().to_msg()
    detections = Detections()
    detections.header = image_header
    detections.image_header = image_header
    detection = Detection()
    detection.class_id = 1
    detection.instance_id = 1
    detection.class_name = "example_class"
    detection.probability = 0.9
    center = Point()
    center.x = 0.5
    center.y = 0.5
    center.z = 0.0
    bounding_box = RegionOfInterest()
    bounding_box.x_offset = 10
    bounding_box.y_offset = 10
    bounding_box.width = 20
    bounding_box.height = 20
    bounding_box.do_rectify = False

    mask = Image()
    mask.width = 100
    mask.height = 100
    mask.encoding = "32FC1"
    mask.step = mask.width * 4
    mask.data = np.ones(mask.width * mask.height, dtype=np.float32).tobytes()
    orientation = Quaternion()
    orientation.x = 0.0
    orientation.y = 0.0
    orientation.z = 0.367
    orientation.w = 0.93
    detection.center = center
    detection.bounding_box = bounding_box
    detection.mask = mask
    detection.orientation = orientation
    detections.detections.append(detection)

    class_name = "example_class"
    dimensions = Vector3()
    dimensions.x = 1.0
    dimensions.y = 2.0
    dimensions.z = 3.0
    weight = 0.5
    rotation_index = 1
    place_coordinates = Point()
    place_coordinates.x = 0.0
    place_coordinates.y = 0.0
    place_coordinates.z = 0.0

    package = Package()
    package.class_name = class_name
    package.dimensions = dimensions
    package.weight = weight
    package.rotation_index = rotation_index
    package.place_coordinates = place_coordinates
    package_sequence.packages.append(package)

    future = grasp_planning_node.call_grasp_planning_service(objects_to_pick, package_sequence, depth_image, detections)

    try:
        rclpy.spin_until_future_complete(grasp_planning_node, future)
    except KeyboardInterrupt:
        grasp_planning_node.get_logger().info("Stopped Node")
    finally:
        grasp_planning_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()