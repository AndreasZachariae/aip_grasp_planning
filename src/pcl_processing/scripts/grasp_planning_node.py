#!/usr/bin/env python3

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

from geometry_msgs.msg import Point
from point_transformation_interfaces.srv import PixelToPoint

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

class GraspPlanning(Node):
    def __init__(self):
        super().__init__('depth_image_sub')
        self.transform_service_group = MutuallyExclusiveCallbackGroup()

        self.image_subscriber = self.create_subscription(Image, '/stereo/depth', self.image_callback, 10)
        self.get_logger().info("Started depth_image_sub")

        self.client = self.create_client(
        PixelToPoint, 'point_transformation_node/pixel_to_point', callback_group=self.transform_service_group)

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('PixelToPoint service not available, waiting again...')


    def pixel_to_point_async(self, pixels: list, height=0, width=0, depth_image:Image=Image()):

        points_msg = []
        for pixel in pixels:
            # Create individual Point objects and add them to the list
            point = Point()
            point.x = float(pixel[0])
            point.y = float(pixel[1])
            point.z = 0.0  # Set the z-coordinate to 0 for now
            points_msg.append(point)

        tranform_request = PixelToPoint.Request()
        tranform_request.pixels = points_msg
        tranform_request.height = int(height)
        tranform_request.width = int(width)
        tranform_request.depth_image = depth_image
        tranform_request.camera_type = "roboception"

        transform_response: PixelToPoint.Response = self.client.call_async(tranform_request)
       
        return transform_response

    def pixel_to_point(self, pixels: list, height=0, width=0, depth_image:Image=Image()):
        future = self.pixel_to_point_async(pixels, height, width, depth_image)
        self.get_logger().info('before spin_until_future_complete')

        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response is not None:
            self.get_logger().info('Received response: %s' % str(future.result().points))
        else:
            self.get_logger().error('Exception while calling service: %r' % future.exception())
        self.get_logger().info(str(response.points))
        return response.points
    
    def image_callback(self, msg):
        self.point = self.pixel_to_point([(5,10), (100,100)], msg.height, msg.width, msg)
        self.get_logger().info("after client call")
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