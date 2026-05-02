#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
# from autoware_auto_vehicle_msgs.msg import Engage
import time
from autoware_adapi_v1_msgs.srv import ChangeOperationMode
import os

class OperationModeChanger(Node):
    def __init__(self):
        super().__init__('operation_mode_changer')
        self.cli = self.create_client(ChangeOperationMode, '/api/operation_mode/change_to_autonomous')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = ChangeOperationMode.Request()

    def send_request(self):
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


class OperationModeStopper(Node):
    def __init__(self):
        super().__init__('operation_mode_changer')
        self.cli = self.create_client(ChangeOperationMode, '/api/operation_mode/change_to_stop')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = ChangeOperationMode.Request()

    def send_request(self):
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()



class PosePublisher(Node):
    def __init__(self):
        super().__init__('pose_publisher')
        self.initial_pose_publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.goal_pose_publisher = self.create_publisher(PoseStamped, '/planning/mission_planning/goal', 10)

    def publish_initial_pose(self, x, y, z, ox, oy, oz, ow):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = z
        msg.pose.pose.orientation.x = ox
        msg.pose.pose.orientation.y = oy
        msg.pose.pose.orientation.z = oz
        msg.pose.pose.orientation.w = ow
        self.initial_pose_publisher.publish(msg)
        self.get_logger().info(f'Published initial pose: {msg}')

    def publish_goal_pose(self, x, y, z, ox, oy, oz, ow):
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.x = ox
        msg.pose.orientation.y = oy
        msg.pose.orientation.z = oz
        msg.pose.orientation.w = ow
        self.goal_pose_publisher.publish(msg)
        self.get_logger().info(f'Published goal pose: {msg}')

def initial_sim(bool_val, args=None):
    
    try:
        rclpy.init()
    except:
        pass
    
    node = PosePublisher()

    # Publish initial pose
    # 100m from obs: 81662.17,50436.71
    node.publish_initial_pose(81644.8, 50528.1, 0.0, 0.0, 0.0, -0.609945, 0.792444)
    time.sleep(3)
    

    # Publish goal pose
    node.publish_goal_pose(81706.10, 50197.8, 0.0, 0.0, 0.0, -0.62531, 0.780921)

    # Keep the node alive for a short period to ensure messages are sent
    rclpy.spin_once(node, timeout_sec=2)
    node.destroy_node()

    time.sleep(3)

    node = OperationModeChanger()
    response = node.send_request()
    if response is not None:
        node.get_logger().info('Operation mode changed to autonomous successfully')
    else:
        node.get_logger().error('Failed to change operation mode')
    node.destroy_node()

    rclpy.shutdown()


def stop_sim(bool_val, args=None):

    if bool_val:
        rclpy.init(args=args)

    node = OperationModeStopper()
    response = node.send_request()
    if response is not None:
        node.get_logger().info('Operation mode changed to autonomous successfully')
    else:
        node.get_logger().error('Failed to change operation mode')
    node.destroy_node()

    rclpy.shutdown()

    

if __name__ == '__main__':
    main()