import rclpy
from rclpy.node import Node
from dummy_perception_publisher.msg import Object
from autoware_auto_perception_msgs.msg import ObjectClassification, Shape
from geometry_msgs.msg import PoseWithCovariance, Twist
from unique_identifier_msgs.msg import UUID
import uuid
import rclpy.executors
import threading
import time
import math
class DummyBusPublisher(Node):
    def __init__(self):
        super().__init__('dummy_bus_publisher')
        self.publisher_ = self.create_publisher(Object, '/simulation/dummy_perception_publisher/object_info', 10)
        self.timer = self.create_timer(1.0, self.publish_dummy_bus)
        self.bus_id = self._generate_uuid()
        self.x_pos = 0.0
        self.y_pos = 0.0
        self.quat = [1.0, 0.0, 0.0, 0.0]  # Default quaternion
        self.bus = None
        self.moving = False
        self.movement_duration = 0.0
        self.speed_vector = [0.0, 0.0]
        self.start_time = None
        self.last_update = self.get_clock().now()

    def _generate_uuid(self):
        new_id = UUID()
        new_id.uuid = list(uuid.uuid4().bytes)
        return new_id

    def publish_dummy_bus(self):
        bus = Object()

        # Header
        bus.header.frame_id = "map"
        bus.header.stamp = self.get_clock().now().to_msg()

        # Classification
        bus.classification.label = ObjectClassification.CAR
        bus.classification.probability = 1.0

        # Shape
        bus.shape.type = Shape.BOUNDING_BOX
        bus.shape.dimensions.x = 4.0  # length
        bus.shape.dimensions.y = 2.0   # width
        bus.shape.dimensions.z = 2.0   # height

        quat = self.quat
        # Initial state with specified pose
        bus.initial_state.pose_covariance = PoseWithCovariance()
        bus.initial_state.pose_covariance.pose.position.x = self.x_pos
        bus.initial_state.pose_covariance.pose.position.y = self.y_pos
        bus.initial_state.pose_covariance.pose.position.z = 0.0
        bus.initial_state.pose_covariance.pose.orientation.w = quat[3]#0.8696213349078672
        bus.initial_state.pose_covariance.pose.orientation.x = quat[0]#-0.3
        bus.initial_state.pose_covariance.pose.orientation.y = quat[1]#0.0
        bus.initial_state.pose_covariance.pose.orientation.z = quat[2]#-0.6135944486860337
        bus.initial_state.pose_covariance.covariance = [0.0] * 36

        # Twist (velocity)
        bus.initial_state.twist_covariance.twist = Twist()
        bus.initial_state.twist_covariance.twist.linear.x = 0.0  # You can set a specific velocity if needed

        # Unique ID
        bus.id = self.bus_id

        self.publisher_.publish(bus)
        self.bus = bus
        self.get_logger().info('Published dummy bus at specified location')
        
    def modify_bus_position(self, new_x: float, new_y: float, new_quaternion: list):
        modify_msg = self.bus
        
        # Maintain same header and ID
        modify_msg.header.frame_id = "map"
        modify_msg.header.stamp = self.get_clock().now().to_msg()
        modify_msg.id = self.bus_id
        
        # Set modification action
        modify_msg.action = Object.MODIFY
        
        # Update position and orientation
        modify_msg.initial_state.pose_covariance.pose.position.x = new_x
        modify_msg.initial_state.pose_covariance.pose.position.y = new_y
        modify_msg.initial_state.pose_covariance.pose.orientation.x = new_quaternion[0]
        modify_msg.initial_state.pose_covariance.pose.orientation.y = new_quaternion[1]
        modify_msg.initial_state.pose_covariance.pose.orientation.z = new_quaternion[2]
        modify_msg.initial_state.pose_covariance.pose.orientation.w = new_quaternion[3]
        self.publisher_.publish(modify_msg)
        self.get_logger().info(f'Modified bus position to ({new_x}, {new_y})')

    def delete_all_objects(self):
        delete_msg = Object()
        delete_msg.header.frame_id = "map"
        delete_msg.header.stamp = self.get_clock().now().to_msg()
        delete_msg.action = Object.DELETEALL

        self.publisher_.publish(delete_msg)
        self.get_logger().info('Deleted all objects')
    
def generate_bus(bool_val, x_pos, y_pos, quaternion):
    try:
        rclpy.init()
    except:
        pass
    dummy_bus_publisher = DummyBusPublisher()
    dummy_bus_publisher.x_pos = x_pos
    dummy_bus_publisher.y_pos = y_pos
    dummy_bus_publisher.quat = quaternion
    dummy_bus_publisher.publish_dummy_bus()
    rclpy.spin_once(dummy_bus_publisher)
    return dummy_bus_publisher

def delete_all_objects():
    rclpy.init()
    dummy_bus_publisher = DummyBusPublisher()
    dummy_bus_publisher.delete_all_objects()
    rclpy.spin_once(dummy_bus_publisher)
    dummy_bus_publisher.destroy_node()
    rclpy.shutdown()



def start_moving_bus(bus_publisher, speed_x: float, speed_y: float, quat, duration: float):
    """Starts continuous movement in background thread"""
    movement_thread = threading.Thread(
        target=_move_bus_worker,
        args=(bus_publisher, speed_x, speed_y, quat, duration),
        daemon=True
    )
    movement_thread.start()
    return movement_thread

def stop_moving_bus(thread: threading.Thread):
    """Stops ongoing movement"""
    if thread.is_alive():
        thread.join(0.1)

def _move_bus_worker(bus_publisher, speed_x: float, speed_y: float,quat, duration: float):
    """Worker function for movement calculations"""
    start_time = time.time()
    dt = 0.1  # Update interval
    
    # Convert speed to orientation quaternion
    yaw = math.atan2(speed_y, speed_x)
    
    try:
        while (time.time() - start_time) < duration:
            # Update position
            bus_publisher.x_pos += speed_x * dt
            bus_publisher.y_pos += speed_y * dt
            
            # Update orientation
            bus_publisher.quat = quat
            
            # Publish modification
            bus_publisher.modify_bus_position(
                bus_publisher.x_pos,
                bus_publisher.y_pos,
                bus_publisher.quat
            )
            
            time.sleep(dt)
    finally:
        # Ensure final position is published
        bus_publisher.modify_bus_position(
            bus_publisher.x_pos,
            bus_publisher.y_pos,
            bus_publisher.quat
        )

def main(args=None):
    rclpy.init(args=args)
    #dummy_bus_publisher = DummyBusPublisher()

    
    dummy_bus_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
