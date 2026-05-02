import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped

class PositionNode(Node):
    def __init__(self):
        super().__init__('position_node')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def get_position(self):
        max_attempts = 10
        attempt = 0
        while attempt < max_attempts:
            try:
                transform: TransformStamped = self.tf_buffer.lookup_transform(
                    'map',
                    'base_link',
                    rclpy.time.Time())
                
                # Extract and return the position
                position = transform.transform.translation
                return position
            
            except TransformException as ex:
                attempt += 1
                rclpy.spin_once(self, timeout_sec=1.0)
        
        self.get_logger().error("Failed to get transform after multiple attempts")
        return None

def get_location(bool_val):
    try:
        rclpy.init()
    except:
        pass
    node = PositionNode()
    position = node.get_position()
    if position:
        pass
    else:
        print("Failed to get position")
    node.destroy_node()
    # rclpy.shutdown()
    return position

if __name__ == '__main__':
    position = get_location()
    print(f"Position: x={position.x:.2f}, y={position.y:.2f}, z={position.z:.2f}")
