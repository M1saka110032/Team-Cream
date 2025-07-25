#/home/eugene/auvc_ws/src/tutorial_ardusub/tutorial_ardusub/calcdepth.py
import rclpy    # the ROS 2 client library for Python
from rclpy.node import Node    # the ROS 2 Node class
from sensor_msgs.msg import FluidPressure as Pressure
from std_msgs.msg import Float64

class DepthPublisher(Node):
    def __init__(self):
        super().__init__("depth_publisher")    # names the node when running
        self.DENSITY_WATER = 1000
        self.G = 9.81
        self.surface_pressure = 101325

        self.sub = self.create_subscription(
            Pressure,        # the message type
            "/pressure",    # the topic name,
            self.calc_depth,  # the subscription's callback method
            10              # QOS (will be covered later)
        )

        self.pub = self.create_publisher(
            Float64,        # the message type
            "/depth",    # the topic name
            10              # QOS (will be covered later)
        )

        self.get_logger().info("initialized publisher node")

    def calc_depth(self,msg):
        depth = -(msg.fluid_pressure-self.surface_pressure) / self.G / self.DENSITY_WATER
        depth_msg = Float64()
        depth_msg.data = depth
        self.pub.publish(depth_msg)
        self.get_logger().info(f"pressure:{msg.fluid_pressure:.2f}, depth: {depth:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = DepthPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()