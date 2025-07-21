import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from mavros_msgs.msg import ManualControl
from std_msgs.msg import Int16

class ControlCombiner(Node):
    def __init__(self):
        super().__init__("control_combiner")
        self.depth_control = 0.0
        self.heading_control = 0.0

        self.depth_sub = self.create_subscription(
            Float64, "/depth_control_output", self.depth_callback, 10)

        self.heading_sub = self.create_subscription(
            Float64, "/heading_control_output", self.heading_callback, 10)

        self.pub = self.create_publisher(ManualControl, "/manual_control", 10)

        self.timer = self.create_timer(0.1, self.publish_manual_control)  # 50Hz

        self.get_logger().info("Initialized ControlCombiner node")

    def depth_callback(self, msg):
        self.depth_control = msg.data
        self.get_logger().info(f"Received depth control: z={self.depth_control:.2f}")

    def heading_callback(self, msg):
        self.heading_control = msg.data
        self.get_logger().info(f"Received heading control: r={self.heading_control:.2f}")

    def publish_manual_control(self):
        msg = ManualControl()
        msg.x = 0.0
        msg.y = 0.0
        msg.z = float(self.depth_control)
        msg.r = float(self.heading_control)
        msg.buttons = 0
        self.pub.publish(msg)
        self.get_logger().info(f"Published ManualControl: z={msg.z:.2f}, r={msg.r:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = ControlCombiner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()