import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from mavros_msgs.msg import ManualControl
from std_msgs.msg import Int16

class ControlCombiner(Node):
    def __init__(self):
        super().__init__("control_combiner")
        self.x_output = 0.0
        self.y_output = 0.0
        self.depth_control = 0.0
        self.heading_control = 0.0

        self.depth_sub = self.create_subscription(
            Float64, "/depth_control_output", self.depth_callback, 10)

        self.heading_sub = self.create_subscription(
            Float64, "/heading_control_output", self.heading_callback, 10)

        self.pub = self.create_publisher(ManualControl, "/manual_control", 10)

        self.x_sub = self.create_subscription(Float64, "/set_x_output", self.set_x_output_callback, 10)

        self.y_sub = self.create_subscription(Float64, "/set_y_output", self.set_y_output_callback, 10)

        self.timer = self.create_timer(0.1, self.publish_manual_control)  # 10Hz


        self.get_logger().info("Initialized ControlCombiner node")

    def depth_callback(self, msg):
        self.depth_control = msg.data
        self.get_logger().info(f"Received depth control: z={self.depth_control:.2f}")

    def heading_callback(self, msg):
        self.heading_control = msg.data
        self.get_logger().info(f"Received heading control: r={self.heading_control:.2f}")
    
    def set_x_output_callback(self, msg):
        new_x_output = msg.data
        if new_x_output < -60 or new_x_output > 60:
            self.get_logger().warn("x_output can not samller than -60 or bigger than 60.")
            return
        self.x_output = new_x_output
        self.get_logger().info(f"Updated x_output to: {self.x_output:.2f} ")

    def set_y_output_callback(self, msg):
        new_y_output = msg.data
        if new_y_output < -60 or new_y_output > 60:
            self.get_logger().warn("x_output can not samller than -60 or bigger than 60.")
            return
        self.y_output = new_y_output
        self.get_logger().info(f"Updated y_output to: {self.y_output:.2f} ")

    def publish_manual_control(self):
        msg = ManualControl()
        msg.x = float(self.x_output)
        msg.y = float(self.y_output)
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