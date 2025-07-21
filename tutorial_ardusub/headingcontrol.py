import rclpy    # the ROS 2 client library for Python
from rclpy.node import Node    # the ROS 2 Node class
from std_msgs.msg import Int16    # the Int16 message type definition
from mavros_msgs.msg import ManualControl
import numpy as np

class headingcontrol(Node):
    def __init__(self):
        super().__init__("heading_control")    # names the node when running
        self.kp = 0.5
        self.ki = 0.01
        self.kd = 0.1

        self.i = 0.0
        self.goal = 90

        self.p_error = 0
        self.p_time = self.get_clock().now()

        self.sub = self.create_subscription(
            Int16,        # the message type
            "/heading",    # the topic name,
            self.heading_control,  # the subscription's callback method
            10              # QOS (will be covered later)
        )

        self.pub = self.create_publisher(ManualControl, "/manual_control", 10)

        self.get_logger().info("initialized HeadingControl node")

    def publish_manual_control(self, r):
        msg = ManualControl()
        msg.x = 0.0
        msg.y = 0.0
        msg.z = 0.0
        msg.r = float(r)
        msg.buttons = 0
        self.pub.publish(msg)
        self.get_logger().info("Moving: msg.x={}, msg.y={}, msg.z={}, msg.r={}".format(msg.x, msg.y, msg.z,msg.r))
    
    def heading_control(self, msg):
        self.heading = msg.data
        c_error = self.goal - self.heading
        c_error = (c_error + 180) % 360 - 180

        self.c_time = self.get_clock().now()
        self.d_time = (self.c_time-self.p_time).nanoseconds/10**9

        u_p = c_error * self.kp

        self.i += self.d_time * c_error
        u_i = self.ki * self.i

        if self.d_time > 0:
            derivative = (c_error - self.p_error) / self.d_time
        else:
            derivative = 0.0
        u_d = self.kd * derivative

        u = u_p + u_i + u_d

        u = np.clip(u, -60, 60)

        self.publish_manual_control(u)

        self.p_time = self.c_time
        self.p_error = self.p_error

        self.get_logger().info(f"Goal: {self.goal} Error: {c_error} Force: {u}")

def main(args=None):
    rclpy.init(args=args)
    node = headingcontrol()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()