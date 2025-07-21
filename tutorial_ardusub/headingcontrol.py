import rclpy    # the ROS 2 client library for Python
from rclpy.node import Node    # the ROS 2 Node class
from std_msgs.msg import Int16 , Float64    # the Int16 message type definition
import numpy as np

class headingcontrol(Node):
    def __init__(self):
        super().__init__("heading_control")    # names the node when running
        self.kp = 0.5
        self.ki = 0.01
        self.kd = 0.1

        self.i = 0.0
        self.goal = 10

        self.p_error = 0
        self.p_time = self.get_clock().now()

        self.sub = self.create_subscription(
            Int16,        # the message type
            "/heading",    # the topic name,
            self.heading_control,  # the subscription's callback method
            10              # QOS (will be covered later)
        )

        self.goal_sub = self.create_subscription(
            Int16, "/set_heading_goal", self.set_goal_callback, 10)

        self.pub = self.create_publisher(Float64, "/heading_control_output", 10)

        self.get_logger().info("initialized HeadingControl node")

    def set_goal_callback(self, msg):
        new_goal = msg.data
        if new_goal <= 0 or new_goal >= 359:
            self.get_logger().warn("Goal heading must be between 0 and 359 degrees, ignoring update.")
            return
        self.goal = new_goal
        self.get_logger().info(f"Updated goal heading to: {self.goal} degrees")
    
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
        
        m = Float64()
        m.data = u
        self.pub.publish(m)

        self.p_time = self.c_time
        self.p_error = c_error

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