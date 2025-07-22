#/home/eugene/auvc_ws/src/tutorial_ardusub/tutorial_ardusub/depthhold.py
import rclpy    # the ROS 2 client library for Python
from rclpy.node import Node    # the ROS 2 Node class
from std_msgs.msg import Float64
import numpy as np

class DepthControl(Node):
    def __init__(self):
        super().__init__("depth_control")    # names the node when running

        self.kp = 50
        self.ki = 0.5
        self.kd = 20

        self.i = 0.0


        self.p_error = 0.0
        self.p_time = self.get_clock().now()


        self.goal = -1.5

        self.sub = self.create_subscription(
            Float64,        # the message type
            "/depth",    # the topic name,
            self.depth_control,  # the subscription's callback method
            10              # QOS (will be covered later)
        )

        self.pub = self.create_publisher(Float64, "/depth_control_output", 10)

        self.goal_sub = self.create_subscription(Float64, "/set_depth_goal", self.set_goal_callback, 10)

        self.get_logger().info("initialized pressure node")
    
    def set_goal_callback(self, msg):
        new_goal = msg.data
        if new_goal > 0:
            self.get_logger().warn("Goal depth must be non-positive, ignoring update.")
            return
        self.goal = new_goal
        self.get_logger().info(f"Updated goal depth to: {self.goal:.2f} meters")

    def depth_control(self, msg):
        self.depth = msg.data
        self.c_error = self.goal - self.depth
        self.c_time = self.get_clock().now()
        self.d_time = (self.c_time-self.p_time).nanoseconds/10**9

        self.u_p = self.c_error * self.kp

        self.i += self.d_time * self.c_error
        self.u_i = self.ki * self.i

        if self.d_time > 0:
            derivative = (self.c_error - self.p_error) / self.d_time
        else:
            derivative = 0.0
        self.u_d = self.kd * derivative

        self.u = (self.u_p + self.u_i + self.u_d)

        self.u = np.clip(self.u, -60, 60)

        m = Float64()
        m.data = self.u
        self.pub.publish(m)

        self.p_time = self.c_time
        self.p_error = self.c_error

        self.get_logger().info(f"Z Goal: {self.goal:.2f} Z Error: {self.c_error:.2f} Z Force: {self.u:.2f}")


        


    
    


def main(args=None):
    rclpy.init(args=args)
    node = DepthControl()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()