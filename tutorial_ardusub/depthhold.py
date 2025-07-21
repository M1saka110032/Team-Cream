
import rclpy    # the ROS 2 client library for Python
from rclpy.node import Node    # the ROS 2 Node class
from sensor_msgs.msg import FluidPressure as Pressure
from mavros_msgs.msg import ManualControl
import numpy as np

class PressureSubscriber(Node):
    def __init__(self):
        super().__init__("depth_control")    # names the node when running
        self.DENSITY_WATER = 1000
        self.G = 9.81
        self.kp = 50
        self.ki = 0.5
        self.kd = 20

        self.i = 0.0
        self.surface_prees = 101325

        self.p_error = 0.0
        self.p_time = self.get_clock().now()

        self.goal = -2.0

        self.sub = self.create_subscription(
            Pressure,        # the message type
            "/pressure",    # the topic name,
            self.depth_control,  # the subscription's callback method
            10              # QOS (will be covered later)
        )

        self.pub = self.create_publisher(ManualControl, "/manual_control", 10)

        self.get_logger().info("initialized pressure node")
    
    def publish_manual_control(self, z):
        msg = ManualControl()
        msg.x = 0.0
        msg.y = 0.0
        msg.z = float(z)
        msg.r = 0.0
        msg.buttons = 0
        self.pub.publish(msg)
        self.get_logger().info("Moving: msg.x={:.2f}, msg.y={:.2f}, msg.z={:.2f}, msg.r={:.2f}".format(msg.x, msg.y, msg.z,msg.r))
    

    def depth_control(self, msg):
        
        self.depth = -(msg.fluid_pressure-self.surface_prees) / self.G / self.DENSITY_WATER
        self.get_logger().info(f"pressure:{msg.fluid_pressure}, depth: {self.depth}")
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

        self.publish_manual_control(self.u)

        self.p_time = self.c_time
        self.p_error = self.c_error

        self.get_logger().info(f"Goal: {self.goal:.2f} Error: {self.c_error:.2f} Force: {self.u:.2f}")


        


    
    


def main(args=None):
    rclpy.init(args=args)
    node = PressureSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()