import rclpy    # the ROS 2 client library for Python
from rclpy.node import Node    # the ROS 2 Node class
from mavros_msgs.msg import ManualControl    # the ManualControl message type definition


class forward_move(Node):
    def __init__(self):
        super().__init__("forward_move")
        self.pub = self.create_publisher(ManualControl, "/manual_control", 10)
        self.timer = self.create_timer(1.0, self.publish_manual_control)  # 1 Hz
        self.publish_count = 0
        self.get_logger().info("Initialized forward_move node")

    def publish_manual_control(self):
        if self.publish_count < 10:  # 10 times after close the node
            msg = ManualControl()
            msg.x = 50.0
            msg.y = 0.0
            msg.z = 0.0
            msg.r = 50.0
            msg.buttons = 0
            self.pub.publish(msg)
            self.get_logger().info("Moving: msg.x={}, msg.y={}, msg.z={}, msg.r={}".format(msg.x, msg.y, msg.z,msg.r))
            self.publish_count += 1
        else:
            self.stop_and_shutdown()

    def stop_and_shutdown(self):
        # set 0
        msg = ManualControl()
        msg.x = 0.0
        msg.y = 0.0
        msg.z = 0.0
        msg.r = 0.0
        msg.buttons = 0
        self.pub.publish(msg)
        self.get_logger().info("Stop Moving.")
        self.get_logger().info("Shutting down forward_move node")
        # cancel time
        self.timer.cancel()
        self.destroy_node()
        

def main(args=None):
    rclpy.init(args=args)
    node = forward_move()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()