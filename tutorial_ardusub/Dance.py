#/home/eugene/auvc_ws/src/tutorial_ardusub/tutorial_ardusub/Dance
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from mavros_msgs.msg import OverrideRCIn, ManualControl
import time

class Dance(Node):
    def __init__(self):
        super().__init__('arm_client')
        self.cli = self.create_client(SetBool, '/arming')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting /arming ...')
        
        self.pub = self.create_publisher(ManualControl, "/manual_control", 10)
        self.command_pub = self.create_publisher(OverrideRCIn, "override_rc", 10)
        self.call_service(True, "Vehicle armed")
        self.dance()
        
    
    def publish_manual_control(self,x,y,z,r):
        msg = ManualControl()
        msg.x = float(x)
        msg.y = float(y)
        msg.z = float(z)
        msg.r = float(r)
        msg.buttons = 0
        self.pub.publish(msg)
        self.get_logger().info("Moving: x={}, y={}, z={}".format(msg.x, msg.y, msg.z))



    def call_service(self, arm: bool, logmsg: str):
        req = SetBool.Request()
        req.data = arm
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() and future.result().success:
            self.get_logger().info(logmsg)
        else:
            self.get_logger().error(f'Service call failed: {future.result()}')
        return future.result()
    
    def turn_lights_on(self, level):
        """
        Turn the lights on.

        Args:
            level (int): The level to turn the lights on to. 0 is off, 100 is full
        """
        self.get_logger().info(f"Turning lights on to level {level}")
        commands = OverrideRCIn()
        commands.channels = [OverrideRCIn.CHAN_NOCHANGE] * 10
        commands.channels[8] = 1000 + level * 10
        commands.channels[9] = 1000 + level * 10
        self.command_pub.publish(commands)
    
    def dance(self):
        moves = [
            (0, 0, 0, 0, 0),
            (50, 0, 0,-30,0),
            (50, 0, 0,-30,0),
            (50, 0, 0,-30,0),
            (-50, 0, 0,30,0),
            (-50, 0, 0,30,0),
            (-50, 0, 0,30,0),
        ]
        for x, y, z, r, level in moves:
            self.get_logger().info(f"x: {x} y: {y} z: {z} r: {r}")
            self.publish_manual_control(x ,y, z, r)
            self.turn_lights_on(level)
            time.sleep(1.0)
        self.call_service(False, "Vehicle disarmed.")

def main():
    rclpy.init()
    node = Dance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node closed")
    finally:
        try:
            node.call_service(False, "Vehicle disarmed")
        except Exception as e:
            node.get_logger().error(f'disarm failed: {e}')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()