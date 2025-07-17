import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from mavros_msgs.msg import OverrideRCIn, ManualControl

class Dance(Node):
    def __init__(self):
        super().__init__('arm_client')
        self.cli = self.create_client(SetBool, '/arming')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting /arming ...')
        
        self.pub = self.create_publisher(ManualControl, "/manual_control", 10)
        self.command_pub = self.create_publisher(OverrideRCIn, "override_rc", 10)
        self.call_service(True, "Vehicle armed")
        
        self.move_index = 0
        self.moves = [
            (0, 0, 0, 0, 0, 1),
            (-20, 0, -40, 0, 0, 1),
            (-20, 0, -40, 0, 0, 1),
            (-20, 0, -40, 0, 0, 1),
            (-20, 0, -40, 0, 0, 1),
            (-20, 0, -40, 0, 0, 1),
            #5
            (-20, 0, -40, 0, 0, 1),
            (-20, 0, -40, 0, 0, 1),
            (20, 0, 40, 0, 0, 1),
            (20, 0, 40, 0, 0, 1),
            (20, 0, 40, 0, 0, 1),
            #10
            (20, 0, 40, 0, 0, 1),
            (20, 0, 40, 0, 0, 1),
            (20, 0, 40, 0, 0, 1),
            (20, 0, 40, 0, 0, 1),
            (40, 20, 0, 0, 0, 1),
            #15
            (40, 20, 0, 0, 0, 1),
            (40, -30, 0, 0, 0, 1),
            (40, -30, 0, 0, 0, 1),
            (40, 20, 0, 0, 0, 1),
            (40, 20, 0, 0, 0, 1),
            #20
            (40, -30, 0, 0, 0, 1),
            (40, -30, 0, 0, 0, 1),
            (40, 20, 0, 0, 0, 1),
            (40, 20, 0, 0, 0, 1),
            (40, -30, 0, 0, 0, 1),
            #25
            (40, -30, 0, 0, 0, 1),
            (40, 20, 0, 0, 0, 1),
            (0, 0, 0, 0, 0, 1),
            (-50, 20, 0, 0, 0, 1),
            (-50, 0, 0, 0, 0, 1),
            #30
            (-50, 0, 0, 0, 0, 1),
            (-50, 0, 0, 0, 0, 1),
            (-50, 0, 0, 0, 0, 1),
            (-50, 0, 0, 0, 0, 1),
            (-50, 0, 0, 0, 0, 1),
            #35
            (-50, 0, 0, 0, 0, 1),
            (-50, 0, 0, 0, 0, 1),
            (-50, 0, 0, 0, 0, 1),
            (-50, 0, 0, 0, 0, 1),
            (-50, 0, 0, 0, 0, 1),
            #40
            (-50, 0, 0, 0, 0, 1),
            (-50, 0, 0, 0, 0, 1),
            (0, -25, 0, 0, 0, 1),
            (0, -25, 0, 0, 0, 1),
            (0, -25, 0, 0, 0, 1),
            #45
            (0, -25, 0, 0, 0, 1),
            (0, -25, 0, 0, 0, 1),
            (0, 0, 0, 0, 60, 1),
            (0, -60, 0, 0, 0, 1),
            (0, -60, 0, 0, 0, 1),
            #50
            (0, 0, 0, 0, 60, 1),
            (0, -25, 0, 0, 0, 1),
            (0, -25, 0, 0, 0, 1),
            (0, -25, 0, 0, 0, 1),
            (0, 0, 0, 0, 100, 1),
            #55
            (0, 60, 0, 0, 0, 1),
            (0, 60, 0, 0, 0, 1),
            (0, 0, 0, 0, 60, 1),
            (0, 60, 0, 0, 0, 1),
            (-50, 0, 0, 0, 0, 1),
            #60
            (0, -60, 0, 0, 0, 1),
            (70, 0, 0, 0, 0, 1),
            (0, 50, 0, 0, 0, 1),
            (30, 0, 0, 0, 0, 1),
            (0, 0, 0, 0, 0, 1),
            #1:05
            (0, 0, 0, 0, 0, 1),
            (0, 0, 0, 0, 60, 1),
            (50, 0, 0, 50, 0, 1),
            (-50, 0, 0, 50, 60, 1),
            (50, 0, 0, 50, 0, 1),
            #1:10
            (-50, 0, 0, 50, 60, 1),
            (50, 0, 0, 50, 0, 1),
            (-50, 0, 0, 50, 60, 1),
            (50, 0, 0, 0, 0, 1),
        ]
        
        self.dance()

    def publish_manual_control(self, x, y, z, r):
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
        def timer_callback():
            nonlocal move_index, publish_count, max_publishes
            if move_index < len(self.moves):
                x, y, z, r, level, t = self.moves[move_index]
                if publish_count == 0:
                    self.get_logger().info(f"x: {x} y: {y} z: {z} r: {r}")
                self.publish_manual_control(x, y, z, r)
                self.turn_lights_on(level)
                publish_count += 1
                if publish_count >= max_publishes:
                    move_index += 1
                    publish_count = 0
                    max_publishes = int(self.moves[move_index][5] * 10) if move_index < len(self.moves) else 0
            else:
                self.call_service(False, "Vehicle disarmed.")
                self.timer.destroy()
                self.get_logger().info("Dance sequence completed.")

        move_index = 0
        publish_count = 0
        max_publishes = int(self.moves[0][5] * 10)  # 初始 t * 10
        self.timer = self.create_timer(0.1, timer_callback)  # 10 Hz = 0.1 秒

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