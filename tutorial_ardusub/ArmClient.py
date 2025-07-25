#/home/eugene/auvc_ws/src/tutorial_ardusub/tutorial_ardusub/ArmClient.py
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import time
class ArmClient(Node):
    def __init__(self):
        super().__init__('arm_client')
        self.cli = self.create_client(SetBool, '/arming')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting /arming ...')
        self.call_service(True, "Vehicle armed")
        #time.sleep(10)
        #self.call_service(False,"Vehicle disarmed")


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


def main():
    rclpy.init()
    node = ArmClient()
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