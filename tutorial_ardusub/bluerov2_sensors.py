#/home/eugene/auvc_ws/src/tutorial_ardusub/tutorial_ardusub/bluerov2_sensors.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, Imu

class BlueRov2Sensors(Node):
    def __init__(self):
        super().__init__("bluerov2_sensor")
        
        self.Battery_voltage = 0
        self.Battery_percentage = 0
        self.Battery_low_voltage_thresold = 13.2

        self.BatteryS = self.create_subscription(
            BatteryState,        # the message type
            "/battery_state",    # the topic name,
            self.battery_callback,  # the subscription's callback method
            10              # QOS (will be covered later)
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/imu',  # MAVROS topic
            self.imu_callback,
            10
        )
        self.get_logger().info("initialized subscriber node")


    def battery_callback(self,msg):
        self.Battery_voltage = msg.voltage
        self.Battery_percentage = msg.percentage *100

        if self.Battery_voltage > self.Battery_low_voltage_thresold:
            self.get_logger().info(f"Battery voltage: {self.Battery_voltage}",throttle_duration_sec=1)
            self.get_logger().info(f"Battery percentage: {self.Battery_percentage}",throttle_duration_sec=1)
        else:
            self.get_logger().warn(f"Battery voltage low!!!: {self.Battery_voltage}",throttle_duration_sec=1)
            self.get_logger().info(f"Battery percentage: {self.Battery_percentage}",throttle_duration_sec=1)
    
    def imu_callback(self, msg):
        self.imu_la = msg.orientation
        self.get_logger().info(
            f"IMU Orientation - X: {msg.orientation.x:.3f}, "
            f"Y: {msg.orientation.y:.3f}, Z: {msg.orientation.z:.3f}, "
            f"Angular Velocity - X: {msg.angular_velocity.x:.3f}, "
            f"Y: {msg.angular_velocity.y:.3f}, Z: {msg.angular_velocity.z:.3f}, "
            f"Linear Acceleration - X: {msg.linear_acceleration.x:.3f}, "
            f"Y: {msg.linear_acceleration.y:.3f}, Z: {msg.linear_acceleration.z:.3f}",
            throttle_duration_sec=1
        )

def main(args=None):
    rclpy.init(args=args)
    node = BlueRov2Sensors()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node closed")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()