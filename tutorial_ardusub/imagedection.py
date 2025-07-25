import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from lane_detection import cut_top_half, detect_lines, draw_lines, detect_lanes, draw_lanes, get_lane_center, recommend_direction, recommend_turn
import numpy as np
import cv2
import os

class ImageDetection(Node):
    def __init__(self):
        super().__init__("image_detection")

        self.bridge = CvBridge()

        self.sub = self.create_subscription(
            Image,
            "bluerov2/camera",
            self.getLane,
            10
        )

        self.save_dir = "saved_images"
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)

        self.get_logger().info(f"Initialized subscriber node. Images will be saved to {self.save_dir}")

    def getLane(self, msg):
        try:

            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.get_logger().debug(f"Received image with frame_id: {msg.header.frame_id}")

            img = cut_top_half(cv_image)

            filtered_lines, edges = detect_lines(img, 20, 60, 3,250,100)

            lanes = detect_lanes(img, filtered_lines)

            mid_intercept, mid_slope, mid_angle = get_lane_center(lanes, img)

            direction = recommend_direction(mid_intercept, mid_slope, mid_angle, img)
            turn = recommend_turn(mid_intercept, mid_slope, mid_angle, img)
            self.get_logger().info(f"Recommended direction: {direction}, Turn: {turn}")

            img_with_lines = draw_lanes(img, lanes)

            frame_id = msg.header.frame_id
            original_path = os.path.join(self.save_dir, f"original_{frame_id}.jpg")
            edges_path = os.path.join(self.save_dir, f"edges_{frame_id}.jpg")
            lanes_path = os.path.join(self.save_dir, f"lanes_{frame_id}.jpg")

            cv2.imwrite(original_path, cv_image)
            cv2.imwrite(edges_path, edges)
            cv2.imwrite(lanes_path, img_with_lines)

        except Exception as e:
            self.get_logger().error(f"Error processing or saving image: {str(e)}")

    def destroy_node(self):
        # 清理 OpenCV 窗口
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ImageDetection()
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