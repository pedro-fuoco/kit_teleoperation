import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2

class CompressedImageViewer(Node):
    def __init__(self):
        super().__init__('kit_teleop_image_viewer')
        self.subscription = self.create_subscription(
            CompressedImage,
            '/kit/camera/image_compressed',
            self.image_callback,
            10
        )
        self.bridge = CvBridge()
        self.get_logger().info("Compressed image viewer has been started.")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')

            cv2.imshow("Compressed camera image", cv_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CompressedImageViewer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
