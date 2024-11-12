import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraCalibrator(Node):
    def __init__(self):
        super().__init__('camera_calibrator')
        self.subscription = self.create_subscription(
            CompressedImage,
            '/kit/camera/image_compressed',
            self.image_callback,
            10
        )
        self.bridge = CvBridge()
        self.get_logger().info("Camera calibrator node has started.")
        
        # Calibration settings
        self.chessboard_size = (9, 6)
        self.square_size = 0.022  # meters
        self.obj_points = []  # 3D points in real-world space
        self.img_points = []  # 2D points in image plane
        
        # Prepare the 3D points for the chessboard
        self.objp = np.zeros((self.chessboard_size[0] * self.chessboard_size[1], 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:self.chessboard_size[0], 0:self.chessboard_size[1]].T.reshape(-1, 2)
        self.objp *= self.square_size
        
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            # Find the chessboard corners
            ret, corners = cv2.findChessboardCorners(gray, self.chessboard_size, None)
            
            if ret:
                self.img_points.append(corners)
                self.obj_points.append(self.objp)
                
                # Draw and display the corners
                cv_image = cv2.drawChessboardCorners(cv_image, self.chessboard_size, corners, ret)
                
                # Perform calibration if enough samples are collected
                if len(self.img_points) >= 100:  # Use 100 valid images for calibration
                    self.calibrate_camera(gray.shape[::-1])
                    cv2.destroyAllWindows()
                    rclpy.shutdown()
            
            cv2.imshow("Chessboard Detection", cv_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")

    def calibrate_camera(self, image_size):
        ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
            self.obj_points, self.img_points, image_size, None, None
        )

        if ret:
            self.get_logger().info("Camera calibration successful.")
            self.get_logger().info(f"Camera Matrix:\n{camera_matrix}")
            self.get_logger().info(f"Distortion Coefficients:\n{dist_coeffs}")
        else:
            self.get_logger().error("Camera calibration failed.")

        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = CameraCalibrator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
