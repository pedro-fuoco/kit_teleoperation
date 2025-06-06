import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraCalibrator(Node):
    def __init__(self):
        super().__init__('camera_calibrator_intrinsic')
        self.subscription = self.create_subscription(
            CompressedImage,
            '/kit/camera/image_compressed',
            self.image_callback,
            10
        )
        self.bridge = CvBridge()
        self.get_logger().info("Intrinsic Camera calibrator node has started.")
        
        # Calibration settings
        self.chessboard_size = (6, 6)
        self.square_size = 0.0121  # meters
        self.img_points = []
        self.labels = []
        self.paired_imgs = []
        self.obj_points = []

        self.necessary_valid_images = 50
        
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            # Find the chessboard corners
            ret, corners, meta = cv2.findChessboardCornersSBWithMeta(gray, self.chessboard_size, flags= cv2.CALIB_CB_EXHAUSTIVE + cv2.CALIB_CB_LARGER + cv2.CALIB_CB_MARKER)
            
            if ret:
                self.get_logger().info(f"Found a chessboard! {len(self.img_points)+1}/{self.necessary_valid_images}")

                self.img_points.append(corners)
                self.labels.append(meta)
                self.paired_imgs.append(cv_image)
                
                # Draw and display the corners
                cv_image = cv2.drawChessboardCorners(cv_image, self.chessboard_size, corners, ret)

                l, a = meta.shape
                objp = np.zeros((l * a, 3), np.float32)
                for i in range(l * a):
                    x = i // a
                    y = i % a
                    objp[i] = [x * self.square_size, y * self.square_size, 0]

                self.obj_points.append(objp)
                
                # Perform calibration once enough samples are collected
                if len(self.img_points) >= self.necessary_valid_images:  # Use 50 valid images for calibration
                    self.get_logger().info(f"{self.necessary_valid_images} valid images captured. Now running calibration...")
                    self.calibrate_camera(gray.shape[::-1])
                    cv2.destroyAllWindows()
                    rclpy.shutdown()
            else:
                self.get_logger().error("No chessboard found")
            
            # cv2.imshow("Chessboard Detection", cv_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")

    def calibrate_camera(self, image_size):
        ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
            self.obj_points, self.img_points, image_size, None, None, flags= cv2.CALIB_RATIONAL_MODEL + cv2.CALIB_THIN_PRISM_MODEL
        )

        if ret:
            self.get_logger().info("Camera calibration successful.")
            self.get_logger().info(f"Camera Matrix:\n{camera_matrix}")
            self.get_logger().info(f"Distortion Coefficients:\n{dist_coeffs}")

            total_error = 0
            for i in range(len(self.obj_points)):
                imgpoints2, _ = cv2.projectPoints(self.obj_points[i], rvecs[i], tvecs[i], camera_matrix, dist_coeffs)
                error = cv2.norm(self.img_points[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
                total_error += error
            mean_error = total_error/len(self.obj_points)
            self.get_logger().warn(f"Mean error: {mean_error}")
            if mean_error >= 1:
                self.get_logger().error(f"Mean error is very high")
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
