import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np
from rclpy.parameter import ParameterType

class CameraCalibrator(Node):
    def __init__(self):
        super().__init__('camera_calibrator_extrinsic')
        self.subscription = self.create_subscription(
            CompressedImage,
            '/kit/camera/image_compressed',
            self.image_callback,
            10
        )
        self.bridge = CvBridge()
        self.get_logger().info("Extrinsic Camera calibrator node has started.")
        
        # Calibration settings
        self.chessboard_size = (9, 13)
        self.square_size = 0.0121  # meters
        
        self.declare_parameter('calibration_matrix', [0.0] * 9)
        self.declare_parameter('dist_coefficients', [0.0] * 12)

        cam_mat = self.get_parameter('calibration_matrix').value
        dist   = self.get_parameter('dist_coefficients').value

        self.calibration_matrix = np.array(cam_mat, dtype=np.float64).reshape(3,3)
        self.dist_coefficients   = np.array(dist, dtype=np.float64).reshape(-1,1)
        self.get_logger().info(f"Loaded camera_matrix:\n{self.calibration_matrix}")
        self.get_logger().info(f"Loaded dist_coeffs:\n{self.dist_coefficients.ravel()}")


    def image_callback(self, msg):
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            ret, corners, meta = cv2.findChessboardCornersSBWithMeta(gray, self.chessboard_size, flags= cv2.CALIB_CB_EXHAUSTIVE + cv2.CALIB_CB_LARGER + cv2.CALIB_CB_MARKER)
            
            if not ret:
                self.get_logger().error("No chessboard found")
                return
            
            self.get_logger().info(f"Found the chessboard!")

            x_offset = 0.06985 # ~7 centimetros até o inicio do padrão xadrez
            y_offset = self.square_size*4 # 4 quadrados até a coordenada zero em Y

            l, a = meta.shape
            objp = np.zeros((l * a, 3), np.float32) # Pontos detectados no referencial do robô
            for i in range(l * a):
                x = i // a
                y = i % a
                objp[i] = [x_offset + (x * self.square_size), y_offset - (y * self.square_size), 0]

            success, rot, trans = cv2.solvePnP(
                objp,
                corners,
                self.calibration_matrix,
                self.dist_coefficients
            )

            if not success:
                self.get_logger().error("Transformation failed!")
                return

            matriz_rot, jacobiano = cv2.Rodrigues(rot)
            
            self.get_logger().info(f"Translation:\n{trans}\n")
            self.get_logger().info(f"Rotation matrix:\n{matriz_rot}\n")            

            cv2.destroyAllWindows()
            rclpy.shutdown()

        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")

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
