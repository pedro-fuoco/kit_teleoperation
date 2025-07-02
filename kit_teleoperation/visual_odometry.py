import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from tf_transformations import quaternion_from_euler
from cv_bridge import CvBridge
import math

class ProcessamentoVisao:
    def __init__(self, calib_data=None, logger=None):
        self.detector = cv2.xfeatures2d.SURF_create()
        self.detector.setHessianThreshold(5000)
        self.matcher = cv2.BFMatcher()
        self.kp_anterior = None
        self.desc_anterior = None
        self.logger = logger

        self.calib_data = calib_data
        self.remap_ready = False


        if calib_data:
            P = np.array([
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0]
            ])

            G = np.eye(4)
            G[:3, :3] = calib_data['rotation']
            G[:3, 3] = (calib_data['translation']*1000).ravel() # Convert from meters to milimeters
            N = P @ G
            Nz = np.delete(N, 2, axis=1)
            Q = np.linalg.inv(Nz)
            S = Q / Q[2,2]

            self.pixels_per_mm = 4  # pixels per mm
            shift_y_mm = -105 # offset in mm

            mtx2 = np.float32([
                [self.pixels_per_mm,    0,      0],       
                [0,     self.pixels_per_mm, -self.pixels_per_mm * shift_y_mm],
                [0,        0,      1]
            ])

            try:
                self.map1, self.map2 = cv2.initUndistortRectifyMap(
                    calib_data['mtx'],
                    calib_data['dist'],
                    S,
                    mtx2,
                    (self.pixels_per_mm*297, self.pixels_per_mm*210),
                    cv2.CV_32FC1
                )

                self.remap_ready = True
            except Exception as e:
                if self.logger != None:
                    self.logger.error("Failed to init remap")

    def undistort(self, frame):
        if self.remap_ready:
            return cv2.remap(
                frame,
                self.map1,
                self.map2,
                interpolation=cv2.INTER_LINEAR,
                borderMode=cv2.BORDER_CONSTANT
            )
        else:
            return frame
        
    
    def store_previous_frame_information(self, keypoints, descriptions, quadro):
        self.kp_anterior = keypoints
        self.desc_anterior = descriptions
        self.quadro_anterior = quadro.copy()

    def primeiroQuadro(self, quadro):
        self.mascara = quadro.copy()
        self.mascara[:-self.mascara.shape[0]//4,:] = 255
        _, self.mascara = cv2.threshold(self.mascara, 50, 255, cv2.THRESH_BINARY)

        kernel = np.ones((5,5), np.uint8)
        self.mascara = cv2.morphologyEx(self.mascara, cv2.MORPH_CLOSE, kernel)
        self.mascara = cv2.erode(self.mascara, kernel, iterations=2)
        # cv2.imwrite("mascara.png", self.mascara)
        self.mascara = self.undistort(self.mascara)
        self.mascara = cv2.erode(self.mascara, kernel, iterations=2)
        # cv2.imwrite("mascara_undistorted.png", self.mascara)


        # cv2.imwrite('first_quadro.png', quadro)
        quadro = self.undistort(quadro)
        # cv2.imwrite('first_undistorted.png', quadro)
        kp_atual, desc_atual = self.detector.detectAndCompute(quadro, self.mascara)
        self.store_previous_frame_information(kp_atual, desc_atual, quadro)

        # debug_deteccao = cv2.drawKeypoints(quadro, self.kp_anterior, None, (255,0,0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        # cv2.imwrite("first_debug_deteccao.png", debug_deteccao)


    def estimaMovimento(self, quadro):
        # cv2.imwrite('quadro.png', quadro)
        quadro = self.undistort(quadro)
        # cv2.imwrite('undistorted.png', quadro)
        kp_atual, desc_atual = self.detector.detectAndCompute(quadro, self.mascara)
        debug_deteccao = cv2.drawKeypoints(quadro, kp_atual, None, (255,0,0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        # cv2.imwrite("debug_deteccao.png", debug_deteccao)

        if self.desc_anterior is None or desc_atual is None:
            self.store_previous_frame_information(kp_atual, desc_atual, quadro)
            if self.logger != None:
                self.logger.error("No descriptors found, cant execute comparisons")
            return 0.0, 0.0, 0.0

        matches = self.matcher.knnMatch(self.desc_anterior, desc_atual, k=2)
        good_matches = []
        for m,n in matches:
            if m.distance < 0.7*n.distance:
                good_matches.append(m)

        # imagem = quadro.copy()
        # imagem = cv2.drawMatches(
        #     self.quadro_anterior,
        #     self.kp_anterior,
        #     quadro,
        #     kp_atual,
        #     good_matches,
        #     imagem)
        # cv2.imwrite("matches.png", imagem)

        if len(good_matches) < 8:
            if self.logger != None:
                self.logger.error(f"Not enough good matches (lowe criteria): {len(good_matches)}")
            self.store_previous_frame_information(kp_atual, desc_atual, quadro)
            return 0.0, 0.0, 0.0
        
        pts_prev = np.float32([self.kp_anterior[m.queryIdx].pt for m in good_matches]).reshape(-1,1,2)
        pts_curr = np.float32([kp_atual[m.trainIdx].pt for m in good_matches]).reshape(-1,1,2)

        M, inliers = cv2.estimateAffinePartial2D(
            pts_prev,
            pts_curr,
        )

        dx, dy = M[0,2], M[1,2]
        a, b = M[0,0], M[0,1]
        c, d = M[1,0], M[1,1]
        scale = math.hypot(a, c)
        angle_radians = math.atan2(c, d)

        self.store_previous_frame_information(kp_atual, desc_atual, quadro)
        return -dx/(self.pixels_per_mm*1000),-dy/(self.pixels_per_mm*1000),-angle_radians


class CameraOdometry(Node):
    def __init__(self):
        super().__init__('camera_odometry')
        self.subscription = self.create_subscription(
            CompressedImage,
            '/kit/camera/image_compressed',
            self.image_callback,
            10
        )
        self.pub = self.create_publisher(Odometry, '/kit/vis_odom', 10)

        self.bridge = CvBridge()
        self.get_logger().info("Camera odometry node has started.")

        self.accumulated_x = 0
        self.accumulated_y = 0
        self.accumulated_theta = 0

        self.first_frame = True

        calib_data = {
            'mtx': np.array([[732.87571529, 0.0, 595.93731223],
                             [0.0, 772.40087734, 787.15631937],
                             [0.0,   0.0,   1.0]], dtype=np.float64),
            'dist': np.array([-0.12409884, -0.07985273, 0.00954864, -0.00082085, -0.07158617, 0.15834334, -0.09910482, -0.1406988,  0.02964974, -0.00961546, -0.11609949, 0.02394629], dtype=np.float64),
            'rotation' : np.array([[0.00782286, 0.99980729, -0.01800502],
                                   [-0.8295578, 0.01654267, 0.55817577],
                                   [0.55836606, 0.01056967, 0.82952735]
                                ], dtype=np.float64),
            'translation' : np.array([[-0.00728118],
                                      [0.03464676],
                                      [0.03513033]
                                ], dtype=np.float64)
        }

        self.image_processing = ProcessamentoVisao(
            calib_data=calib_data,
            logger=self.get_logger()
        )

    def wrap_angle(self, angle: float) -> float:
        return (angle + math.pi) % (2*math.pi) - math.pi

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')

            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            if self.first_frame:
                self.first_frame = False
                self.image_processing.primeiroQuadro(gray)
                return

            dx, dy, theta = self.image_processing.estimaMovimento(gray)

            self.accumulated_x = self.accumulated_x + dx
            self.accumulated_y = self.accumulated_y + dy
            self.accumulated_theta = self.wrap_angle(self.accumulated_theta + theta)

            self.get_logger().info(f"\ndelta = ({dx},{dy},{theta})\naccumulated = ({self.accumulated_x},{self.accumulated_y},{self.accumulated_theta})\n")

            odom = Odometry()
            odom.header.stamp = self.get_clock().now().to_msg()
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_link'

            odom.pose.pose.position.x = self.accumulated_x
            odom.pose.pose.position.y = self.accumulated_y
            odom.pose.pose.position.z = 0.0

            q = quaternion_from_euler(0.0, 0.0, self.accumulated_theta)
            odom.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
            self.pub.publish(odom)

        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CameraOdometry()

    try:
        rclpy.spin(node)
    except Exception as e:
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
