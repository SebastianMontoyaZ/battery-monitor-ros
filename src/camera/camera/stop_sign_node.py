import rclpy
from rclpy.node import Node  # <-- Add this line

from sensor_msgs_py import point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO

class StopSignDetector(Node):
    def __init__(self):
        super().__init__('stop_sign_detector')
        self.bridge = CvBridge()
        self.model = YOLO('/home/seb/workspace/src/camera/models/yolov8n.pt')

        self.image_sub = self.create_subscription(
            Image, '/robot/front_rgbd_camera/color/image_raw', self.image_callback, 10)
        self.point_cloud_sub = self.create_subscription(
            PointCloud2, '/robot/front_rgbd_camera/depth/points', self.point_cloud_callback, 10)

        self.image_pub = self.create_publisher(Image, '/stop_sign/image', 10)
        self.dist_pub = self.create_publisher(Float32, '/stop_sign/distance', 10)

        self.latest_point_cloud = None

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        results = self.model(frame, conf=0.2, iou=0.4)

        stop_sign_distance = None

        for result in results:
            print(f"result {result}\n")
            for box in result.boxes:
                print(f"result {box}\n")
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                label = result.names[int(box.cls[0])]

                if label == "stop sign":
                    center_x, center_y = (x1 + x2) // 2, (y1 + y2) // 2

                    # Draw bounding box
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(frame, "Stop Sign", (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                    # Get depth from point cloud
                    if self.latest_point_cloud is not None:
                        # Use your logic to get depth data from the point cloud here
                        stop_sign_distance = self.get_distance_from_point_cloud(center_x, center_y)

                        if stop_sign_distance:
                            cv2.putText(frame, f"{stop_sign_distance:.2f}m", (x1, y2 + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

        self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, 'bgr8'))

        if stop_sign_distance:
            self.dist_pub.publish(Float32(data=stop_sign_distance))

    def point_cloud_callback(self, msg):
        # Convert PointCloud2 message to usable point cloud data
        pc_data = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        self.latest_point_cloud = list(pc_data)

    def get_distance_from_point_cloud(self, x, y):
        # Assuming you know how to map pixel (x, y) to 3D point in the point cloud.
        # This part requires a depth-to-point cloud mapping, which depends on your camera calibration.
        # You may want to check the calibration of your RealSense to get this working correctly.
        if self.latest_point_cloud:
            # Simple approach: find the nearest point in the cloud and return the distance
            # You can improve this by using the correct mapping from pixel coordinates to the point cloud
            for point in self.latest_point_cloud:
                px, py, pz = point
                # You need to map (x, y) to (px, py, pz)
                distance = np.sqrt(px**2 + py**2 + pz**2)  # Example distance calculation
                return distance

        return None

def main(args=None):
    rclpy.init(args=args)
    node = StopSignDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()