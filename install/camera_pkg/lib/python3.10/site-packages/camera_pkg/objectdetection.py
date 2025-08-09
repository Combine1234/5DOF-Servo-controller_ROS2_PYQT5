import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import numpy as np

class YoloObjectDetector(Node):
    def __init__(self):
        super().__init__('yolo_object_detector')

        model_path = '/home/monkey/Innovedex_Bot/src/camera_pkg/camera_pkg/last copy.pt'
        try:
            self.model = YOLO(model_path)
            self.get_logger().info(f"✅ Model loaded successfully from: {model_path}")
        except Exception as e:
            self.get_logger().error(f"❌ Error loading YOLO model: {e}")
            self.model = None

        self.bridge = CvBridge()

        # รับภาพจากกล้อง
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.detected_image_publisher = self.create_publisher(Image, '/camera_detected', 10)

    def get_dominant_color(self, roi):
        hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([179, 255, 255])

        lower_green = np.array([40, 40, 40])
        upper_green = np.array([80, 255, 255])

        lower_blue = np.array([100, 50, 50])
        upper_blue = np.array([140, 255, 255])

        mask_red = cv2.inRange(hsv_roi, lower_red1, upper_red1) + cv2.inRange(hsv_roi, lower_red2, upper_red2)
        mask_green = cv2.inRange(hsv_roi, lower_green, upper_green)
        mask_blue = cv2.inRange(hsv_roi, lower_blue, upper_blue)

        counts = {
            'Red': np.sum(mask_red > 0),
            'Green': np.sum(mask_green > 0),
            'Blue': np.sum(mask_blue > 0)
        }
        return max(counts, key=counts.get)

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"❌ Error converting image: {e}")
            return

        if self.model is None:
            return

        results = self.model(frame, verbose=False)[0]
        if len(results.boxes) == 0:
            with open("detected_color.txt", "w") as f:
                f.write("None")
            return

        # ใช้แค่ object แรกที่ detect ได้
        box = results.boxes[0]
        x1, y1, x2, y2 = map(int, box.xyxy[0])
        roi = frame[y1:y2, x1:x2]
        color_label = self.get_dominant_color(roi)

        # 💾 Save ลงไฟล์ทุกครั้งที่ detect
        with open("detected_color.txt", "w") as f:
            f.write(color_label)

        self.get_logger().info(f"💾 Saved detected color: {color_label}")

        # วาดกรอบและสีบนภาพ
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(frame, color_label, (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.detected_image_publisher.publish(img_msg)

def main(args=None):
    rclpy.init(args=args)
    node = YoloObjectDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
