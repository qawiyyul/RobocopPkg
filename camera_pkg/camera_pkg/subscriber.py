import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import time
import numpy as np
model=YOLO("yolov8l.pt")
class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.publisher = self.create_publisher(Int32, 'humans', 10)
        
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        print('image received')
        start_time = time.time()
        #np_arr = np.fromstring(msg.data, np.uint8)
        #img = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
        img = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='passthrough')
        #img = cv2.rotate(img, cv2.ROTATE_180)
        results = model.predict(source=img, classes=0, show=True)
        for result in results:
            print('result')
            print(len(result.boxes))
            self.publish_msg(len(result.boxes))
        end_time=time.time()
        time_taken = end_time-start_time
        print(f"The time taken to execute is {time_taken:.5f} seconds.")
        cv2.imwrite('image.jpg', img)
        
    def publish_msg(self, num):
        msg = Int32()
        msg.data = num
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
