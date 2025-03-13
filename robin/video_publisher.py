import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge
import base64

IMAGE_SUBSCRIBE_TOPIC_NAME = '/camera/color/image_raw'

class VideoPublisher(Node):
    def __init__(self):
        super().__init__('video_publisher')

        # Image Subscriber
        self.image_subscriber = self.create_subscription(
            Image,
            IMAGE_SUBSCRIBE_TOPIC_NAME,
            self.image_callback,
            10
        )

        # Image Publisher
        self.base64_publisher = self.create_publisher(String, '/camera/color/image_base64', 10)

        # Initialize CvBridge
        self.bridge = CvBridge()
        print("Video Publisher Node Initialized")

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")
            return

        # Convert OpenCV image to JPEG
        ret, jpeg_image = cv2.imencode('.jpg', cv_image)
        if not ret:
            self.get_logger().error("Failed to encode image to JPEG")
            return

        # Encode the JPEG image to base64
        jpeg_bytes = jpeg_image.tobytes()
        encoded_string = base64.b64encode(jpeg_bytes).decode('utf-8')

        # Publish the base64-encoded string
        base64_msg = String()
        base64_msg.data = str(msg.width) + ":" + str(msg.height) + ":" + encoded_string
        #base64_msg.data = encoded_string
        self.base64_publisher.publish(base64_msg)

        self.get_logger().info("Published Base64 encoded image")


def main(args=None):
    rclpy.init(args=args)

    # Create and spin the node
    video_publisher = VideoPublisher()
    rclpy.spin(video_publisher)

    # Shutdown the node
    video_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()