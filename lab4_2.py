import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CannyEdgeDetectionNode(Node):
    """
    A node for performing Canny Edge Detection on an input image.
    """
    def __init__(self):
        super().__init__('canny_edge_detection')
        self.get_logger().info("Initializing Canny Edge Detection Node...")
        self.declare_parameter('lower_threshold', 50)
        self.declare_parameter('upper_threshold', 150)

        self.subscription = self.create_subscription(
            Image,
            "image_raw",
            self.image_callback,
            10)
        self.subscription

        self.publisher = self.create_publisher(
            Image,
            'output_edges',
            10)

        self.bridge = CvBridge()

    def image_callback(self, msg):
        lower_threshold = self.get_parameter('lower_threshold').get_parameter_value().integer_value
        upper_threshold = self.get_parameter('upper_threshold').get_parameter_value().integer_value
        edges = cv2.Canny(cv_image, lower_threshold, upper_threshold)
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            edges = cv2.Canny(cv_image, 200, 100)  # Note: The order here is reversed from your description

            edge_msg = self.bridge.cv2_to_imgmsg(edges, "mono8")
            self.publisher.publish(edge_msg)
        except Exception as e:
            self.get_logger().error('Failed to convert image: %s' % str(e))
            return

def main(args=None):
    rclpy.init(args=args)
    canny_edge_detection_node = CannyEdgeDetectionNode()
    rclpy.spin(canny_edge_detection_node)
    canny_edge_detection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()