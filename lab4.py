import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from cv_bridge import CvBridge
import cv2

class GaussianBlurNode(Node):
    """
    A node for blurring an image, with dynamically adjustable blur kernel size.
    """
    def __init__(self):
        """
        Constructs all the necessary attributes for the Gaussian blur node,
        including dynamic parameters for the blur kernel size.
        """
        super().__init__('gaussian_blur')
        self.get_logger().info("Initializing Gaussian Blur Node with dynamic blur kernel size...")

        # Define a parameter for the Gaussian blur kernel size with a default value of 5
        blur_radius_descriptor = ParameterDescriptor(description='Size of the Gaussian blur kernel',
                                                     type=ParameterType.PARAMETER_INTEGER)
        self.declare_parameter('blur_radius', 5, blur_radius_descriptor)

        # Subscribe to an image topic
        self.subscription = self.create_subscription(
            Image,
            "image_raw",
            self.image_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Publish the blurred image
        self.publisher = self.create_publisher(
            Image,
            'blurred_image',
            10)

        # Initialize CVBridge
        self.bridge = CvBridge()

    def image_callback(self, msg):
        """
        Callback function for input image topic.
        Applies Gaussian blur to the received image using the dynamically adjustable kernel size and publishes it.
        """
        # Retrieve the current blur radius parameter
        blur_radius = self.get_parameter('blur_radius').get_parameter_value().integer_value
        kernel_size = (blur_radius, blur_radius) if blur_radius > 0 else (1, 1)  # Ensure kernel size is at least 1x1

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            blurred_image = cv2.GaussianBlur(cv_image, kernel_size, 0)

            # Convert back to ROS Image message
            blurred_msg = self.bridge.cv2_to_imgmsg(blurred_image, "bgr8")
            self.publisher.publish(blurred_msg)
        except Exception as e:
            self.get_logger().error('Failed in image processing: %s' % str(e))
            return

# Initialize the node
def main(args=None):
    rclpy.init(args=args)
    gaussian_blur_node = GaussianBlurNode()
    rclpy.spin(gaussian_blur_node)
    gaussian_blur_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


