import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class Kinect2ImageSubscriber(Node):
    def __init__(self):
        super().__init__('kinect2_image_subscriber')

        # 初始化CvBridge
        self.bridge = CvBridge()

        # 订阅/kinect2/hd/image_raw话题
        self.subscription = self.create_subscription(
            Image,
            '/kinect2/hd/image_raw',
            self.image_callback,
            10  # 队列大小
        )

    def image_callback(self, msg):
        try:
            # 将ROS图像消息转换为OpenCV图像
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")
            return

        # 显示图像
        cv2.imshow("Kinect2 Image", cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)

    # 创建Kinect2图像订阅节点
    kinect2_image_subscriber = Kinect2ImageSubscriber()

    # 保持节点运行
    rclpy.spin(kinect2_image_subscriber)

    # 销毁节点
    kinect2_image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
