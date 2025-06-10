import rclpy
from builtin_interfaces.msg import Time
from picamera2 import Picamera2
from libcamera import Transform
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

ENCODING = "rgba8"  # http://docs.ros.org/en/jade/api/sensor_msgs/html/image__encodings_8h_source.html

DEFAULT_WIDTH = 640
DEFAULT_HEIGHT = 480

DEFAULT_FRAME_INTERVAL = 0.1


class SimpleImagePublisher(Node):
    bridge = CvBridge()

    def __init__(self):
        super().__init__('image_publisher')

        self.publisher = self.create_publisher(Image, 'internal/camera', 10)

        self.declareParameters()

        self.configure_picamera()

        self.frame_id = 0

        self.create_timer(self.frame_interval, self.image_callback)

    def declareParameters(self):
        self.declare_parameter('width', DEFAULT_WIDTH)
        self.width = self.get_parameter('width').value

        self.declare_parameter('height', DEFAULT_HEIGHT)
        self.height = self.get_parameter('height').value

        self.declare_parameter('frame_interval', DEFAULT_FRAME_INTERVAL)
        self.frame_interval = self.get_parameter('frame_interval').value

    def configure_picamera(self):
        self.picam2 = Picamera2()
        # https://datasheets.raspberrypi.com/camera/picamera2-manual.pdf
        config = self.picam2.create_still_configuration(
            main={
                'size': (2592, 1944),  # full frame
                'size': (1296, 972),  # full frame
                'format': 'RGB888',  # Compatible with OpenCV BGR default encoding
            },
            buffer_count=2,
            queue=True,
            transform=Transform(hflip=True, vflip=True),  # Flip because camera is upside down with LiDAR up
            # lores={"size": (self.width, self.height)},
        )
        # self.picam2.preview_configuration.main.size = (1296, 972)
        # self.picam2.preview_configuration.main.format = 'RGB888'
        # self.picam2.preview_configuration.align()
        # self.picam2.configure('preview')
        self.picam2.align_configuration(config)
        self.picam2.configure(config)
        self.picam2.start()

    def get_time_msg(self):
        time_msg = Time()
        msg_time = self.get_clock().now().seconds_nanoseconds()
        time_msg.sec = int(msg_time[0])
        time_msg.nanosec = int(msg_time[1])
        return time_msg

    def image_callback(self):
        yuv = self.picam2.capture_array('main')

        image = cv2.cvtColor(yuv, cv2.COLOR_YUV420p2RGB)
        image_msg  =self.bridge.cv2_to_imgmsg(image, 'bgr8')

        self.frame_id += 1
        image_msg.header.frame_id = str(self.frame_id)
        self.publisher.publish(image_msg)

def main(args=None):
    rclpy.init(args=args)
    simple_image_publisher = SimpleImagePublisher()
    rclpy.spin(simple_image_publisher)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
