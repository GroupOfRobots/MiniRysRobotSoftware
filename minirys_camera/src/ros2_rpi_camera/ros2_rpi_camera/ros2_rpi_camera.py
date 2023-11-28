from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import Image

import rclpy

from picamera2 import Picamera2
from builtin_interfaces.msg import Time

FRAME_INTERVAL = 0.1
ENCODING = "rgba8"  # http://docs.ros.org/en/jade/api/sensor_msgs/html/image__encodings_8h_source.html

WIDTH_START_PIXEL = 100
WIDTH_END_PIXEL = 400
WIDTH_INTERVAL = 2

HEIGHT_START_PIXEL = 100
HEIGHT_END_PIXEL = 400
HEIGHT_INTERVAL = 2

WIDTH = 640
HEIGHT = 480


class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')

        self.publisher = self.create_publisher(Image, 'video_frames', 10)

        self.picam2 = Picamera2()
        self.picam2.configure(self.picam2.create_video_configuration(main={"size": (HEIGHT, WIDTH)}))

        self.picam2.start()

        self.frame_id = 0
        self.imageMsg = Image()

        self.imageMsg.encoding = ENCODING
        self.imageMsg.width = WIDTH
        self.imageMsg.height = HEIGHT

        self.create_timer(FRAME_INTERVAL, self.image_callback)

    def get_time_msg(self):
        time_msg = Time()
        msg_time = self.get_clock().now().seconds_nanoseconds()
        time_msg.sec = int(msg_time[0])
        time_msg.nanosec = int(msg_time[1])
        return time_msg

    def image_callback(self):
        data = self.picam2.capture_array()
        data = data[WIDTH_START_PIXEL:WIDTH_END_PIXEL:WIDTH_INTERVAL,
               HEIGHT_START_PIXEL:HEIGHT_END_PIXEL:HEIGHT_INTERVAL, :]

        self.imageMsg.width = data.shape[0]
        self.imageMsg.height = data.shape[1]

        self.imageMsg.data = data.flatten().tolist()
        self.imageMsg.header.stamp = self.get_time_msg()

        self.frame_id += 1
        self.imageMsg.header.frame_id = str(self.frame_id)
        self.publisher.publish(self.imageMsg)


def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    rclpy.spin(image_publisher)
    image_publisher.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
