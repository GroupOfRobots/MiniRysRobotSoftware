import rclpy
from builtin_interfaces.msg import Time
from picamera2 import Picamera2
from libcamera import Transform
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

# For debugging slow publishing
PROFILE = False
if PROFILE: from pyinstrument import Profiler

class SimpleImagePublisher(Node):
    bridge = CvBridge()

    low_resolution = (640, 480)  # almost full frame
    high_resolution = (1296, 972)  # full frame
    very_high_resolution = (2592, 1944)  # full frame

    def __init__(self):
        super().__init__('image_publisher')

        self.publisher = self.create_publisher(Image, 'internal/camera', 10)
        self.publisher_lores = self.create_publisher(Image, 'internal/camera_low_res', 10)

        frame_interval_main, frame_interval_lores = self.get_parameters()

        self.configure_picamera()

        self.frame_id = 0
        self.frame_id_lores = 0

        self.timer = self.create_timer(frame_interval_main, self.image_callback)
        self.timer_lores = self.create_timer(frame_interval_lores, self.image_callback_lores)

    def get_parameters(self):
        self.declare_parameter('high_res_frequency', 5.0)
        frame_interval_main = 1.0 / self.get_parameter('high_res_frequency').value

        self.declare_parameter('low_res_frequency', 10.0)
        frame_interval_lores = 1.0 / self.get_parameter('low_res_frequency').value

        return frame_interval_main, frame_interval_lores

    def configure_picamera(self):
        self.picam2 = Picamera2()

        # https://datasheets.raspberrypi.com/camera/picamera2-manual.pdf
        modes = self.picam2.sensor_modes
        sensor_modes_msg = 'Avalable sensor modes are:'
        for mode in modes: sensor_modes_msg += f'\n{mode}'
        self.get_logger().info(sensor_modes_msg)

        def get_hires_mode(modes, desired_resolution):
            for mode in modes:
                if mode['size'] == desired_resolution:
                    return mode
            raise RuntimeError('Could not obtain a high resolution sensor mode')

        mode = get_hires_mode(self.picam2.sensor_modes, self.high_resolution)

        config = self.picam2.create_still_configuration(
            transform=Transform(hflip=True, vflip=True),  # Flip because camera is upside down with LiDAR up
            buffer_count=6,  # The same as in video configuration to be on the safe side
            queue=True,
            sensor={
                'output_size': mode['size'],
                'bit_depth': mode['bit_depth'],
            },
            main={
                'size': self.high_resolution,
                'format': 'RGB888',  # Compatible with OpenCV BGR default encoding
            },
            lores={
                'size': self.low_resolution,
                # 'format': 'RGB888',  # Format is mandatory to be YUV420 on Pi 4
            },
        )

        self.get_logger().info('Requested configurations are:' +
                               f"\nmain:  {config['main']}\nlores: {config['lores']}")
        self.picam2.align_configuration(config)
        self.get_logger().info('Aligned configurations are:' +
                               f"\nmain:  {config['main']}\nlores: {config['lores']}")
        self.picam2.configure(config)

        print(f"Camera controls:\n{self.picam2.camera_controls}")
        self.picam2.set_controls({
            # "AeEnable": False,         # Disable auto-exposure
            # "AwbEnable": False,        # Optionally disable auto white balance
            # "ExposureTime": 1000,        # Use a short exposure time (near the minimum)
            "ExposureValue": -2.0,        # Use a short exposure time (near the minimum)
            # "AnalogueGain": 1.0,       # Use the minimal analogue gain
        })

        self.picam2.start()

    def get_time_msg(self):
        time_msg = Time()
        msg_time = self.get_clock().now().seconds_nanoseconds()
        time_msg.sec = int(msg_time[0])
        time_msg.nanosec = int(msg_time[1])
        return time_msg

    def image_callback(self):
        if PROFILE:
            profiler = Profiler()
            profiler.start(target_description="Main callback")

        image = self.picam2.capture_array('main')
        image_msg = self.bridge.cv2_to_imgmsg(image)

        self.frame_id += 1
        image_msg.header.frame_id = str(self.frame_id)
        self.publisher.publish(image_msg)

        if PROFILE:
            profiler.stop()
            profiler.print()

    def image_callback_lores(self):
        if PROFILE:
            profiler = Profiler()
            profiler.start(target_description="Lores callback")

        yuv = self.picam2.capture_array('lores')
        image = cv2.cvtColor(yuv, cv2.COLOR_YUV420p2RGB)
        image_msg = self.bridge.cv2_to_imgmsg(image)

        self.frame_id_lores += 1
        image_msg.header.frame_id = str(self.frame_id_lores)
        self.publisher_lores.publish(image_msg)

        if PROFILE:
            profiler.stop()
            profiler.print()

def main(args=None):
    rclpy.init(args=args)
    simple_image_publisher = SimpleImagePublisher()
    rclpy.spin(simple_image_publisher)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
