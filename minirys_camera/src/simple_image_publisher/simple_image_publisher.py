import rclpy
from rclpy.node import Node  # Handles the creation of nodes
from rclpy.qos import QoSProfile, qos_profile_sensor_data  # QoS configurations
from rclpy.impl.logging_severity import LoggingSeverity

from sensor_msgs.msg import Image
from std_msgs.msg import Header

from picamera2 import Picamera2
from libcamera import Transform

import os
import cv2
from cv_bridge import CvBridge
import numpy as np

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

        self.declare_parameter('high_res_frequency', 5.0  )
        self.declare_parameter('low_res_frequency',  20.0 )
        self.declare_parameter('exposure_value',     -2.0 )
        self.declare_parameter('flip_image',         True )
        self.declare_parameter('debug',              False)
        self.declare_parameter('enable_profiling',   False)

        self.declare_parameter('high_res_crop_factor_top',    0.0)
        self.declare_parameter('high_res_crop_factor_bottom', 0.0)
        self.declare_parameter('high_res_crop_factor_left',   0.0)
        self.declare_parameter('high_res_crop_factor_right',  0.0)

        self.declare_parameter('low_res_crop_factor_top',    0.0)
        self.declare_parameter('low_res_crop_factor_bottom', 0.0)
        self.declare_parameter('low_res_crop_factor_left',   0.0)
        self.declare_parameter('low_res_crop_factor_right',  0.0)

        high_res_frequency = self.get_parameter('high_res_frequency').value
        low_res_frequency  = self.get_parameter('low_res_frequency' ).value
        exposure_value     = self.get_parameter('exposure_value'    ).value
        flip_image         = self.get_parameter('flip_image'        ).value
        debug              = self.get_parameter('debug'             ).value
        enable_profiling   = self.get_parameter('enable_profiling'  ).value

        high_res_crop_factor_top    = self.get_parameter('high_res_crop_factor_top'   ).value
        high_res_crop_factor_bottom = self.get_parameter('high_res_crop_factor_bottom').value
        high_res_crop_factor_left   = self.get_parameter('high_res_crop_factor_left'  ).value
        high_res_crop_factor_right  = self.get_parameter('high_res_crop_factor_right' ).value

        low_res_crop_factor_top     = self.get_parameter('low_res_crop_factor_top'   ).value
        low_res_crop_factor_bottom  = self.get_parameter('low_res_crop_factor_bottom').value
        low_res_crop_factor_left    = self.get_parameter('low_res_crop_factor_left'  ).value
        low_res_crop_factor_right   = self.get_parameter('low_res_crop_factor_right' ).value

        def verify_crop_factor(value: float, name: str):
            if value < 0.0 or 1.0 < value:
                raise RuntimeError(f'Variable {name} = {value} is outside allowed bounds: [0.0; 1.0]')

        verify_crop_factor(high_res_crop_factor_top,    'high_res_crop_factor_top'   )
        verify_crop_factor(high_res_crop_factor_bottom, 'high_res_crop_factor_bottom')
        verify_crop_factor(high_res_crop_factor_left,   'high_res_crop_factor_left'  )
        verify_crop_factor(high_res_crop_factor_right,  'high_res_crop_factor_right' )

        verify_crop_factor(low_res_crop_factor_top,    'low_res_crop_factor_top'   )
        verify_crop_factor(low_res_crop_factor_bottom, 'low_res_crop_factor_bottom')
        verify_crop_factor(low_res_crop_factor_left,   'low_res_crop_factor_left'  )
        verify_crop_factor(low_res_crop_factor_right,  'low_res_crop_factor_right' )

        self.get_logger().info(f'Got parameter: high_res_frequency := {high_res_frequency}')
        self.get_logger().info(f'Got parameter: low_res_frequency  := {low_res_frequency}' )
        self.get_logger().info(f'Got parameter: exposure_value     := {exposure_value}'    )
        self.get_logger().info(f'Got parameter: flip_image         := {flip_image}'        )
        self.get_logger().info(f'Got parameter: debug              := {debug}'             )
        self.get_logger().info(f'Got parameter: enable_profiling   := {enable_profiling}'  )

        self.get_logger().info(f'Got parameter: high_res_crop_factor_top    := {high_res_crop_factor_top}'   )
        self.get_logger().info(f'Got parameter: high_res_crop_factor_bottom := {high_res_crop_factor_bottom}')
        self.get_logger().info(f'Got parameter: high_res_crop_factor_left   := {high_res_crop_factor_left}'  )
        self.get_logger().info(f'Got parameter: high_res_crop_factor_right  := {high_res_crop_factor_right}' )

        self.get_logger().info(f'Got parameter: low_res_crop_factor_top     := {low_res_crop_factor_top}'   )
        self.get_logger().info(f'Got parameter: low_res_crop_factor_bottom  := {low_res_crop_factor_bottom}')
        self.get_logger().info(f'Got parameter: low_res_crop_factor_left    := {low_res_crop_factor_left}'  )
        self.get_logger().info(f'Got parameter: low_res_crop_factor_right   := {low_res_crop_factor_right}' )

        if debug:
            self.get_logger().set_level(LoggingSeverity.DEBUG)

        self.publisher_high_res = self.create_publisher(Image, '~/output/camera',         qos_profile=qos_profile_sensor_data)
        self.publisher_low_res  = self.create_publisher(Image, '~/output/camera_low_res', qos_profile=qos_profile_sensor_data)

        main_size, lores_size = self.configure_picamera(exposure_value, flip_image)
        # Flip (width, height) -> (height, width) so that the size is consistent with OpenCV
        main_size[0], main_size[1] = main_size[1], main_size[0]
        lores_size[0], lores_size[1] = lores_size[1], lores_size[0]

        self.frame_id = os.path.join(self.get_namespace(), 'camera')

        self.timer_high_res = self.create_timer((1.0 / high_res_frequency), self.image_callback_high_res)
        self.timer_low_res  = self.create_timer((1.0 / low_res_frequency ), self.image_callback_low_res )

        self.high_res_crop_idx_top    = int(main_size[0] * high_res_crop_factor_top)
        self.high_res_crop_idx_bottom = int(main_size[0] * (1.0 - high_res_crop_factor_bottom))
        self.high_res_crop_idx_left   = int(main_size[1] * high_res_crop_factor_left)
        self.high_res_crop_idx_right  = int(main_size[1] * (1.0 - high_res_crop_factor_right))

        self.low_res_crop_idx_top    = int(lores_size[0] * low_res_crop_factor_top)
        self.low_res_crop_idx_bottom = int(lores_size[0] * (1.0 - low_res_crop_factor_bottom))
        self.low_res_crop_idx_left   = int(lores_size[1] * low_res_crop_factor_left)
        self.low_res_crop_idx_right  = int(lores_size[1] * (1.0 - low_res_crop_factor_right))

        main_size_cropped = self.crop_image(np.ones(main_size),
                                            self.high_res_crop_idx_top,
                                            self.high_res_crop_idx_bottom,
                                            self.high_res_crop_idx_left,
                                            self.high_res_crop_idx_right).shape

        lores_size_cropped = self.crop_image(np.ones(lores_size),
                                             self.low_res_crop_idx_top,
                                             self.low_res_crop_idx_bottom,
                                             self.low_res_crop_idx_left,
                                             self.low_res_crop_idx_right).shape

        self.get_logger().info(f'Capturing main image of size {main_size}'
                               + f' and publishing it cropped to size {main_size_cropped} on topic'
                               + f' "{self.publisher_high_res.topic_name}"'
                               + f' with frequency {high_res_frequency} Hz')
        self.get_logger().info(f'Capturing lores image of size {lores_size}'
                               + f' and publishing it cropped to size {lores_size_cropped} on topic'
                               + f' "{self.publisher_low_res.topic_name}"'
                               + f' with frequency {low_res_frequency} Hz')

    def configure_picamera(self, exposure_value: float, flip_image: bool):
        self.picam2 = Picamera2()

        # https://datasheets.raspberrypi.com/camera/picamera2-manual.pdf
        modes = self.picam2.sensor_modes
        sensor_modes_msg = 'Available sensor modes are:'
        for mode in modes: sensor_modes_msg += f'\n{mode}'
        self.get_logger().debug(sensor_modes_msg)

        def get_hires_mode(modes, desired_resolution):
            for mode in modes:
                if mode['size'] == desired_resolution:
                    return mode
            raise RuntimeError('Could not obtain a high resolution sensor mode')

        mode = get_hires_mode(self.picam2.sensor_modes, self.high_resolution)

        config = self.picam2.create_still_configuration(
            transform=Transform(hflip=flip_image, vflip=flip_image),  # Camera is upside down with LiDAR up
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
                # 'format': 'RGB888',  # Format is mandatory to be YUV420 on Pi 4 in lores stream
            },
        )

        self.get_logger().debug('Requested configurations are:' +
                                f"\nmain:  {config['main']}\nlores: {config['lores']}")
        self.picam2.align_configuration(config)
        self.get_logger().debug('Aligned configurations are:' +
                                f"\nmain:  {config['main']}\nlores: {config['lores']}")
        self.picam2.configure(config)

        self.get_logger().debug(f"Camera controls:\n{self.picam2.camera_controls}")
        self.picam2.set_controls({
            # "AeEnable": False,
            # "AwbEnable": False,
            # "ExposureTime": 1000,
            "ExposureValue": exposure_value,
            # "AnalogueGain": 1.0,
        })

        self.picam2.start()

        return config['main']['size'], config['lores']['size']

    def crop_image(self,
                   image: np.ndarray,
                   crop_idx_top: int,
                   crop_idx_bottom: int,
                   crop_idx_left: int,
                   crop_idx_right: int) -> np.ndarray:
        image_cropped = image[
            crop_idx_top:crop_idx_bottom,
            crop_idx_left:crop_idx_right
        ]
        self.get_logger().debug(f'Received image shape {image.shape}')
        self.get_logger().debug(f'Cropped image shape {image_cropped.shape}')
        self.get_logger().debug(f'Received crop indices: crop_idx_top:={crop_idx_top}, crop_idx_bottom:={crop_idx_bottom}, crop_idx_left:={crop_idx_left}, crop_idx_right:={crop_idx_right}')
        return image_cropped

    def image_callback_high_res(self):
        if PROFILE:
            profiler = Profiler()
            profiler.start(target_description="Main callback")

        image = self.picam2.capture_array('main')
        image = self.crop_image(image,
                                self.high_res_crop_idx_top,
                                self.high_res_crop_idx_bottom,
                                self.high_res_crop_idx_left,
                                self.high_res_crop_idx_right)

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id

        image_msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8', header=header)

        self.publisher_high_res.publish(image_msg)

        if PROFILE:
            profiler.stop()
            profiler.print()

    def image_callback_low_res(self):
        if PROFILE:
            profiler = Profiler()
            profiler.start(target_description="Lores callback")

        yuv = self.picam2.capture_array('lores')
        image = cv2.cvtColor(yuv, cv2.COLOR_YUV420p2RGB)
        image = self.crop_image(image,
                                self.low_res_crop_idx_top,
                                self.low_res_crop_idx_bottom,
                                self.low_res_crop_idx_left,
                                self.low_res_crop_idx_right)

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id

        image_msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8', header=header)

        self.publisher_low_res.publish(image_msg)

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
