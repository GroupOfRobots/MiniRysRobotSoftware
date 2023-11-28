import os

import rclpy
from minirys_msgs.srv import RecordVideoStart, RecordVideoStop
from picamera2 import Picamera2
from picamera2.encoders import MJPEGEncoder, Quality
from picamera2.outputs import FfmpegOutput
from rclpy.node import Node  # Handles the creation of nodes


class VideoRecorder(Node):
    def __init__(self):
        super().__init__("video_recorder")

        self.encoder = MJPEGEncoder()
        self.output = FfmpegOutput("video.mp4")

        self.picam2 = Picamera2()

        self.create_service(RecordVideoStop, 'stop_video_recording', self.stop_video_recording)
        self.create_service(RecordVideoStart, 'start_video_recording', self.start_video_recording)

    def start_video_recording(self, request, response):
        if self.picam2.started:
            response.started = False
            response.message = "Camera is already working, can not start recording"
            return response
        print(request)
        width = request.width or 640
        height = request.height or 480
        quality = self.getQuality(request.quality)
        print("quality", quality)
        self.picam2.configure(self.picam2.create_video_configuration(main={"size": (width, height)}))
        self.picam2.start_recording(encoder=self.encoder, output=self.output, quality=quality)
        response.started = True
        return response

    def getQuality(self, quality):
        if quality == '':
            return Quality.HIGH
        if quality == 'VERY LOW':
            return Quality.VERY_LOW
        if quality == 'LOW':
            return Quality.LOW
        if quality == 'MEDIUM':
            return Quality.MEDIUM
        if quality == 'HIGH':
            return Quality.HIGH
        if quality == 'VERY HIGH':
            return Quality.VERY_HIGH

    def stop_video_recording(self, request, response):
        if not self.picam2.started:
            response.message = "Camera was not working"
            return response
        self.picam2.stop_recording()

        current_location = os.getcwd()
        response.video_file_path = current_location + '/video.mp4'
        return response


def main(args=None):
    rclpy.init(args=args)
    video_recorder = VideoRecorder()
    rclpy.spin(video_recorder)
    video_recorder.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
