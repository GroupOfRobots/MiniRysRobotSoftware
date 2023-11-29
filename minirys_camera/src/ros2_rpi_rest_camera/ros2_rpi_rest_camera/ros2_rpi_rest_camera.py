#!/usr/bin/python3
import threading

import rclpy
import io
import logging
import socketserver
from http import server
from threading import Condition
import json

from picamera2 import Picamera2
from picamera2.encoders import JpegEncoder
from picamera2.outputs import FileOutput
from minirys_msgs.srv import RecordVideoStart, RecordVideoStop
from picamera2.encoders import MJPEGEncoder, Quality
from picamera2.outputs import FfmpegOutput
from rclpy.node import Node  # Handles the creation of nodes
import os

class VideoRecorder(Node):
    def __init__(self):
        super().__init__("video_recorder")
        print("VideoRecorder")

        self.encoder = MJPEGEncoder()
        self.output = FfmpegOutput("video.mp4")

        self.picam2 = picam2

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
        self.picam2.configure(self.picam2.create_video_configuration(lores={"size": (width, height)}))
        self.picam2.start_recording(encoder=self.encoder, output=self.output, quality=quality, name="lores")
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
        self.picam2.stop_recording(self.encoder)

        current_location = os.getcwd()
        response.video_file_path = current_location + '/video.mp4'
        return response



class StreamingOutput(io.BufferedIOBase):
    def __init__(self):
        self.frame = None
        self.condition = Condition()

    def write(self, buf):
        with self.condition:
            self.frame = buf
            self.condition.notify_all()

class StreamingHandler(server.BaseHTTPRequestHandler):
    width = 640
    height = 480

    def set_response(self):
        self.send_response(200)
        self.send_header('Age', 0)
        self.send_header('Cache-Control', 'no-cache, private')
        self.send_header('Pragma', 'no-cache')
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')

    def do_GET(self):
        if self.path == '/continuous-stream.mjpg' or self.path == '/stream.mjpg':
            self.set_response()
            try:
                with (self.server.output.condition if self.path == '/stream.mjpg' else output.condition):
                    (self.server.output.condition if self.path == '/stream.mjpg' else output.condition).wait()
                    frame = (self.server.output.frame if self.path == '/stream.mjpg' else output.frame)
                self.send_header('Content-Type', 'image/jpeg')
                self.send_header('Content-Length', len(frame))
                self.end_headers()
                self.wfile.write(frame)
                self.wfile.write(b'\r\n')
            except Exception as e:
                logging.warning('Removed streaming client %s: %s', self.client_address, str(e))
        else:
            self.send_error(404)
            self.end_headers()

    def do_POST(self):
        if self.path == '/stream.mjpg':
            self.set_response()
            try:
                content_len = int(self.headers.get('Content-Length'))
                post_body = self.rfile.read(content_len)
                decoded_json = json.loads(post_body)

                width = decoded_json.get('width', 640)
                height = decoded_json.get('height', 480)

                if (StreamingHandler.width != width) or (StreamingHandler.height != height):
                    StreamingHandler.width = width
                    StreamingHandler.height = height

                    picam2.stop_recording(encoder)
                    picam2.configure(picam2.create_video_configuration(main={"size": (width, height)}))
                    picam2.start_recording(JpegEncoder(), FileOutput(output))

                with self.server.output.condition:
                    self.server.output.condition.wait()
                frame = self.server.output.frame

                self.send_header('Content-Type', 'image/jpeg')
                self.send_header('Content-Length', len(frame))
                self.end_headers()
                self.wfile.write(frame)
                self.wfile.write(b'\r\n')
            except Exception as e:
                logging.warning('Removed streaming client %s: %s', self.client_address, str(e))
        else:
            self.send_error(404)
            self.end_headers()

class StreamingServer(socketserver.ThreadingMixIn, server.HTTPServer):
    allow_reuse_address = True
    daemon_threads = True

picam2 = Picamera2()
output = StreamingOutput()
encoder=JpegEncoder()
def run_server():
    print("runtest11")
    try:
        address = ('', 8000)
        server = StreamingServer(address, StreamingHandler)
        server.output = output
        server.serve_forever()
    finally:
        picam2.stop_recording(encoder)
        rclpy.shutdown()
    print("test2")

def main(args=None):

    print("RPI CAMERA")
    picam2.configure(picam2.create_video_configuration(main={"size": (640, 480)}))
    picam2.start_recording(encoder, FileOutput(output))

    server_thread = threading.Thread(target=run_server)
    server_thread.daemon = True  # The thread will exit when the main program exits
    server_thread.start()

    print("test1")

    rclpy.init(args=args)
    video_recorder = VideoRecorder()
    rclpy.spin(video_recorder)
    video_recorder.destroy_node()

    # rclpy.spin(video_recorder)
    # video_recorder.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()