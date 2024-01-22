# !/usr/bin/python3
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

    def setResponse(self):
        self.send_response(200)
        self.send_header('Age', 0)
        self.send_header('Cache-Control', 'no-cache, private')
        self.send_header('Pragma', 'no-cache')
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
    def do_GET(self):
        if self.path == '/continuous-stream.mjpg':
            self.setResponse()
            self.end_headers()
            try:
                while True:
                    with output.condition:
                        output.condition.wait()
                        frame = output.frame
                    self.wfile.write(b'--FRAME\r\n')
                    self.send_header('Content-Type', 'image/jpeg')
                    self.send_header('Content-Length', len(frame))
                    self.end_headers()
                    self.wfile.write(frame)
                    self.wfile.write(b'\r\n')
            except Exception as e:
                logging.warning(
                    'Removed streaming client %s: %s',
                    self.client_address, str(e))
        elif self.path == '/stream.mjpg':
            self.setResponse()
            try:
                with self.server.output.condition:
                    self.server.output.condition.wait()
                    frame = self.server.output.frame
                self.send_header('Content-Type', 'image/jpeg')
                self.send_header('Content-Length', len(frame))

                self.end_headers()
                self.wfile.write(frame)
                self.wfile.write(b'\r\n')
            except Exception as e:
                logging.warning(
                    'Removed streaming client %s: %s',
                    self.client_address, str(e))
            except Exception as e:
                logging.warning(
                    'Removed streaming client %s: %s',
                    self.client_address, str(e))
        else:
            self.send_error(404)
            self.end_headers()

    def do_POST(self):
        if self.path == '/stream.mjpg':
            self.setResponse()

            try:
                content_len = int(self.headers.get('Content-Length'))
                post_body = self.rfile.read(content_len)
                decoded_json = json.loads(post_body)

                width = decoded_json.get('width', 640)
                height = decoded_json.get('height', 480)

                if (StreamingHandler.width != width) or (StreamingHandler.height != height):
                    StreamingHandler.width = width
                    StreamingHandler.height = height

                    picam2.stop_recording()
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
                logging.warning(
                    'Removed streaming client %s: %s',
                    self.client_address, str(e))
        else:
            self.send_error(404)
            self.end_headers()


class StreamingServer(socketserver.ThreadingMixIn, server.HTTPServer):
    allow_reuse_address = True
    daemon_threads = True


picam2 = Picamera2()
output = StreamingOutput()


def main(args=None):
    print("RPI CAMERA :)")
    # Initialize the rclpy library
    rclpy.init(args=args)
    picam2.configure(picam2.create_video_configuration(main={"size": (640, 480)}))
    picam2.start_recording(JpegEncoder(), FileOutput(output))

    try:
        address = ('', 8000)
        server = StreamingServer(address, StreamingHandler)
        server.output = output
        server.serve_forever()
    finally:
        picam2.stop_recording()
    # Shutdown the ROS client library for Python
    rclpy.shutdown()


if __name__ == '__main__':
    main()
