import json
import socketserver
import time
from http import server

from picamera2 import Picamera2
from picamera2.encoders import MJPEGEncoder, JpegEncoder, H264Encoder
from picamera2.outputs import FfmpegOutput
import threading

isRecording=True


class StreamingHandler(server.BaseHTTPRequestHandler):
    def setResponse(self):
        self.send_response(200)
        self.send_header('Age', 0)
        self.send_header('Cache-Control', 'no-cache, private')
        self.send_header('Pragma', 'no-cache')
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')

    def do_POST(self):
        if self.path == '/start-recording':
            print("start recording")
            self.setResponse()

            content_len = int(self.headers.get('Content-Length'))
            post_body = self.rfile.read(content_len)
            decoded_json = json.loads(post_body)

            width = decoded_json.get('width', 640)
            height = decoded_json.get('height', 480)
            picam2.configure(picam2.create_video_configuration(main={"size": (width, height)}))

            outputFileName = decoded_json.get('output', 'output')

            t = threading.Thread(target=test, args=[outputFileName])
            t.start()

            self.send_response(200)

            self.send_header('Content-type', 'application/json')
            self.end_headers()

            # Send the response back to the client
            response_str = "OK"
            self.wfile.write(response_str.encode('utf-8'))
            pass
        if self.path == '/stop-recording':
            self.setResponse()
            global isRecording

            isRecording= False

            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()

            # Send the response back to the client
            response_str = "OK"
            self.wfile.write(response_str.encode('utf-8'))
            pass

def test(fileName):
    print("test")
    global isRecording

    encoder = MJPEGEncoder()
    output = FfmpegOutput(fileName)

    picam2.start_recording(encoder, output)
    while isRecording:
        time.sleep(1)
    isRecording=False
    picam2.stop_recording()


class StreamingServer(socketserver.ThreadingMixIn, server.HTTPServer):
    allow_reuse_address = True
    daemon_threads = True

picam2 = Picamera2()
video_config = picam2.create_video_configuration()
picam2.configure(video_config)

def main(args=None):
    print("RPI CAMERA")
    # Initialize the rclpy library

    try:
        address = ('', 8000)
        server = StreamingServer(address, StreamingHandler)

        server.serve_forever()
    finally:
        picam2.stop_recording()
    # Shutdown the ROS client library for Python


if __name__ == '__main__':
    main()
