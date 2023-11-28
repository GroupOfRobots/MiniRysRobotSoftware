import time

from picamera2 import Picamera2
from picamera2.encoders import H264Encoder,MJPEGEncoder
from picamera2.outputs import FfmpegOutput

picam2 = Picamera2()
video_config = picam2.create_video_configuration()
picam2.configure(video_config)

encoder = MJPEGEncoder()
output = FfmpegOutput('qwerty.mp4')

picam2.start_recording(encoder, output)
time.sleep(10)
picam2.stop_recording()
