# ROS2 RPI REST CAMERA
## API
- continuous-stream.mjpg
  - GET: continous stream from picamera2. Emits continuous stream of frames in not ending connection - connection is never closed.
- stream.mjpg
  - GET: gets single frame in current camera resolution
  - POST: gets single frame and allows to change camera resolution: Json with width, height fields.
