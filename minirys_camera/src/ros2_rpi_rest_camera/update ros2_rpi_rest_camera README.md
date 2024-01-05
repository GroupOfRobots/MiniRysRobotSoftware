# ROS2 RPI REST CAMERA
## API
URL: http://<HOST>:8000/<PATH>   
FOR EXAMPLE: http://192.168.0.101:8000/stream.mjpg  
### AVAILABLE PATHS
- continuous-stream.mjpg
  - GET: continous stream from picamera2. Emits continuous stream of frames in not ending connection - connection is never closed.
- stream.mjpg
  - GET: gets single frame in current camera resolution
  - POST: gets single frame and allows to change camera resolution: Json with width, height fields.
