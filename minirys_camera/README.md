# minirys_camera
The repository responsible for handling the robot's webcam

## Nodes
- `ros2_rpi_camera`: responsible for transmitting the image from the camera on ros2 topic
- `ros2_rpi_cv_camera`: responsible for transmitting image in black and white theme via ros2 topic
- `ros2_rpi_rest_camera`: responsible for transmitting the image from the camera via the REST protocol
- `ros2_rpi_video_recorder`: responsible for recording video
- `simple_image_publisher`: responsible for transmitting the configurable image from both camera streams on ROS 2 topic
  - Can publish both 'main' and 'lores' streams with different resolutions and different frame rates
  - Both streams are transmitted on different topics - only one may be selected
  - Each stream can be configured to transmit only a specified part of image (ROI)
    - Data tha would be discarded anyway does not have to be transmitted - which improves performance
