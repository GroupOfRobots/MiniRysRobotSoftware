# BscJakubSadowski

## OpenCV

Basically, there was no openCV library on the RPI. You had to install it, for example, with the command `sudo apt install -y python3-opencv`.  It could also be done via `pip`. ROS 2 did not include the `cv_bridge` library. It can be installed by  following the instructions at `https://index.ros.org/p/cv_bridge/`. However the needed library is also linked to this repository by `git submodule`

## Launching programmes
This directory contain four executable files: 

  - Two line followers, One using camera directly and another using topic with image from coamera.
  - Wall follower
  - Combined follower

In order to turn them on, MiniRyś main launch must first be runned `ros2 launch minirys_ros2 minirys.launch.py`. Of course you must be sourced and pins schould be prepared.

To start line follower with camera inside programm `ros2 launch followers line_follower.launch.py`

To start line follower with image reciving from Topic `ros2 launch followers line_follower_no_cam.launch.py`

To start wall follower `ros2 launch followers wall_follower.launch.py`

To start combined follower `ros2 launch followers combined_follower.launch.py`
