# L6470 library for GNU/Linux
[![pipeline status](https://gitlab.com/mjbogusz/l6470-linux/badges/master/pipeline.svg)](https://gitlab.com/mjbogusz/l6470-linux/-/commits/master)

A library for interfacing multiple L6470 drivers from a GNU/Linux SBC (e.g. Raspberry Pi).

Links:
* Homepage: [https://gitlab.com/mjbogusz/l6470-linux/](https://gitlab.com/mjbogusz/l6470-linux/)
* Documentation/API: [https://mjbogusz.gitlab.io/l6470-linux/](https://mjbogusz.gitlab.io/l6470-linux/)

## Status
The two examples provided in the `examples` directory are tested on a Raspberry Pi 4 with a custom shield hosting 2 L6470s in a daisy chain configuration.

## Usage
This library can be used either as a git submodule or as a ROS/Colcon/Catkin workspace package.

Use CMake and either add `add_subdirectory()` (for git submodule) or `find_package()` (for workspace package), then follow the API and examples.

## Attribution
This library is based off of the [Braingram's dSPIN library](https://github.com/braingram/dSPIN), which in turn was based off of the [SparkFun AutoDriver library](https://github.com/sparkfun/L6470-AutoDriver).

## License
Just as the libraries this is based upon:

This product is open source!
The code is beerware; if you see me (or Braingram, or Brett Graham, or any other SparkFun employee) at the local, and you've found our code helpful, please buy us a round!
Please use, reuse, and modify these files as you see fit. Please maintain attribution to SparkFun Electronics and Brett Graham and release anything derivative under the same license.

Distributed as-is; no warranty is given.
