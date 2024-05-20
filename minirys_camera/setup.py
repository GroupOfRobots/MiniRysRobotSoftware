from setuptools import setup

package_name = 'minirys_camera'
from glob import glob

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name + '/ros2_rpi_camera',
              package_name + '/ros2_rpi_cv_camera',
              package_name + '/ros2_rpi_rest_camera',
              package_name + '/ros2_rpi_video_recorder'],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('launch/*launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pablo',
    maintainer_email='pawel.rawicki@gmail.com',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ros2_rpi_camera = ' + package_name + '.ros2_rpi_camera.ros2_rpi_camera:main',
            'ros2_rpi_cv_camera = ' + package_name + '.ros2_rpi_cv_camera.ros2_rpi_cv_camera:main',
            'ros2_rpi_rest_camera = ' + package_name + '.ros2_rpi_rest_camera.ros2_rpi_rest_camera:main',
            'ros2_rpi_video_recorder = ' + package_name + '.ros2_rpi_video_recorder.ros2_rpi_video_recorder:main'
        ],
    },
)
