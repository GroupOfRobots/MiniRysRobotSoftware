from setuptools import setup
from glob import glob

package_name = 'ros2_rpi_cv_camera'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('launch/*launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pablo',
    maintainer_email='pawel.rawicki@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ros2_rpi_cv_camera = ros2_rpi_cv_camera.ros2_rpi_cv_camera:main'
        ],
    },
)
