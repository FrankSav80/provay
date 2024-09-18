from setuptools import setup, find_packages

package_name = 'rclpy_param_helper'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy', 'rclpy', 'ros2param'],
    zip_safe=True,
    maintainer='Ricardo de Azambuja',
    maintainer_email='ricardo.azambuja@gmail.com',
    description='Convert between Python dictionary and ROS2 parameters',
    license='MIT',
    tests_require=['pytest'],
)