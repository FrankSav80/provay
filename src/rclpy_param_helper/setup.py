from setuptools import setup, find_packages

package_name = 'rclpy_param_helper'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy', 'rospy'],
    zip_safe=True,
    maintainer='Ricardo de Azambuja',
    maintainer_email='ricardo.azambuja@gmail.com',
    description='Convert between Python dictionary and ROS parameters for ROS 1',
    license='MIT',
    tests_require=['pytest'],
)
