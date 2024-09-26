import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'ros2_quad_sim_python'

setup(
    name=package_name,
    version='0.0.1',
    # Include the package and sub-packages
    packages=find_packages(include=[package_name, f'{package_name}.*']),
    data_files=[
        ('share/' + package_name, 
            ['package.xml']),
        ('share/' + package_name + '/cfg', 
            ['cfg/rviz_flying_sensor.rviz', 'cfg/flying_sensor.json']),
        ('share/' + package_name + '/launch', 
            glob('launch/*.launch'))
    ],
    install_requires=['setuptools', 'scipy>=1.6', 'numpy>=1.20', 'quad_sim_python'],
    zip_safe=True,
    maintainer='Ricardo de Azambuja',
    maintainer_email='ricardo.azambuja@gmail.com',
    description='Quadcopter simulator and controller based on Python',
    license='MIT',
    tests_require=['pytest']
)
