from glob import glob
from setuptools import setup

package_name = 'serial_sensor_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/serial_sensor_launch.py']),
        ('share/' + package_name + '/config',
            glob('config/*.yaml')),
    ],
    install_requires=[
        'setuptools',
        'pyserial',
        'diagnostic_updater',
    ],
    zip_safe=True,
    author='Dave',
    author_email='dave@dave.com',
    maintainer='YOUR_NAME',
    maintainer_email='YOU@EMAIL.com',
    keywords=['ROS'],
    description='Bridge for JSON-over-UART to ROS2 topics with diagnostics and auto-reset',
    license='Apache-2.0',
  

    tests_require=['pytest'],

    entry_points={
        'console_scripts': [
            'serial_sensor_node = serial_sensor_bridge.serial_sensor_node:main',
        ],
    },
)
