from setuptools import find_packages, setup

package_name = 'rosbridge_data_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'roslibpy',
        'rclpy',
        'rosidl_runtime_py',
        'rosapi'
    ],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='Relay and process data from a rosbridge instance into ROS2 topics',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'rosbridge_topic_receive_node = rosbridge_data_pkg.ros_lib_main:main',
        ],
    },
)
