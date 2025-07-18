from setuptools import find_packages, setup

package_name = 'webots_ros2_example'
data_files = []
data_files.append(
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', [
    'launch/robot_launch.py'
]))
data_files.append(('share/' + package_name + '/resource', [
    'resource/pioneer3at_webots.urdf',
    'resource/ros2_control.yml',
    'resource/nav2_params.yaml',
    'resource/default.rviz',
    'resource/slam_toolbox_params.yaml',
]))
data_files.append(('share/' + package_name, ['package.xml']))
data_files.append(('share/' + package_name + '/worlds', [
    'worlds/pioneer3at.wbt',
]))

setup(
    name=package_name,
    version='2025.0.0',
    packages=[],
    data_files=data_files,
    install_requires=['setuptools', 'launch'],
    zip_safe=True,
    author='Cyberbotics',
    author_email='support@cyberbotics.com',
    maintainer='Cyberbotics',
    maintainer_email='support@cyberbotics.com',
    keywords=['ROS', 'Webots', 'Robot', 'Simulation', 'Examples', 'Pioneer3at'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Pioneer3at robots ROS2 interface for Webots.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'launch.frontend.launch_extension': ['launch_ros = launch_ros']
    }
)

