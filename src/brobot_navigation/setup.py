from setuptools import find_packages, setup

package_name = 'brobot_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    data_files=[
        ('share/' + package_name + '/launch', ['launch/nav2_launch.py']),
        ('share/' + package_name + '/config', [
            'config/amcl.yaml',
            'config/nav2_params.yaml',
            'config/bt_navigator.yaml',
            'config/planner_server.yaml',
            'config/navigate_through_poses_w_replanning_and_recovery.xml',
            'config/navigate_to_pose_w_replanning_and_recovery.xml']),
        ('share/' + package_name + '/maps', ['maps/map.yaml', 'maps/map.pgm']),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your@email.com',
    description='Nav2 launch package for robot navigation',
    license='MIT',
    entry_points={
        'console_scripts': [],
    },
)