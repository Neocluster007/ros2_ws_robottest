from setuptools import setup

package_name = 'brobot_slam'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/slam_launch.py']),
        ('share/' + package_name + '/config', ['config/slam_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='SLAM toolbox launch for brobot',
    license='MIT',
    entry_points={
        'console_scripts': [],
    },
)
