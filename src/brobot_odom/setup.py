from setuptools import setup

package_name = 'brobot_odom'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/ekf_launch.py']),
        ('share/' + package_name + '/config', ['config/ekf.yaml']),
        ('share/' + package_name + '/resource', ['resource/' + package_name]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='Odometry publisher using encoders',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom = brobot_odom.odom_publisher:main',
        ],
    },
)
