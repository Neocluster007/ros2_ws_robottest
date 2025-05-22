from setuptools import setup

package_name = 'brobot_tf'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='TF broadcaster from base_link to laser',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tf = brobot_tf.tf_broadcaster:main',
        ],
    },
)
