from setuptools import setup

package_name = 'simple_filter'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='adam',
    maintainer_email='adam.eliasdk@gmail.com',
    description='Simple complementary filter fusing IMU + odometry orientation (ROS2 port).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_filter = simple_filter.simple_filter:main',
        ],
    },
)
