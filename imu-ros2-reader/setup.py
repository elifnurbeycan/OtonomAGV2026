from setuptools import find_packages, setup

package_name = 'imu_serial_reader'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='elif',
    maintainer_email='elif@example.com',
    description='Read IMU data from Arduino over serial and publish as sensor_msgs/Imu',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_serial_publisher = imu_serial_reader.imu_serial_publisher:main',
        ],
    },
)
