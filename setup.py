from setuptools import setup

package_name = 'sgengine'

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
    maintainer='Colorado School of Mines - Robotics Club',
    maintainer_email='jcdavis@mines.edu',
    description='SpaceGrantEngine - ROS2 Utilities Package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'talker = py_pubsub.publisher_member_function:main',
        ],
    },
)
