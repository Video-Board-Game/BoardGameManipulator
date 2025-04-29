from setuptools import find_packages, setup

package_name = 'board_manipulator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='smarkwick',
    maintainer_email='spmarkwick@wpi.edu',
    description='Examples of minimal publisher/subscriber using rclpy',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
        	'board_arm_node = board_manipulator.BoardArmNode:main',
        ],
    },
)