from setuptools import find_packages, setup

package_name = 'my_ros2_package'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='ros2 docker hw',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker_node = my_ros2_package.talker_node:main',
            'listener_node = my_ros2_package.listener_node:main',
        ],
    },
)
