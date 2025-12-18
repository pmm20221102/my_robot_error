from setuptools import find_packages, setup

package_name = 'demo_robot_stop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wagn',
    maintainer_email='wagn@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [ 'lidar_sim_node = demo_robot_stop.lidar_sim_node:main',
            'sicher_stop = demo_robot_stop.sicher_stop:main',
            'sicher_marker_node = demo_robot_stop.sicher_marker_node:main',
            'circle_node = demo_robot_stop.circle_node:main',
        ],
    },
)
