
from setuptools import find_packages, setup

package_name = 'assignment_nodes'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/launch',
        ['launch/assignment.launch.py']),
    ],
    
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Package Maintainer',
    maintainer_email='habiba@example.com',
    description='ROS 2 nodes for the mechatronics systems assignment',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'reset_node = assignment_nodes.reset_node:main',
            'go_to_pose_node = assignment_nodes.go_to_pose_node:main',
            'draw_square_node = assignment_nodes.draw_square_node:main',
        ],
    },
)
