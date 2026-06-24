from setuptools import find_packages, setup

package_name = 'scene_graph_vision'

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
    maintainer='husarion',
    maintainer_email='husarion@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		'clusterer = scene_graph_vision.lidar_clusterer:main',
		'zone_publisher = scene_graph_vision.zone_publisher:main',
		'manager = scene_graph_vision.scene_graph_manager:main',
        	'controller = scene_graph_vision.safety_controller:main'
        ],
    },
)
