import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'backend'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/description', glob('description/*')),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='max',
    maintainer_email='woodsm25@up.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rc_controller = backend.rc_controller:main',
            'head_controller = backend.head_controller:main',
            'main_controller = backend.main_controller:main',
            'motion_controller = backend.motion_controller:main',
            'global_mapper = backend.global_mapper:main',
            'global_mapper_edge = backend.global_mapper_edge:main',
            'global_costmapper = backend.global_costmapper:main',
            'path_planner = backend.path_planner:main',
            'test_path_plan = backend.test_path_planner:main',
            'test_tag = backend.tag_client:main',
            'image_saver = backend.debug.image_saver:main',
            'test_path_replanning = backend.test_path_replanning:main',
            'localizer = backend.localizer:main',
            'test_localizer = backend.test_localizer:main',
            'mining_controller = backend.mining_controller:main',
            'tag_detector = backend.tag_detector:main',
            'rgb_transport = backend.rgb_transport:main',
            'joystick_driver = backend.joystick_driver:main',
            'keyboard_driver_comp = backend.keyboard_driver_comp:main',
        ],
    },
)
