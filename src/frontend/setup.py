from setuptools import find_packages, setup

package_name = 'frontend'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/jet_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Max',
    maintainer_email='woodsm25@up.edu',
    description='Nodes for remote controlling the baby bot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
			'driver = frontend.frontend_driver:main',
            'drive_motors = frontend.drive_motors:main',
            'conveyor = frontend.conveyor:main',
            'bucket_servos = frontend.bucket_servos:main',
            'camera = frontend.camera:main',
            'bucket_spin = frontend.bucket_spin:main',
            'arduino_driver = frontend.arduino_driver:main',
            'camera_pan = frontend.camera_pan:main',
            'kb_controller = frontend.keyboard_driver:main'
        ],
    },
)
