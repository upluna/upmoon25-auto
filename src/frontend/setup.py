from setuptools import find_packages, setup

package_name = 'bbot'

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
			'driver = bbot.bbot_driver:main',
            'drive_motors = bbot.drive_motors:main',
            'conveyor = bbot.conveyor:main',
            'bucket_servos = bbot.bucket_servos:main',
            'camera = bbot.camera:main',
            'bucket_spin = bbot.bucket_spin:main',
            'bbot_receiver_js = bbot.bbot_receiver_js:main',
            'arduino_driver = bbot.arduino_driver:main',
            'camera_pan = bbot.camera_pan:main',
            'kb_controller = bbot.keyboard_driver:main'
        ],
    },
)
