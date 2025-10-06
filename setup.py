from setuptools import setup

package_name = 'joy_drive'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mroavi',
    maintainer_email='mroavi@gmail.com',
    description='Drive robot with joystick and buttons (X/O throttle, stick steering).',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joy_drive_node = joy_drive.joy_drive_node:main',
        ],
    },
)
