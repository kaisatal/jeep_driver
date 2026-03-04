from setuptools import setup

package_name = 'jeep_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/driver.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='The jeep_driver package',
    entry_points={
        'console_scripts': [
            'driver_node = jeep_driver.driver:main',
            'feedback_node = jeep_driver.mrp_sensor:main',
            'teleop_node = jeep_driver.teleop_keyboard:main',
        ],
    },
)