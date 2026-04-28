from setuptools import setup

package_name = 'jeep_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/jeep_driver.launch.py',]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Kaisa Talts',
    author_email='taltskaisa@gmail.com',
    description='The jeep_driver package',
    license='TODO',
    entry_points={
        'console_scripts': [
            'jeep_driver_node = jeep_driver.jeep_driver:main',
            'feedback_node = jeep_driver.mrp_sensor:main',
            'keyboard_node = jeep_driver.teleop_keyboard:main',
            'path_follower_node = jeep_driver.path_follower:main',
            'drive_logic_node = jeep_driver.drive_logic:main',
            'last_path_recorder_node = jeep_driver.last_path_recorder:main',
        ],
    },
)