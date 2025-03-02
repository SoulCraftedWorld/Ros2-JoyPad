from setuptools import find_packages, setup

package_name = 'joypad'

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
    maintainer='JC',
    maintainer_email='madewithmysoul@gmail.com',
    description='ROS2 JoyPad - управление роботом через клавиатуру и кнопки/robot control via keyboard and buttons',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'joypad = joypad.joypad:main',
        ],
    },
)
