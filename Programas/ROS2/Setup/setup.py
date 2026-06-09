from setuptools import find_packages, setup
import os              
from glob import glob

package_name = 'tesis_robot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Instala todos los archivos de la carpeta launch
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Instala todos los archivos de la carpeta config (el yaml, el pgm, el mapa)
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eze',
    maintainer_email='eze@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'uwb_driver = tesis_robot_control.uwb_driver:main',
            'uwb_core = tesis_robot_control.uwb_core:main',
            'button_calibrator = tesis_robot_control.button_calibrator_node:main',
            'uwb_visualizer = tesis_robot_control.uwb_visualizer:main',
            'uwb_raw_reader = tesis_robot_control.uwb_raw_reader:main',
            'mecanum_test_node = tesis_robot_control.mecanum_test_node:main',
            'mecanum_controller = tesis_robot_control.mecanum_controller:main',
            'mpu6050_node = tesis_robot_control.mpu6050_node:main',
        ],
    },
)