import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'camera_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # เพิ่มส่วนนี้เพื่อติดตั้งไฟล์ Python ทั้งหมดใน package ไปยังตำแหน่งที่ถูกต้อง
        (os.path.join('lib/python3.10/site-packages', package_name), glob(os.path.join(package_name, '*.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='monkey',
    maintainer_email='thanawatsukamporn@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cam_node = camera_pkg.cam_node:main',
            'controller = camera_pkg.controller:main',
            'objdetect = camera_pkg.objectdetection:main',
            'conui = camera_pkg.robot_controller_app:main',
        ],
    },
)