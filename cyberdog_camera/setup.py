from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'cyberdog_camera'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(where='.'),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'qr_detector'), glob('qr_detector/*')),
        (os.path.join('share', package_name, 'model'), glob('cyberdog_camera/model/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@todo.todo',
    description='CyberDog Camera Service Node',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_service = cyberdog_camera.vision_service_node:main',
            'vision_test = cyberdog_camera.vision_test_client:main',
        ],
    },
)
