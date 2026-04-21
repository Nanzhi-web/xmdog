from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'cyberdog_camera'

setup(
    name=package_name,
    version='0.0.0',
    
    # 1. 自动查找所有包 (它会自动找到 'cyberdog_camera' 目录和里面的 'vision_logic' 子目录)
    packages=find_packages(where='.'),

     data_files=[
    # ← 这一行很重要！确保包被 ament 索引
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'qr_detector'), glob('qr_detector/*')),
],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@todo.todo',
    description='CyberDog Camera Service Node',
    license='TODO: License declaration',
    tests_require=['pytest'],
    
    # 2. ！！！这才是匹配你目录结构的正确入口点！！！
    # 它的意思是：在 'cyberdog_camera' (模块) 里的 'vision_service_node' (文件) 里找 'main' (函数)
    entry_points={
        'console_scripts': [
            'vision_service = cyberdog_camera.vision_service_node:main',
            'vision_test = cyberdog_camera.vision_test_client:main',
        ],
    },
)
