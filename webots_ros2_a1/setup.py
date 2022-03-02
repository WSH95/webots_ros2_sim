from setuptools import setup
import os
from glob import glob

package_name = 'webots_ros2_a1'

data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/worlds', glob(os.path.join('worlds', '*.wbt'))),
    ('share/' + package_name + '/worlds', glob(os.path.join('worlds', '*.wbproj'))),
    ('share/' + package_name + '/worlds/textures', glob(os.path.join('worlds/textures', '*.png'))),
    ('share/' + package_name + '/protos', glob(os.path.join('protos', '*.proto'))),
    ('share/' + package_name + '/protos/textures', glob(os.path.join('protos/textures', '*.png'))),
    ('share/' + package_name + '/resource', glob(os.path.join('resource', '*.urdf'))),
    ('share/' + package_name + '/resource', glob(os.path.join('resource', '*.yaml'))),
    ('share/' + package_name + '/launch', glob(os.path.join('launch', '*.launch'))),
    ('share/' + package_name + '/launch', glob(os.path.join('launch', '*.launch.py'))),
    ('share/' + package_name + '/launch', glob(os.path.join('launch', '*.py'))),
]

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wsh',
    maintainer_email='d201780203@hust.edu.cn',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'a1_driver = webots_ros2_a1.a1_driver:main'
        ],
        'launch.frontend.launch_extension': ['launch_ros = launch_ros']
    },
)
