from setuptools import find_packages, setup
import os  # 1. 'os' 임포트 추가
from glob import glob  # 2. 'glob' 임포트 추가

package_name = 'hardware_interface'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 3. [핵심] 런치 폴더 안의 모든 .launch.py 파일을 설치하도록 추가
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wosasa',
    maintainer_email='wosasa@todo.todo',
    description='Arduino serial interface for 3-point bending test',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [# '실행명령어 = 패키지명.파일이름:main'
            'arduino_serial_node = hardware_interface.arduino_serial_node:main',
            'arduino_serialPump_node = hardware_interface.arduino_serialPump_node:main',
        ],
    },
)
