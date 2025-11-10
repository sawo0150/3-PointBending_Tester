from setuptools import find_packages, setup
import os  # 1. 'os' 임포트 추가
from glob import glob  # 2. 'glob' 임포트 추가

package_name = 'vision_processor'

setup(
    name=package_name,
    version='0.0.0',
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
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # [추가] 새 노드를 실행 파일로 등록
            'deflection_tracker_node = vision_processor.deflection_tracker_node:main',
        ],
    },
)
