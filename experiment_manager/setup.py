from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'experiment_manager'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # [추가] 런치 파일 설치
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
        # [추가] 파라미터(config) 파일 설치
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
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
            # [추가] 노드 실행파일로 등록
            'experiment_manager_node = experiment_manager.experiment_manager_node:main',
        ],
    },
)
