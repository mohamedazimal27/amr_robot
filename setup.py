import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'amr_robot'

def package_files(directory):
    paths = []
    for (path, directories, filenames) in os.walk(directory):
        for filename in filenames:
            paths.append((os.path.join('share', package_name, path), [os.path.join(path, filename)]))
    return paths

data_files = [
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'),
        glob('launch/*.launch.py')),
    (os.path.join('share', package_name, 'urdf'),
        glob('urdf/*')),
    (os.path.join('share', package_name, 'worlds'),
        glob('worlds/*')),
    (os.path.join('share', package_name, 'config'),
        glob('config/*')),
    (os.path.join('share', package_name, 'maps'),
        glob('maps/*')),
    (os.path.join('share', package_name, 'rviz'),
        glob('rviz/*')),
]

if os.path.exists('models'):
    data_files += package_files('models')

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your@email.com',
    description='AMR robot simulation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'qr_detector = amr_robot.qr_detector:main',
            'inventory_logger = amr_robot.inventory_logger:main',
            'mission_manager = amr_robot.mission_manager:main',
            'metrics_evaluator = amr_robot.metrics_evaluator:main',
        ],
    },
)
