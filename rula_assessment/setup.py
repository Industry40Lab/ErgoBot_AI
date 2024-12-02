from setuptools import find_packages, setup
from glob import glob

import os
package_name = 'rula_assessment'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'configs'), glob('configs/*.task')),
        (os.path.join('share', package_name, 'configs/reba_score'), glob('configs/reba_score/*.csv')),
        (os.path.join('share', package_name, 'configs/rula_score'), glob('configs/rula_score/*.csv')),
        (os.path.join('share', package_name, 'configs/ows_score'), glob('configs/ows_score/*.csv')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mostafa',
    maintainer_email='mostafazarei1995mz@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pose_detect_node = rula_assessment.check_setup:main'
        ],
    },
)
