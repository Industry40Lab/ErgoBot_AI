from setuptools import find_packages, setup
import os
package_name = 'rula_gui'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'resource'), 
            ['resource/no_frame.png']),
        ('share/' + package_name + '/launch', ['launch/rula_run.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shayan',
    maintainer_email='shayan@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rulaGui = rula_gui.rulaGui:main'
        ],
    },
)
