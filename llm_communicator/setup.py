from setuptools import find_packages, setup

package_name = 'llm_communicator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'jarvis = llm_communicator.jarvis:main',
            'jarvis_robot_manager = llm_communicator.jarvis_robot_manager:main',
            'jarvis_ui = llm_communicator.jarvis_ui:main',
            'cobot_llm = llm_communicator.cobot_llm:main'

        ],
    },
)
