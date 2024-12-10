from setuptools import find_packages, setup

package_name = 'project_Aruco'

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
    maintainer='cankrz',
    maintainer_email='cankrz@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "listener_node = project_Aruco.listener_node:main",
            "aruco_detection = project_Aruco.integrated_aruco_detection:main",
        ],
    },
)
