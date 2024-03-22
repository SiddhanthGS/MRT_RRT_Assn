from setuptools import find_packages, setup

package_name = 'aruco_detect_pkg'

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
    maintainer='siddy',
    maintainer_email='siddy@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
           "img_server=aruco_detect_pkg.aruco_server:main",
           "img_client=aruco_detect_pkg.client_aruco:main",
           "arucoDet=aruco_detect_pkg.rrt_Server_2:main",
           "img_client_2=aruco_detect_pkg.client_aruco_2:main" 
        ],
    },
)
