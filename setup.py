from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'iris_drone'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.*')),
        (os.path.join('share', package_name), glob('models/iris_with_*/*/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.*')),
        (os.path.join('share', package_name, 'config'), glob('config/*.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jagadeesh',
    maintainer_email='01fe21bee114@kletech.ac.in',
    description='TODO: Package description',
    license='TODO: License declaration',
    #tests_require=['pytest'],
    extras_require={'test':['pytest'],},
    entry_points={
        'console_scripts': [
            'odom_pub = iris_drone.odom_publisher:main',
            'tf_pub = iris_drone.dynamic_tf:main',
            'ardupilot = iris_drone.ardupilot:run_sim_vehicle',
            'mission = iris_drone.mission:main'
        ],
    },
)
