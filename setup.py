import os
from glob import glob


from setuptools import find_packages, setup


package_name = 'o3r_ros2'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*_launch.py')),
        (os.path.join('lib', package_name, 'scripts'), glob('scripts/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sameer Tuteja',
    maintainer_email='sameer.tuteja05@gmail.com',
    description='O3R Examples',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'color_pcl_pub = o3r_ros2.color_pcl_pub:main'
        ],
    },
)
