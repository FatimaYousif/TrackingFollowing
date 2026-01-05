from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'rpi_check'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
          # Add this line to include launch files
            (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.xml') + glob('launch/*.launch.py')),
     
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='pi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detection_node = rpi_check.detection:main',
            'detection_node2 = rpi_check.track_id:main',
            'botsort_node = rpi_check.botsort_track:main',
            'main = rpi_check.main:main',
            
            
        ],
    },
)
