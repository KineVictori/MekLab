from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'qube_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    
    # In data_files we say witch files are needed and used.
    # Here we start by adding the package, and package.xml files.
    # Then we need to add the setup files we created,
    # as well as the urdf and xacro files needed for the launch file.
    
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'urdf'), 
         glob(os.path.join('urdf', '*.urdf')) + glob(os.path.join('urdf', '*.xacro')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='islandrock1',
    maintainer_email='oysteinmb@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
