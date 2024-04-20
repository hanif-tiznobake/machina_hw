import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'sensor_service'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Hanif Tiznobake',
    maintainer_email='hanif@tiznobake.com',
    description='Client/Server network for 3-DOF sensor',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'service = sensor_service.server:main',
            'client = sensor_service.client:main',
        ],
    },
)
