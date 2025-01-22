from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'line_follower'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch-Dateien hinzufügen
        (os.path.join('share', package_name, 'launch'), glob('line_follower/launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='simon',
    maintainer_email='Simon.Hoertzsch@student.tu-freiberg.de',
    description='Follows a white line using the camera',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Hier die Python-Node registrieren
            'line_follower = line_follower.line_follower:main',
        ],
    },
)
