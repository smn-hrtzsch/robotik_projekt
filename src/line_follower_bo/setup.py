from setuptools import find_packages, setup

package_name = 'line_follower_bo'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/line_follower_bo_launch.py']),  # Launch-File hinzufügen
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Simon Hörtzsch',
    maintainer_email='Simon.Hoertzsch@student.tu-freiberg.de',
    description='Package für Linienverfolgung zwischen Hindernissen',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'line_follower = line_follower_bo.line_follower:main',
            'handle_obstacle = line_follower_bo.handle_obstacle:main',
            'state_manager = line_follower_bo.state_manager:main',
        ],
    },
)
