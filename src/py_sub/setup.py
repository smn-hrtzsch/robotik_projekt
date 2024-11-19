from setuptools import setup

package_name = 'py_sub'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Simon Hörtzsch',
    maintainer_email='Simon.Hoertzsch@student.tu-freiberg.de',
    description='Mein ROS2 Subscriber',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'subscriber = py_sub.subscriber_member_function:main',
        ],
    },
)
