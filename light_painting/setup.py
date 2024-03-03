from setuptools import find_packages, setup
import glob
package_name = 'light_painting'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob('launch/*.launch.*')),
        ('share/' + package_name + '/config', glob.glob('config/*')),
        ('share/' + package_name + '/script', glob.glob('script/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kyle',
    maintainer_email='kylewang239@gmail.com',
    description='A light package for doing light painting with the crazyflies',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'flight = light_painting.flight:main',
            'led = light_painting.led:main',
            'waypoint = light_painting.waypoint:main',
        ],
    },
)
