from setuptools import find_packages, setup

package_name = 'ur_motion_routines'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', 
            ['launch/ur_motion_routines.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gustavo-fuentevilla',
    maintainer_email='gustave.loof@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ur_motion_routines_node = ur_motion_routines.ur_motion_routines_node:main',
        ],
    },
)
