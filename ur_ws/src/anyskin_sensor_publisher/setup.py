from setuptools import find_packages, setup

package_name = 'anyskin_sensor_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gustavo-fuentevilla',
    maintainer_email='gustave.loof@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'anyskin_sensor_node = anyskin_sensor_publisher.anyskin_sensor_node:main',
        ],
    },
)
