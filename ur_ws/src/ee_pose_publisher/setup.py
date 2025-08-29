from setuptools import find_packages, setup

package_name = 'ee_pose_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', 
            ['launch/tf_ee_pose_publisher.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gustavo-fuentevilla',
    maintainer_email='gustave.loof@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'tf_pose_publisher = ee_pose_publisher.tf_pose_publisher:main',
            'ee_pose_publisher = ee_pose_publisher.ee_pose_publisher:main',
        ],
    },
)
