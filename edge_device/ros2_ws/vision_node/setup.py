from setuptools import find_packages, setup

package_name = 'vision_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # This installs the package resource file that lets ROS 2 know the package exists.
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # This installs the package.xml to the share directory.
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dev',
    maintainer_email='bart@opentier.com',
    description='A ROS 2 vision node package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_node = vision_node.vision_node:main'
        ],
    },
)
