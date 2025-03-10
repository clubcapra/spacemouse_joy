from setuptools import find_packages, setup

package_name = 'capra_spacemouse_package'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Club Capra ETS',
    maintainer_email='capra@ens.etsmtl.ca',
    description='''This ROS 2 package provides a node to publish a 3DSpaceMouse's input as constant `Joy` messages and another to listen to and print `Joy` messages for easy validation. It supports disconnects.''',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'spacemouse_joy = capra_spacemouse_package.spacemouse_joy:main',
            'joy_listener = capra_spacemouse_package.joy_listener:main'

        ],
    },
)
