from setuptools import find_packages, setup

package_name = 'continuous_navigator'

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
    maintainer='fukushima',
    maintainer_email='tsfk9981@gmail.com',
    description='TurtleBot3 continuously moves in a world',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigator_node = continuous_navigator.continuous_navigator:main',
        ],
    },
    )
