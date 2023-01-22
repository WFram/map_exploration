from setuptools import setup

package_name = 'frontier_discoverer'

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
    maintainer='Andrey Penkovskiy',
    maintainer_email='earl.freedom.ea@gmail.com',
    description='Frontier Exploration for ROS2 Navigation',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'discoverer = frontier_discoverer.frontier_discoverer:main',
            'recovery = frontier_discoverer.recovery:main'
        ],
    },
)
