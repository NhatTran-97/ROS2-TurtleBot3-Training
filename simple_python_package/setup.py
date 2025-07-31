from setuptools import find_packages, setup

package_name = 'simple_python_package'

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
    maintainer='duynhat',
    maintainer_email='nhat.tranduy1997@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_publisher = simple_python_package.simple_publisher:main',
            'simple_subscriber = simple_python_package.simple_subscriber:main',
            'example_node = simple_python_package.example_node:main',
            'teleop_node = simple_python_package.teleop:main',
            'image_subscriber_node = simple_python_package.image_subscriber:main',
        ],
    },
)
