from setuptools import find_packages, setup

package_name = 'process'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'driver_msgs'],
    zip_safe=True,
    maintainer='ljm',
    maintainer_email='ljm@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'process_node = process.process_node:main',
    ],
    },
)
