from setuptools import setup

package_name = 'ltm_go2_auxiliary_sensors'

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
    maintainer='gungnir',
    maintainer_email='gungnir@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'auxiliary_sensors_node = ltm_go2_auxiliary_sensors.auxiliary_sensors_node:main'
        ],
    },
)
