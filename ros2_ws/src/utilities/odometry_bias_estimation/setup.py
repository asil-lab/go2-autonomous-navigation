from setuptools import setup

package_name = 'odometry_bias_estimation'

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
    maintainer='Alexander James Becoy',
    maintainer_email='alexanderjames.becoy@outlook.com',
    description='This package is used to estimate the odometry bias of Unitree Go2',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odometry_bias_estimator_node = odometry_bias_estimation.odometry_bias_estimator_node:main'
        ],
    },
)
