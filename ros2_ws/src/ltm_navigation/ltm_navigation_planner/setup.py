from setuptools import setup

package_name = 'ltm_navigation_planner'

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
    maintainer_email='bec_alexanderjames@outlook.com',
    description='This package contains the navigation planner node for the LTM Navigation System.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigation_planner_node = ltm_navigation_planner.navigation_planner_node:main'
        ],
    },
)
