from setuptools import setup

package_name = 'ltm_scan_procedure'

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
    description='This package contains the scan procedure node for the LTM project.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'scan_procedure_node = ltm_scan_procedure.scan_procedure_node:main'
        ],
    },
)
