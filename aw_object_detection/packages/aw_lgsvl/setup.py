from setuptools import setup

package_name = 'aw_lgsvl'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alexey Razgulyaev',
    maintainer_email='alexey@webelement.click',
    description='Converting Autoware.Auto perception messages to LGSVL perception messages',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'converter_node = aw_lgsvl.converter_node:main'
        ],
    },
)
