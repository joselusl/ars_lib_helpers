from setuptools import find_packages, setup

package_name = 'ars_lib_helpers'

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
 maintainer='Jose Luis SANCHEZ-LOPEZ',
 maintainer_email='joseluis.sanlop@gmail.com',
 description='TODO: Package description',
 license='BSD',
 tests_require=['pytest'],
 entry_points={'console_scripts': [
        ],},
)


