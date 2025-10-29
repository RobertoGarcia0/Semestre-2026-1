from setuptools import find_packages, setup

package_name = 'example_control'

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
    maintainer='robousr',
    maintainer_email='roberto.gar.1748@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
          "controller_manager = example_control.controller_manager:main",
          "hardware_interface = example_control.hardware_interface:main",
          "manipulator_controller = example_control.manipulator_controller:main"
        ],
    },
)
