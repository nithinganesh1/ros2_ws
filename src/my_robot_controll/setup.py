from setuptools import find_packages, setup

package_name = 'my_robot_controll'

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
    maintainer='nithin',
    maintainer_email='nithingganesh1@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "say_hello = my_robot_controll.my_first_node:main",
            "Drow_Circle = my_robot_controll.drow_circle:main"
        ],
    },
)
