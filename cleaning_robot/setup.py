from setuptools import find_packages, setup

package_name = 'cleaning_robot'

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
    maintainer='kevin',
    maintainer_email='kevin@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
	'console_scripts': [
	     'cleaning_fsm = cleaning_robot.cleaning_fsm:main',
             'cleaning_viz = cleaning_robot.cleaning_with_visualization:main'
	],
    },
)
