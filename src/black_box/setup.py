from setuptools import find_packages, setup

package_name = 'black_box'

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
    maintainer='Jakub',
    maintainer_email='Jakub@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
entry_points={
    'console_scripts': [
        'black_box_node = black_box.black_box_node:main',
        'turtle_controller_node = black_box.turtle_controller_node:main',
    ],
},

)
