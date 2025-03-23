from setuptools import find_packages, setup

package_name = 'project_01'

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
    maintainer='tarikul69',
    maintainer_email='tarikul69@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "my_node = project_01.object_detection:main",
            "draw_circle = project_01.draw_circle:main"
        ],
    },
)
