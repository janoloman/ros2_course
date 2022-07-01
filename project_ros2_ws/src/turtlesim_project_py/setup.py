from setuptools import setup

package_name = 'turtlesim_project_py'

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
    maintainer='drone',
    maintainer_email='manuelpedrazag@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "target_publisher = turtlesim_project_py.target_publisher:main",
            "target_controller = turtlesim_project_py.turtle_controller:main"
        ],
    },
)
