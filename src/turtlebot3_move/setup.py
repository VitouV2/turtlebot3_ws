from setuptools import setup

package_name = 'turtlebot3_move'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='v2',
    maintainer_email='v2@todo.todo',
    description='TurtleBot3 movement package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_turtlebot = turtlebot3_move.move_turtlebot3:main',  # Must match script and main function
        ],
    },
)
