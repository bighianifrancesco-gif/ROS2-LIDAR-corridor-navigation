import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'maze_runner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),

        (os.path.join('share', package_name, 'worlds'),
         glob('worlds/*.world')),

        # install all model files (not just .sdf)
        (os.path.join('share', package_name, 'models'),
         glob('models/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='francesco',
    maintainer_email='francesco@todo.todo',
    description='Maze runner wall follower',
    license='TODO',
    entry_points={
        'console_scripts': [
            'wall_follower = maze_runner.wall_follower:main',
        ],
    },
)
