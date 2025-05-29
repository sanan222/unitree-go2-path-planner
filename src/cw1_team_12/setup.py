from setuptools import find_packages, setup
import os
from glob import glob

# Define the package name
package_name = 'cw1_team_12'

setup(
    # Package metadata
    name=package_name,
    version='0.0.0',
    description='COMP0244 Coursework 1',
    license='TODO: License declaration',
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    tests_require=['pytest'],
    python_requires='>=3.6',

    # Automatically find and include all packages (excluding tests)
    packages=[package_name],

    # Data files to be installed.
    # This ensures that package.xml and launch files are installed in the share folder.
    data_files=[
        # Make the package available for ROS2 to find
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        # Install package.xml
        ('share/' + package_name, ['package.xml']),
        # Install all launch files in the launch folder
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Ensure config directory and parameters.json are installed
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],

    # Dependencies required for installation
    install_requires=['setuptools'],

    # This flag indicates that the package can be safely installed as a zip file
    zip_safe=True,

    # Entry points define the executable nodes provided by this package.
    entry_points={
        'console_scripts': [
            # Task 1: Obstacle Follower node.
            'obstacle_follower_node = cw1_team_12.obstacle_follower:main',
            
            # Task 2: Bug0 node.
            'bug0_node = cw1_team_12.bug0:main',
            
            # Task 3: Bug1 node.
            'bug1_node = cw1_team_12.bug1:main',
        ],
    },
)
