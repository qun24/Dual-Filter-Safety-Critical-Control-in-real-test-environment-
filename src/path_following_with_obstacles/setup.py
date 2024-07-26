from setuptools import setup

package_name = 'path_following_with_obstacles'

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
    maintainer='qun',
    maintainer_email='qun@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'elliptical_path_following = path_following_with_obstacles.elliptical_path_following_typeII-d-zcbfs_avoidence:main',
        'path_generator = path_following_with_obstacles.path_generator:main',
        'path_visualizer = path_following_with_obstacles.path_rviz_visualizer:main',
    ],
    },
)
