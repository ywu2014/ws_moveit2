from setuptools import find_packages, setup
from glob import glob

package_name = 'path_planning_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/launch', glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ywu',
    maintainer_email='yejunwu123@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'path_planning_action = path_planning_pkg.path_planning_action:main',
            'path_planning_moveitpy = path_planning_pkg.path_planning_moveitpy:main',
            'marker_test = path_planning_pkg.marker_test:main',
        ],
    },
)
