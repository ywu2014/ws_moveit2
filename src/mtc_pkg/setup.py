from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'mtc_pkg'

# 自动查找 scripts 目录下的所有 .py 文件
scripts_list = glob(os.path.join('scripts', '*.py'))

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
    scripts=scripts_list,
    entry_points={
        'console_scripts': [
        ],
    },
)
