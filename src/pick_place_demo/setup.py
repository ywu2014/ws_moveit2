from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'pick_place_demo'

def get_data_files(root_src_dir, root_dst_dir):
    """
    递归查找 root_src_dir 下的所有文件，并生成 data_files 所需的列表。
    结构: [(目标安装路径, [源文件列表]), ...]
    """
    data_files = []
    
    # 遍历源目录
    for (dirpath, dirnames, filenames) in os.walk(root_src_dir):
        # 如果当前目录下没有文件，跳过（避免安装空文件夹）
        if not filenames:
            continue
            
        # 获取相对于 root_src_dir 的相对路径
        # 例如: model/assets/ -> assets/
        relative_path = os.path.relpath(dirpath, root_src_dir)
        
        # 拼接完整的目标安装路径
        # 例如: share/panda_mujoco_sim/model/assets
        full_dst_dir = os.path.join('share', package_name, root_dst_dir, relative_path)
        
        # 获取当前目录下所有文件的完整路径
        # glob('*') 获取当前目录文件，然后拼接上 dirpath
        files_list = [os.path.join(dirpath, f) for f in filenames]
        
        # 添加到结果列表
        data_files.append((full_dst_dir, files_list))
        
    return data_files

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
        *get_data_files('model', 'model'), 
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
            'sim_mujoco_pick_place = pick_place_demo.sim_mujoco_pick_place:main',
            'path_planning = pick_place_demo.path_planning_moveitpy_v2:main',
            'static_tf_publish = pick_place_demo.static_tf_publish:main',
        ],
    },
)
