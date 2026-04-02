import numpy as np
from scipy.spatial.transform import Rotation as R
import yaml

class CoordinateTransformer:

    def __init__(self, config_path):
        self.config = self.load_config(config_path)

    def load_config(self, file_path):
        """加载 YAML 配置文件"""
        try:
            with open(file_path, 'r') as f:
                config = yaml.safe_load(f)
                return config
        except Exception as e:
            raise e

    def get_pose(self, frame_id, frame_base='world'):
        pose = self.config[frame_id][frame_base]
        return pose['translation'], pose['rotation']
    
    def get_relative_transform(self, parent_frame, child_frame, frame_base='world'):
        pos_parent, quat_parent = self.get_pose(parent_frame, frame_base=frame_base)
        pos_child, quat_child = self.get_pose(child_frame, frame_base=frame_base)

        return self._get_relative_transform(pos_parent, quat_parent, pos_child, quat_child)

    def _get_relative_transform(self, pos_a, quat_a, pos_b, quat_b):
        """
        计算 B 相对于 A 的位置和旋转
        
        参数:
        pos_a: A的世界坐标
        quat_a: A的世界旋转[xyzw]
        pos_b: B的世界坐标
        quat_b: B的世界旋转
        
        返回:
        relative_pos: B相对于A的位置
        relative_quat: B相对于A的旋转
        """
        
        # 1. 处理旋转
        # 创建旋转对象
        r_a = R.from_quat(quat_a)
        r_b = R.from_quat(quat_b)
        
        # 计算 B 相对于 A 的旋转: R_rel = R_a_inv * R_b
        # 四元数乘法顺序很重要，这里对应的是左乘 R_a 的逆
        r_rel = r_a.inv() * r_b
        relative_quat = r_rel.as_quat()

        # 强制 w 为正
        # 如果 w (relative_quat[3]) 是负数，则整个四元数取反
        if relative_quat[3] < 0:
            relative_quat = -relative_quat
        
        # 2. 处理位置
        # 计算位置差向量
        diff_pos = np.array(pos_b) - np.array(pos_a)
        
        # 将差向量旋转到 A 的局部坐标系: P_rel = R_a_inv * (P_b - P_a)
        # 使用 r_a.inv().apply() 相当于矩阵乘法
        relative_pos = r_a.inv().apply(diff_pos)
        
        # return relative_pos, relative_quat
        return [relative_pos[0].item(), relative_pos[1].item(), relative_pos[2].item()], [relative_quat[0].item(), relative_quat[1].item(), relative_quat[2].item(), relative_quat[3].item()]

    def generate_static_tf(self, frame_pairs, file_path='./static_tf.yaml'):
        static_tf_configs = {}
        tfs = []
        for frame_pair in frame_pairs:
            parent_frame = frame_pair[0]
            child_frame = frame_pair[1]
            translation, rotation = self.get_relative_transform(parent_frame, child_frame)
            tfs.append({
                'frame_id': parent_frame,
                'child_frame_id': child_frame,
                'translation': {
                    'x': translation[0],
                    'y': translation[1],
                    'z': translation[2],
                },
                'rotation': {
                    'x': rotation[0],
                    'y': rotation[1],
                    'z': rotation[2],
                    'w': rotation[3],
                }
            })

        static_tf_configs['tfs'] = tfs
        with open(file_path, 'w', encoding='utf-8') as f:
            # allow_unicode=True 允许在 YAML 中直接显示中文，而不是转义字符
            yaml.dump(static_tf_configs, f, allow_unicode=True)

if __name__ == "__main__":
    config_path = './frame_config.yaml'
    transformer = CoordinateTransformer(config_path)
    translation, rotation = transformer.get_pose('pre_grasp_pt_1')
    print(f'translation: {translation}, rotation: {rotation}')

    ## 生成静态tf配置文件
    static_tf_path = './static_tf.yaml'
    frame_pairs = [
        # (frame_id, child_frame_id)
        ('track_link', 'pre_grasp_pt_1'),
        ('track_link', 'grasp_pt_1'),
    ]
    transformer.generate_static_tf(frame_pairs, file_path=static_tf_path)