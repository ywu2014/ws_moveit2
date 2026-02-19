import rclpy
from rclpy.node import Node
import yaml
import os
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose, Point
from moveit_msgs.msg import PlanningScene, CollisionObject, ObjectColor
from shape_msgs.msg import SolidPrimitive, Mesh, MeshTriangle
from std_msgs.msg import ColorRGBA
import pyassimp

class SceneObjectPublisher(Node):
    def __init__(self):
        super().__init__("scene_object_publish")

        package_share_directory = get_package_share_directory('path_planning_pkg')
        self.mesh_root_path = f"{package_share_directory}/meshes"
        self.config_path = f"{package_share_directory}/config/scene_objects.yaml"

        # 场景更新发布
        self.scene_publisher = self.create_publisher(
            PlanningScene,
            '/planning_scene',
            10
        )

        # 从配置文件加载并添加/删除物体
        self.load_and_apply_scene_config()

        self.get_logger().info("-------------Scene Object Publish Node Init-------------")
    
    def load_and_apply_scene_config(self):
        """加载YAML配置文件并应用场景更新"""
        try:
            with open(self.config_path, 'r') as f:
                config = yaml.safe_load(f)
            
            # 添加配置文件中的物体
            if 'objects_to_add' in config:
                for obj_config in config['objects_to_add']:
                    self.add_object_from_config(obj_config)
            
            # 删除配置文件中的物体
            if 'objects_to_remove' in config:
                for obj_name in config['objects_to_remove']:
                    self.remove_object(obj_name)
                    
        except Exception as e:
            self.get_logger().error(f'加载场景配置失败: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())
    
    def add_object_from_config(self, obj_config):
        """根据配置添加物体"""
        name = obj_config['name']
        obj_type = obj_config['type']
        
        # 创建位姿
        pose = Pose()
        pose.position.x = obj_config['pose']['position']['x']
        pose.position.y = obj_config['pose']['position']['y']
        pose.position.z = obj_config['pose']['position']['z']
        pose.orientation.x = obj_config['pose']['orientation']['x']
        pose.orientation.y = obj_config['pose']['orientation']['y']
        pose.orientation.z = obj_config['pose']['orientation']['z']
        pose.orientation.w = obj_config['pose']['orientation']['w']
        
        frame_id = obj_config.get('frame_id', 'world')
        color = obj_config.get('color', [0.7, 0.7, 0.7, 1.0])
        
        # 根据类型添加不同的物体
        if obj_type == 'box':
            self.add_box_object(name, obj_config['size'], pose, frame_id, color)
        elif obj_type == 'cylinder':
            self.add_cylinder_object(name, obj_config['height'], obj_config['radius'], pose, frame_id, color)
        elif obj_type == 'mesh':
            mesh_path = os.path.join(self.mesh_root_path, obj_config['mesh_file'])
            scale = obj_config.get('scale', [1.0, 1.0, 1.0])
            self.add_mesh_obstacle(name, mesh_path, pose, frame_id, scale)
        else:
            self.get_logger().error(f'未知的物体类型: {obj_type}')
    
    def add_box_object(self, name, size, pose: Pose, frame_id="world", color=[0.7, 0.7, 0.7, 1.0]):
        """添加一个盒子障碍物"""
        # 创建碰撞对象
        collision_object = CollisionObject()
        collision_object.header.frame_id = frame_id
        collision_object.header.stamp = self.get_clock().now().to_msg()
        collision_object.id = name
        
        # 设置碰撞对象形状
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = size  # [x, y, z]
        collision_object.primitives.append(box)

        # 设置碰撞对象位姿
        collision_object.primitive_poses.append(pose)

        # 操作为添加
        collision_object.operation = CollisionObject.ADD

        # 创建PlanningScene消息
        scene = PlanningScene()
        
        # 将碰撞对象添加到场景中
        scene.world.collision_objects.append(collision_object)
        scene.is_diff = True

        # 设置颜色
        obj_color = ObjectColor()
        obj_color.id = name
        obj_color.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=color[3])
        scene.object_colors.append(obj_color)
        
        # 发布场景
        self.scene_publisher.publish(scene)
        self.get_logger().info(f'已添加障碍物: {name}, pose: {pose}')

    def add_cylinder_object(self, name, height, radius, pose: Pose, frame_id="world", color=[0.7, 0.7, 0.7]):
        """添加一个圆柱形障碍物"""
        collision_object = CollisionObject()
        collision_object.header.frame_id = frame_id
        collision_object.header.stamp = self.get_clock().now().to_msg()
        collision_object.id = name
        
        cylinder = SolidPrimitive()
        cylinder.type = SolidPrimitive.CYLINDER
        cylinder.dimensions = [height, radius]
        collision_object.primitives.append(cylinder)

        collision_object.primitive_poses.append(pose)

        collision_object.operation = CollisionObject.ADD

        scene = PlanningScene()

        scene.world.collision_objects.append(collision_object)
        scene.is_diff = True

        # 创建颜色对象
        obj_color = ObjectColor()
        obj_color.id = name
        obj_color.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=color[3])
        scene.object_colors.append(obj_color)
        
        self.scene_publisher.publish(scene)
        self.get_logger().info(f'已添加圆柱障碍物: {name}')

    def add_mesh_obstacle(self, name, mesh_path, pose: Pose, frame_id="world", scale=(1.0, 1.0, 1.0)):
        """添加基于Mesh文件的障碍物 (使用 pyassimp 加载)"""
        
        # 检查文件是否存在
        if not os.path.exists(mesh_path):
            self.get_logger().error(f'Mesh 文件不存在: {mesh_path}')
            return

        scene = PlanningScene()
        collision_object = CollisionObject()
        collision_object.header.frame_id = frame_id
        collision_object.header.stamp = self.get_clock().now().to_msg()
        collision_object.id = name
        
        try:
            # 使用 with 语句加载模型 (pyassimp 5.x+ 推荐方式)
            with pyassimp.load(mesh_path, processing=pyassimp.postprocess.aiProcess_Triangulate) as load_scene:
                # 创建 ROS Mesh 消息
                mesh_msg = Mesh()
                
                # 遍历 assimp 场景中的所有 mesh
                for a_mesh in load_scene.meshes:
                    # 处理面
                    for face in a_mesh.faces:
                        triangle = MeshTriangle()
                        
                        # 修正：兼容不同版本的 pyassimp 返回的数据结构
                        if hasattr(face, 'indices'):
                            indices = face.indices
                        else:
                            indices = face
                        
                        if len(indices) == 3:
                            triangle.vertex_indices = [int(indices[0]), int(indices[1]), int(indices[2])]
                            mesh_msg.triangles.append(triangle)
                        elif len(indices) == 4:
                            # 处理四边形
                            v0, v1, v2, v3 = int(indices[0]), int(indices[1]), int(indices[2]), int(indices[3])
                            tri1 = MeshTriangle()
                            tri1.vertex_indices = [v0, v1, v2]
                            mesh_msg.triangles.append(tri1)
                            tri2 = MeshTriangle()
                            tri2.vertex_indices = [v0, v2, v3]
                            mesh_msg.triangles.append(tri2)
                    
                    # 处理顶点位置
                    for vertex in a_mesh.vertices:
                        point = Point()
                        point.x = vertex[0] * scale[0]
                        point.y = vertex[1] * scale[1]
                        point.z = vertex[2] * scale[2]
                        mesh_msg.vertices.append(point)
                
                # 设置碰撞mesh物体形状
                collision_object.meshes.append(mesh_msg)
                # 设置碰撞mesh位姿
                collision_object.mesh_poses.append(pose)

                collision_object.operation = CollisionObject.ADD
                
                scene.world.collision_objects.append(collision_object)
                scene.is_diff = True
                
                self.scene_publisher.publish(scene)
                self.get_logger().info(f'已添加 mesh 障碍物: {name}')
        except pyassimp.errors.AssimpError as e:
            self.get_logger().error(f'Assimp 加载错误: {str(e)}')
        except Exception as e:
            self.get_logger().error(f'加载 mesh 失败: {mesh_path}, 错误: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())

    def remove_object(self, name):
        """移除障碍物"""
        scene = PlanningScene()
        
        collision_object = CollisionObject()
        collision_object.id = name
        collision_object.operation = CollisionObject.REMOVE
        
        scene.world.collision_objects.append(collision_object)
        scene.is_diff = True
        
        self.scene_publisher.publish(scene)
        self.get_logger().info(f'已移除障碍物: {name}')

def main(args=None):
    rclpy.init(args=args)
    node = SceneObjectPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()