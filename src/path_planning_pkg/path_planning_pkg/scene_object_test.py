import rclpy
from rclpy.node import Node

from moveit_msgs.msg import CollisionObject, PlanningScene, ObjectColor
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from std_msgs.msg import ColorRGBA
from ament_index_python.packages import get_package_share_directory
from shape_msgs.msg import SolidPrimitive
import time

from geometry_msgs.msg import Point
from shape_msgs.msg import Mesh, MeshTriangle
import pyassimp
import os

class SceneObjectTest(Node):
    """
    场景物体发布
    """
    def __init__(self):
        super().__init__("scene_object_test")

        package_share_directory = get_package_share_directory('path_planning_pkg')
        self.mesh_root_path = f"{package_share_directory}/meshes"

        # 场景更新发布
        self.scene_publisher = self.create_publisher(
            PlanningScene,
            '/planning_scene',
            10
        )

        box_pose = Pose()
        box_pose.position.x = 0.1
        box_pose.position.y = 0.1
        box_pose.position.z = 0.1
        self.add_box_object('box_0', [0.1, 0.1, 0.05], box_pose, color=[1.0, 0.0, 0.0, 1.0])

        cyl_pose = Pose()
        cyl_pose.position.x = 0.1
        cyl_pose.position.y = -0.2
        cyl_pose.position.z = 0.2
        self.add_cylinder_object('cyl_0', 0.1, 0.05, cyl_pose, color=[0.0, 1.0, 0.0, 1.0])

        mesh_path = os.path.join(self.mesh_root_path, 'table.stl')
        mesh_pose = Pose()
        mesh_pose.position.x = -0.5
        mesh_pose.position.y = 0.5
        mesh_pose.position.z = 0.6
        self.add_mesh_obstacle('table', mesh_path, mesh_pose)

        time.sleep(10)
        self.remove_object('box_0')

        self.get_logger().info("-------------Scene Object Node Init-------------")
    
    def add_box_object(self, name, size, pose: Pose, frame_id="world", color=[0.7, 0.7, 0.7, 1.0]):
        """添加一个盒子障碍物
        
        Args:
            name (str): 障碍物名称
            size (list): [x, y, z] 尺寸
            pose (list): 盒子中心点位姿
            frame_id: pose相对的坐标系
            color: 物体名称, [r, g, b, a]
        """
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
    node = SceneObjectTest()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()