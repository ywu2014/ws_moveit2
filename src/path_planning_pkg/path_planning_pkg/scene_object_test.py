import rclpy
from rclpy.node import Node

from moveit_msgs.msg import CollisionObject, PlanningScene, ObjectColor
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from std_msgs.msg import ColorRGBA
from ament_index_python.packages import get_package_share_directory
from shape_msgs.msg import SolidPrimitive
import time

class SceneObjectTest(Node):
    """
    场景物体发布
    """
    def __init__(self):
        super().__init__("scene_object_test")

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