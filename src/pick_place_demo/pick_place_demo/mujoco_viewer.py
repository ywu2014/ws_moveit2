import time
import mujoco
import mujoco.viewer
import glfw
import cv2
import numpy as np
import threading

class BaseViewer:
    def __init__(self, model_path, sleep_time:float=None):
        """
        mujoco viewer
        
        :param self: Description
        :param model_path: MuJoCo模型XML文件的路径
        """
        self.model_path = model_path
        # 获取模型对象, 包含了物理场景的静态描述, 如：物体的质量、几何形状、关节类型、连接方式、传感器定义
        self.model = mujoco.MjModel.from_xml_path(model_path)
        # 获取运行时数据对象, 包含了物理场景的动态状态, 如：关节的角度、速度、加速度、激活状态、传感器测量值、仿真时间等
        self.data = mujoco.MjData(self.model)
        # 图形窗口句柄
        self.mujoco_handle = None

        # 每个step运行等待时间
        self.sleep_time = sleep_time

        self.init_viewer()

    def is_running(self):
        """
        模拟器是否还在运行
        """
        return self._is_running and self.mujoco_handle.is_running()
    
    def stop(self):
        """
        停止 Viewer 线程
        """
        self._is_running = False
        if self._viewer_thread and self._viewer_thread.is_alive():
            self._viewer_thread.join(timeout=1.0)

    def sync(self):
        """
        将物理引擎（data）中计算出的最新状态，同步并渲染到图形窗口中
        """
        self.mujoco_handle.sync()

    def _get_obj_id(self, obj_type, obj_name:str):
        """
        获取物体(body、camera等)的运行时id
        
        :param obj_type: 物体类型, mujoco.mjtObj.mjOBJ_xxx
        :param obj_name: 物体名称, MJCF中物体的name属性
        """
        return mujoco.mj_name2id(self.model, obj_type, obj_name)
    
    def get_body_id(self, name:str):
        """
        获取body物体的运行时id
        
        :param name: body 名称
        """
        return self._get_obj_id(mujoco.mjtObj.mjOBJ_BODY, name)
    
    def get_joint_id(self, name:str):
        """
        获取joint的运行时id
        
        :param name: joint 名称
        """
        return self._get_obj_id(mujoco.mjtObj.mjOBJ_JOINT, name)

    def start(self):
        """
        起动 MuJoCo 模型的主循环
        """
        self._is_running = True
        # self.init_viewer()

        # 前置处理, 循环前的一些自定义初始化工作
        self.pre_process()

        # while self.is_running():
        #     self.step()

        # 创建并启动 viewer 线程
        self._viewer_thread = threading.Thread(target=self._run_viewer_thread)
        self._viewer_thread.daemon = True # 设置为守护线程，主程序退出时它也会自动退出
        self._viewer_thread.start()

    def _run_viewer_thread(self):
        """
        内部方法：在独立线程中运行 Viewer 循环
        """
        while self.is_running():
            self.step()

    def init_viewer(self):
        self.mujoco_handle = mujoco.viewer.launch_passive(self.model, self.data)

    def step(self):
        # 执行一次前向动力学模拟
        # mujoco.mj_forward(self.model, self.data)

        self.step_callback()

        # 将物理仿真世界向前推进一步（通常是一个时间步长）
        mujoco.mj_step(self.model, self.data)

        self.sync()

        # 时间等待
        if self.sleep_time:
            time.sleep(self.sleep_time)
    
    def pre_process(self):
        """
        MuJoCo 模型主循环前置处理
        """
        pass

    def step_callback(self):
        """
        每个时间步处理回调
        """
        pass

class CameraViewer(BaseViewer):
    def __init__(self, model_path, sleep_time = None, default_resolution=[640, 480]):
        """
        :param model_path: MJCF模型文件路径
        :param sleep_time: 每个时间步执行等待时间
        :param default_resolution: 图象默认分辨率
        """
        super().__init__(model_path, sleep_time)
        self.default_resolution = default_resolution

        self.scene = mujoco.MjvScene(self.model, maxgeom=10000)

        # 保存创建好的相机模型及相机参数
        self.cameras = {}

    def get_camera_id(self, name:str):
        """
        获取camera物体的运行时id
        
        :param name: camera 名称
        """
        camera_id = self._get_obj_id(mujoco.mjtObj.mjOBJ_CAMERA, name)
        if camera_id == -1:
            raise ValueError(f"Camera '{name}' not found")
        return camera_id
    
    def get_camera_type(self, mode:int):
        """
        相机类型表示转换
        
        :param mode: 相机mode的运行时数值
        """
        if mode == 0:
            return mujoco.mjtCamera.mjCAMERA_FIXED
        elif mode == 3:
            return mujoco.mjtCamera.mjCAMERA_TRACKING
        
        raise Exception(f'invalid mode value {mode}')
    
    def add_camera(self, camera, name, cam_type, context, resolution, window):
        self.cameras[name] = {
            'name': name,
            'camera': camera,
            'type': cam_type,
            'context': context,
            'resolution': resolution,
            'window': window
        }

    def init_camera(self, name:str):
        """
        初始化相机
        :param name: 相机名称, MJCF中定义的camera元素的name属性
        """
        # 获取相机配置参数
        cam_id = self.get_camera_id(name)
        cam_mode = self.model.cam_mode[cam_id]
        cam_type = self.get_camera_type(cam_mode)
        cam_resolution = self.model.cam_resolution[cam_id]  # 分辨率, MJCF中默认值为(1, 1, 3)
        if cam_resolution is None or cam_resolution[0] == 1:
            print(f"camera {name} resolution is not set. Using default resolution.")
            cam_resolution = np.array(self.default_resolution)
        print(f'init camera, name: {name}, type: {cam_type}, resolution: {cam_resolution}')

        # 创建相机对象
        camera = mujoco.MjvCamera()
        camera.fixedcamid = cam_id
        camera.type = cam_type
        if cam_type == mujoco.mjtCamera.mjCAMERA_TRACKING:
            track_body_id = self.model.cam_targetbodyid[cam_id]
            camera.trackbodyid = track_body_id

        # 创建OpenGL上下文
        if not glfw.init():
            return False
        window = glfw.create_window(cam_resolution[0], cam_resolution[1], name, None, None)
        if not window:
            glfw.terminate()
            return False
        glfw.make_context_current(window)
        context = mujoco.MjrContext(self.model, mujoco.mjtFontScale.mjFONTSCALE_150)

        self.add_camera(camera, name, cam_type, context, cam_resolution, window)
    
    def step_callback(self):
        for name, cam in self.cameras.items():
            camera = cam['camera']
            context = cam['context']
            resolution = cam['resolution']
            window = cam['window']

            # 将 window 指定的窗口的 OpenGL 上下文设置为当前线程的当前上下文
            glfw.make_context_current(window)

            color_img = self.get_image(camera, context, resolution[0], resolution[1])
            self.image_process_callback(name, color_img)
    
    def get_image(self, camera, context, w, h):
        """
        从相机模型获取图象
        
        :param camera: 相机模型
        :param context: 上下文
        :param w: 图象宽度
        :param h: 图象高度
        """
        # 定义视口大小
        viewport = mujoco.MjrRect(0, 0, w, h)
        # 更新场景
        mujoco.mjv_updateScene(
            self.model, self.data, mujoco.MjvOption(), 
            mujoco.MjvPerturb(), camera, mujoco.mjtCatBit.mjCAT_ALL, self.scene
        )
        # 渲染到缓冲区
        mujoco.mjr_render(viewport, self.scene, context)
        # 读取 RGB 数据（格式为 HWC, uint8）
        rgb = np.zeros((h, w, 3), dtype=np.uint8)
        depth = np.zeros((h, w), dtype=np.float64)
        mujoco.mjr_readPixels(rgb, depth, viewport, context)
        cv_image = cv2.cvtColor(np.flipud(rgb), cv2.COLOR_RGB2BGR)

        return cv_image
    
    def image_process_callback(self, name:str, color_img):
        """
        图象处理回调
        
        :param color_img: 相机名称
        :param color_img: 彩色图象数据
        """
        pass