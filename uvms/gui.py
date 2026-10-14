"""Native Qt kinematics panel using the existing MuJoCo geometry and solver."""
import math
import os
import json
import time
from datetime import datetime
from pathlib import Path
import numpy as np
import mujoco
from PySide6.QtCore import Qt, QTimer, QPointF, Signal, QSignalBlocker
from PySide6.QtGui import QColor, QImage, QPainter, QPen, QFont, QFontDatabase
from PySide6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
    QHBoxLayout, QGridLayout, QLabel, QPushButton, QSlider, QDoubleSpinBox,
    QGroupBox, QComboBox, QCheckBox, QScrollArea, QFileDialog, QTabWidget)
from .kinematics import ArmKinematics, ARM_JOINTS
from .motion import MotionPlanner, MotionError
from .tracking import TrackingSession
from .pick_place import load_reference, PHASE_LABELS


class Projection:
    """Perspective projection/rays for the average MuJoCo stereo camera."""
    def __init__(self, scene, width, height):
        left, right = scene.camera
        self.origin = (left.pos.astype(float) + right.pos.astype(float)) / 2
        self.forward = left.forward.astype(float)
        self.forward /= np.linalg.norm(self.forward)
        self.up = left.up.astype(float)
        self.up /= np.linalg.norm(self.up)
        self.right = np.cross(self.forward, self.up)
        self.right /= np.linalg.norm(self.right)
        self.width, self.height = width, height
        self.focal = height * left.frustum_near / (left.frustum_top - left.frustum_bottom)

    def project(self, point):
        delta = np.asarray(point) - self.origin
        depth = float(delta @ self.forward)
        if depth <= 1e-6:
            return None
        return np.array([self.width/2 + self.focal * (delta @ self.right)/depth,
                         self.height/2 - self.focal * (delta @ self.up)/depth])

    def ray(self, pixel):
        direction = (self.forward + (pixel[0]-self.width/2)/self.focal * self.right
                     + (self.height/2-pixel[1])/self.focal * self.up)
        return self.origin, direction / np.linalg.norm(direction)

    def plane_point(self, pixel, anchor):
        origin, direction = self.ray(pixel)
        denominator = float(direction @ self.forward)
        return origin + direction * float((anchor-origin) @ self.forward) / denominator

    def axis_parameter(self, pixel, anchor, axis):
        origin, direction = self.ray(pixel)
        return np.linalg.lstsq(np.column_stack((axis, -direction)), origin-anchor, rcond=None)[0][0]


class ArmViewport(QWidget):
    targetChanged = Signal(object)
    dragFinished = Signal()
    dragStarted = Signal()

    def __init__(self, arm):
        super().__init__()
        self.arm = arm
        self.renderer = mujoco.Renderer(arm.model, height=600, width=800)
        self.camera = mujoco.MjvCamera()
        self.camera.lookat[:] = [0.12, 0, 0.38]
        self.camera.distance = 0.90
        self.camera.azimuth = 135
        self.camera.elevation = -25
        scene_path = Path(__file__).resolve().parents[1]/"config/scene.json"
        if scene_path.exists():
            camera = json.loads(scene_path.read_text(encoding="utf-8"))["camera"]
            self.camera.lookat[:] = camera["lookat_m"]
            self.camera.distance = camera["distance_m"]
            self.camera.azimuth = camera["azimuth_deg"]
            self.camera.elevation = camera["elevation_deg"]
        self.target = arm.position().copy()
        self.target_ok = True
        self.target_editable = True
        self.image = None
        self.projection = None
        self.drag = None
        self.message = ""
        self.workspace_points = np.empty((0,3))
        self.show_workspace = False
        self.planned_path = np.empty((0,3))
        self.setMinimumSize(520, 440)
        self.setMouseTracking(True)
        self.setToolTip("左键拖动橙色目标点：视平面移动；拖动 X/Y/Z 箭头：沿世界轴移动。右键旋转，滚轮缩放。")
        self.refresh()

    def refresh(self):
        self.renderer.update_scene(self.arm.data, camera=self.camera)
        self.projection = Projection(self.renderer.scene, 800, 600)
        scene = self.renderer.scene
        if scene.ngeom < scene.maxgeom:
            color = [1, 0.62, 0.15, 1] if self.target_ok else [1, 0.2, 0.3, 1]
            mujoco.mjv_initGeom(scene.geoms[scene.ngeom], mujoco.mjtGeom.mjGEOM_SPHERE,
                               np.array([0.009,0,0]), self.target, np.eye(3).ravel(), np.array(color))
            scene.ngeom += 1
        pixels = self.renderer.render()
        self.image = QImage(pixels.data, 800, 600, pixels.strides[0], QImage.Format.Format_RGB888).copy()
        self.update()

    def content_rect(self):
        scale = min(self.width()/800, self.height()/600)
        return (self.width()/2-400*scale, self.height()/2-300*scale, scale)

    def pixel(self, position):
        x, y, scale = self.content_rect()
        return np.array([(position.x()-x)/scale, (position.y()-y)/scale])

    def widget_point(self, point):
        projected = self.projection.project(point)
        if projected is None:
            return None
        x, y, scale = self.content_rect()
        return QPointF(x+projected[0]*scale, y+projected[1]*scale)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        painter.fillRect(self.rect(), QColor("#111923"))
        x, y, scale = self.content_rect()
        if self.image:
            from PySide6.QtCore import QRectF
            painter.drawImage(QRectF(x,y,800*scale,600*scale), self.image)
        if self.show_workspace and len(self.workspace_points):
            cloud = self.workspace_points[::max(1,len(self.workspace_points)//1400)]
            delta = cloud-self.projection.origin
            depth = delta @ self.projection.forward
            valid = depth > 0.01
            delta,depth = delta[valid],depth[valid]
            px = self.projection.width/2 + self.projection.focal*(delta @ self.projection.right)/depth
            py = self.projection.height/2 - self.projection.focal*(delta @ self.projection.up)/depth
            painter.setPen(QPen(QColor(69,183,240,65),2))
            painter.drawPoints([QPointF(x+u*scale,y+v*scale) for u,v in zip(px,py)])
        painter.setPen(QPen(QColor("#6ccbf5"),2))
        previous = None
        for sample in self.planned_path:
            projected = self.widget_point(sample)
            if previous is not None and projected is not None:
                painter.drawLine(previous,projected)
            previous = projected
        block_id = mujoco.mj_name2id(self.arm.model,mujoco.mjtObj.mjOBJ_BODY,"grasp_block")
        if block_id >= 0:
            block_point = self.widget_point(self.arm.data.xpos[block_id])
            if block_point is not None:
                painter.setPen(QPen(QColor("#ffbf70"),1))
                painter.drawLine(block_point,block_point+QPointF(17,-15))
                painter.drawText(block_point+QPointF(20,-17),"目标方块")
        point = self.widget_point(self.target)
        if point is not None:
            tcp = self.widget_point(self.arm.position())
            if tcp is not None:
                painter.setPen(QPen(QColor("#ffbc61"), 1, Qt.PenStyle.DashLine))
                painter.drawLine(point, tcp)
            painter.setPen(QPen(QColor("#ffbc61" if self.target_ok else "#ff526b"), 2))
            painter.setBrush(QColor("#ffbc61" if self.target_ok else "#ff526b"))
            painter.drawEllipse(point, 7, 7)
            colors = ("#ff7479", "#69dbaa", "#73afff")
            for i in range(3):
                end = self.widget_point(self.target + np.eye(3)[i]*0.065)
                if end is None:
                    continue
                painter.setPen(QPen(QColor(colors[i]), 3))
                painter.drawLine(point, end)
                painter.setBrush(QColor(colors[i]))
                painter.drawEllipse(end, 4, 4)
                painter.drawText(end + QPointF(8,-5), "XYZ"[i])
        painter.setPen(QColor("#c5d0df"))
        painter.drawText(18,26,"左键：目标点 / 轴向拖动    右键：旋转    滚轮：缩放")
        painter.setPen(QColor("#70ddb5" if self.target_ok else "#ff8f99"))
        painter.drawText(18,49,self.message.split("\n")[0])
        painter.end()

    @staticmethod
    def line_distance(pixel, start, end):
        vector = end-start
        length2 = vector @ vector
        if length2 < 1e-6:
            return float("inf")
        factor = np.clip(((pixel-start) @ vector)/length2, 0.15, 1.1)
        return np.linalg.norm(pixel-(start+factor*vector))

    def mousePressEvent(self, event):
        pixel = self.pixel(event.position())
        if event.button() == Qt.MouseButton.RightButton:
            self.drag = {"mode": "orbit", "pixel": pixel}
            return
        if event.button() != Qt.MouseButton.LeftButton:
            return
        if not self.target_editable:
            return
        center = self.projection.project(self.target)
        if center is None:
            return
        mode = None
        if np.linalg.norm(pixel-center) <= 13:
            mode = "plane"
        else:
            hits = []
            for i in range(3):
                end = self.projection.project(self.target+np.eye(3)[i]*0.065)
                if end is not None and np.linalg.norm(end-center) >= 12:
                    hits.append((self.line_distance(pixel, center, end), i))
            if hits and min(hits)[0] < 10:
                mode = min(hits)[1]
        if mode is not None:
            self.drag = {"mode": mode, "pixel": pixel, "anchor": self.target.copy()}
            self.dragStarted.emit()
            self.setCursor(Qt.CursorShape.ClosedHandCursor)

    def mouseMoveEvent(self, event):
        if self.drag is None:
            return
        pixel = self.pixel(event.position())
        mode = self.drag["mode"]
        if mode == "orbit":
            delta = pixel-self.drag["pixel"]
            self.camera.azimuth -= delta[0]*0.4
            self.camera.elevation = np.clip(self.camera.elevation-delta[1]*0.3, -85, 85)
            self.drag["pixel"] = pixel
            self.refresh()
            return
        anchor = self.drag["anchor"]
        if mode == "plane":
            target = anchor + self.projection.plane_point(pixel, anchor) - self.projection.plane_point(self.drag["pixel"], anchor)
        else:
            axis = np.eye(3)[mode]
            delta = self.projection.axis_parameter(pixel, anchor, axis) - self.projection.axis_parameter(self.drag["pixel"], anchor, axis)
            target = anchor+axis*delta
        self.target = target
        self.targetChanged.emit(target.copy())

    def mouseReleaseEvent(self, event):
        was_target = self.drag is not None and self.drag["mode"] != "orbit"
        self.drag = None
        self.unsetCursor()
        if was_target:
            self.dragFinished.emit()

    def wheelEvent(self, event):
        if self.drag is not None:
            return
        self.camera.distance = np.clip(self.camera.distance * math.exp(-event.angleDelta().y()/1200), 0.25, 3)
        self.refresh()

    def close_renderer(self):
        self.renderer.close()


class KinematicsWindow(QMainWindow):
    def __init__(self, medium="water"):
        super().__init__()
        # Offscreen Qt has no automatic font discovery on Windows.
        if not QFontDatabase.families():
            fonts = Path(os.environ.get("WINDIR", "C:/Windows")) / "Fonts"
            for name in ("msyh.ttc", "consola.ttf"):
                if (fonts/name).exists():
                    QFontDatabase.addApplicationFont(str(fonts/name))
        self.setFont(QFont("Microsoft YaHei", 10))
        self.arm = ArmKinematics(medium)
        self.arm.model.vis.headlight.ambient[:] = [0.4,0.4,0.4]
        self.arm.model.vis.headlight.diffuse[:] = [0.7,0.7,0.7]
        self.planner = MotionPlanner(self.arm)
        self.motion = None
        self.motion_elapsed = 0.0
        self.motion_clock = 0.0
        self.goal_q = self.arm.home_q.copy()
        self.target_clipped = False
        self.tracking = None
        self.tracking_monitor = None
        self.tracking_output = None
        self.tracking_ticks = 0
        self.water_parameters = (1000.0,0.001)
        self.root_path = Path(__file__).resolve().parents[1]
        self.loaded_trajectory = self.root_path/"trajectories/feasible_pose.json"
        self.frame = "world"
        self.pending_target = None
        self.setWindowTitle("Alpha 5 · 运动学 / 动力学工作台")
        self.resize(1200, 820)
        root = QWidget()
        self.setCentralWidget(root)
        layout = QHBoxLayout(root)
        self.viewport = ArmViewport(self.arm)
        self.viewport.workspace_points = self.planner.workspace_cloud()
        layout.addWidget(self.viewport, 1)
        panel = QWidget()
        panel.setFixedWidth(350)
        layout.addWidget(panel)
        controls = QVBoxLayout(panel)
        title = QLabel("Alpha 5 · 机械臂仿真")
        title.setStyleSheet("font-size:23px; font-weight:600; margin:6px 0 8px;")
        controls.addWidget(title)
        outer_controls = controls
        self.control_tabs = QTabWidget()
        controls.addWidget(self.control_tabs,1)
        kinematic_page,tracking_page = QWidget(),QWidget()
        kinematic_controls,tracking_controls = QVBoxLayout(kinematic_page),QVBoxLayout(tracking_page)
        for title,page in (("关节 / 末端",kinematic_page),("动力学轨迹",tracking_page)):
            page_scroll = QScrollArea()
            page_scroll.setWidgetResizable(True)
            page_scroll.setWidget(page)
            page_scroll.setFrameShape(QScrollArea.Shape.NoFrame)
            self.control_tabs.addTab(page_scroll,title)
        controls = kinematic_controls
        tracking_box = QGroupBox("动力学轨迹 · 文件输入")
        tracking_layout = QGridLayout(tracking_box)
        self.example_select = QComboBox()
        self.example_select.addItems(["完整位姿曲线 · 可实现示例","位置闭合曲线","关节空间曲线","抓取方块 → 绿色放置区"])
        self.example_select.currentIndexChanged.connect(self.select_example)
        tracking_layout.addWidget(self.example_select,0,0,1,2)
        self.trajectory_label = QLabel(self.loaded_trajectory.name)
        self.trajectory_label.setWordWrap(True)
        tracking_layout.addWidget(self.trajectory_label,1,0,1,2)
        self.load_trajectory_button = QPushButton("载入 JSON / CSV")
        self.load_trajectory_button.clicked.connect(self.load_trajectory_file)
        tracking_layout.addWidget(self.load_trajectory_button,2,0,1,2)
        self.physics_medium = QComboBox()
        self.physics_medium.addItems(["空气 · 刚体基准","水下 · 基础流体阻力"])
        tracking_layout.addWidget(self.physics_medium,3,0,1,2)
        self.time_scale_input = QDoubleSpinBox()
        self.time_scale_input.setRange(.1,20)
        self.time_scale_input.setValue(1)
        self.time_scale_input.setSuffix(" × 时间")
        tracking_layout.addWidget(QLabel("轨迹时间缩放"),4,0)
        tracking_layout.addWidget(self.time_scale_input,4,1)
        self.start_tracking_button = QPushButton("预检查并开始力矩跟踪")
        self.start_tracking_button.clicked.connect(lambda:self.start_tracking())
        tracking_layout.addWidget(self.start_tracking_button,5,0,1,2)
        self.monitor_button = QPushButton("期望 / 实际 / 力矩曲线")
        self.monitor_button.clicked.connect(self.show_tracking_monitor)
        tracking_layout.addWidget(self.monitor_button,6,0,1,2)
        self.telemetry_label = QLabel("日志：CSV / NPZ / 误差摘要 / 对比图\n跟踪时物理时间推进；停止时暂停仿真。")
        self.telemetry_label.setWordWrap(True)
        tracking_layout.addWidget(self.telemetry_label,7,0,1,2)
        tracking_controls.addWidget(tracking_box)
        tracking_controls.addStretch()
        joints = QGroupBox("关节目标 · 限幅 / °")
        joints_layout = QGridLayout(joints)
        self.sliders, self.angle_inputs = [], []
        for i, name in enumerate(ARM_JOINTS):
            low, high = np.rad2deg(self.arm.limits[i])
            slider = QSlider(Qt.Orientation.Horizontal)
            slider.setRange(math.ceil(low*100), math.floor(high*100))
            slider.setSingleStep(10)
            slider.setObjectName("slider_"+name)
            slider.setToolTip(f"目标角度：{low:.3f} ～ {high:.3f} °；机械臂将线性运动到目标")
            number = QDoubleSpinBox()
            number.setRange(low, high)
            number.setDecimals(3)
            number.setSingleStep(0.5)
            number.setSuffix(" °")
            number.setObjectName("angle_"+name)
            number.setFixedWidth(94)
            joints_layout.addWidget(QLabel(name), i,0)
            joints_layout.addWidget(slider, i,1)
            joints_layout.addWidget(number, i,2)
            slider.valueChanged.connect(lambda value, index=i: self.set_joint(index, value/100))
            number.valueChanged.connect(lambda value, index=i: self.set_joint(index, value))
            self.sliders.append(slider)
            self.angle_inputs.append(number)
        joints_layout.setColumnStretch(1,1)
        controls.addWidget(joints)
        tcp = QGroupBox("当前末端 TCP · m")
        tcp_layout = QVBoxLayout(tcp)
        self.coordinates = QLabel()
        self.coordinates.setStyleSheet("font-family:Consolas; font-size:18px; line-height:1.5;")
        tcp_layout.addWidget(self.coordinates)
        controls.addWidget(tcp)
        inputs = QGroupBox("目标位置 · 逆运动学")
        inputs_layout = QGridLayout(inputs)
        self.frame_select = QComboBox()
        self.frame_select.addItems(["世界坐标 / world", "基座坐标 / base_link"])
        self.frame_select.currentIndexChanged.connect(self.change_frame)
        inputs_layout.addWidget(self.frame_select,0,0,1,2)
        self.xyz_inputs = []
        for i, axis in enumerate("XYZ"):
            spin = QDoubleSpinBox()
            spin.setRange(-10,10)
            spin.setDecimals(6)
            spin.setSingleStep(0.005)
            spin.setSuffix(" m")
            spin.setObjectName("target_"+axis)
            inputs_layout.addWidget(QLabel(axis),i+1,0)
            inputs_layout.addWidget(spin,i+1,1)
            self.xyz_inputs.append(spin)
        self.reject_contacts = QCheckBox("检查整条路径的碰撞代理体")
        self.reject_contacts.setChecked(True)
        inputs_layout.addWidget(self.reject_contacts,4,0,1,2)
        self.solve_button = QPushButton("求解并更新姿态")
        self.solve_button.setObjectName("solve_target")
        self.solve_button.clicked.connect(self.solve_inputs)
        inputs_layout.addWidget(self.solve_button,5,0,1,2)
        align = QPushButton("目标点对齐当前 TCP")
        self.align_button = align
        align.clicked.connect(self.align_target)
        inputs_layout.addWidget(align,6,0,1,2)
        controls.addWidget(inputs)
        motion_controls = QGroupBox("线性运动 · 限幅")
        motion_layout = QGridLayout(motion_controls)
        self.duration_input = QDoubleSpinBox()
        self.duration_input.setRange(0.1,20)
        self.duration_input.setSingleStep(0.1)
        self.duration_input.setValue(self.planner.config["requested_duration_s"])
        self.duration_input.setSuffix(" s")
        motion_layout.addWidget(QLabel("期望运动时间"),0,0)
        motion_layout.addWidget(self.duration_input,0,1)
        speed = QLabel("关节 ≤ "+"/".join(f"{v:g}" for v in self.planner.config["joint_speed_deg_s"])+" °/s\n"
                       +f"TCP ≤ {self.planner.config['tcp_speed_m_s']:g} m/s；超限自动延长时间\n"
                       +"目标受关节限位、任务区域及可达性检查约束")
        speed.setWordWrap(True)
        motion_layout.addWidget(speed,1,0,1,2)
        controls.addWidget(motion_controls)
        self.status = QLabel("就绪")
        self.status.setWordWrap(True)
        self.status.setMinimumHeight(45)
        outer_controls.addWidget(self.status)
        tip = QLabel("蓝色点云：已采样可行区域（腕部初值），不是完整可达体积。\n蓝线：预检查路径。位置运动保持腕部角度；直线无法通过时拒绝执行。")
        tip.setWordWrap(True)
        tip.setStyleSheet("color:#8498b0; font-size:12px;")
        controls.addWidget(tip)
        controls.addStretch()
        toolbar = self.addToolBar("运动")
        toolbar.setMovable(False)
        self.stop_action = toolbar.addAction("停止运动",lambda checked=False:self.stop_motion())
        self.home_action = toolbar.addAction("线性返回初始姿态",self.home)
        workspace_action = toolbar.addAction("显示采样工作区域")
        workspace_action.setCheckable(True)
        workspace_action.setChecked(False)
        workspace_action.toggled.connect(self.toggle_workspace)
        self.reset_block_action = toolbar.addAction("重置方块",self.reset_block)
        self.setStyleSheet("""
            QWidget { background:#17212e; color:#dce7f4; font-size:13px; }
            QGroupBox { border:1px solid #344456; border-radius:8px; margin-top:16px; padding:12px 8px 8px; }
            QGroupBox::title { subcontrol-origin:margin; left:10px; color:#94b9dc; }
            QDoubleSpinBox,QComboBox { background:#0f1823; padding:5px; border:1px solid #35485d; border-radius:4px; }
            QPushButton { background:#29445e; padding:9px; border:0; border-radius:5px; }
            QPushButton:hover { background:#365b7e; }
            QSlider::groove:horizontal { height:5px; background:#34485d; border-radius:2px; }
            QSlider::handle:horizontal { width:15px; margin:-5px 0; background:#63c5d6; border-radius:7px; }
        """)
        self.drag_timer = QTimer(self)
        self.drag_timer.setInterval(33)
        self.drag_timer.timeout.connect(self.process_drag)
        self.viewport.targetChanged.connect(self.queue_drag)
        self.viewport.dragFinished.connect(self.finish_drag)
        self.viewport.dragStarted.connect(lambda:setattr(self,"target_clipped",False))
        self.motion_timer = QTimer(self)
        self.motion_timer.setInterval(33)
        self.motion_timer.timeout.connect(self.advance_motion)
        self.tracking_timer = QTimer(self)
        self.tracking_timer.setInterval(33)
        self.tracking_timer.timeout.connect(self.advance_tracking)
        self.reject_contacts.toggled.connect(lambda checked:self.stop_motion())
        self.sync_state()
        self.update_input_limits()
        self.update_inputs()

    def sync_state(self, sync_joints=True):
        state = self.arm.state(self.frame)
        angles = state["q_deg"] if sync_joints else np.rad2deg(self.goal_q)
        for i, angle in enumerate(angles):
            with QSignalBlocker(self.sliders[i]), QSignalBlocker(self.angle_inputs[i]):
                self.sliders[i].setValue(round(angle*100))
                self.angle_inputs[i].setValue(angle)
        self.coordinates.setText("\n".join(f"{axis}  {value: .6f}" for axis,value in zip("XYZ",state["position_m"])))
        return state

    def update_inputs(self):
        origin, rotation = self.arm._frame(self.frame)
        target = rotation.T @ (self.viewport.target-origin)
        for spin, value in zip(self.xyz_inputs, target):
            spin.setValue(float(value))

    def set_joint(self, index, value):
        if self.tracking is not None and self.tracking.status == "running":
            return
        self.cancel_drag()
        q = self.goal_q.copy() if self.motion is not None else self.arm.data.qpos[self.arm.q_indices].copy()
        q[index] = np.deg2rad(value)
        try:
            plan = self.planner.joint(q,duration=self.duration_input.value(),reject_contacts=self.reject_contacts.isChecked())
            self.viewport.target = plan.xyz[-1].copy()
            self.start_motion(plan)
        except MotionError as exc:
            self.motion_failure(str(exc))

    def change_frame(self, index):
        self.frame = "world" if index == 0 else "base"
        self.sync_state(self.motion is None)
        self.update_input_limits()
        self.update_inputs()

    def update_input_limits(self):
        origin,rotation = self.arm._frame(self.frame)
        corners = np.array([[x,y,z] for x in (self.planner.low[0],self.planner.high[0])
                            for y in (self.planner.low[1],self.planner.high[1])
                            for z in (self.planner.low[2],self.planner.high[2])])
        bounds = (corners-origin) @ rotation
        for i,spin in enumerate(self.xyz_inputs):
            spin.setRange(float(bounds[:,i].min()),float(bounds[:,i].max()))
            spin.setToolTip(f"任务范围：{bounds[:,i].min():.3f} ～ {bounds[:,i].max():.3f} m；还需可达性和路径检查")

    def solve_inputs(self):
        if self.tracking is not None and self.tracking.status == "running":
            return
        self.cancel_drag()
        origin, rotation = self.arm._frame(self.frame)
        self.viewport.target = origin + rotation @ np.array([s.value() for s in self.xyz_inputs])
        self.target_clipped = False
        self.solve_target(full=True)

    def solve_target(self, full=False):
        target,clipped = self.planner.clamp_target(self.viewport.target)
        self.viewport.target = target
        self.target_clipped |= clipped
        try:
            plan = self.planner.cartesian(target,duration=self.duration_input.value(),reject_contacts=self.reject_contacts.isChecked())
            plan.clipped |= self.target_clipped
            self.start_motion(plan)
            return {"success":True,"duration_s":plan.duration}
        except MotionError as exc:
            self.motion_failure(str(exc))
            return {"success":False,"reason":str(exc)}

    def start_motion(self,plan):
        self.motion = plan
        self.motion_elapsed = 0
        self.motion_clock = time.monotonic()
        self.goal_q = plan.q[-1].copy()
        self.viewport.planned_path = plan.xyz.copy()
        self.viewport.target_ok = True
        self.target_clipped = plan.clipped
        self.sync_state(False)
        self.update_inputs()
        self.show_motion_status(0)
        self.viewport.refresh()
        self.motion_timer.start()

    def show_motion_status(self,progress):
        name = "末端直线" if self.motion.kind == "cartesian" else "关节线性"
        self.status.setText(f"{name}运动 {progress*100:.0f}% · {self.motion.duration:.2f} s"
                            + (" · 目标已限幅" if self.motion.clipped else ""))
        self.status.setStyleSheet("color:#70ddb5;")
        self.viewport.message = self.status.text()

    def advance_motion(self):
        if self.motion is None:
            return
        now = time.monotonic()
        self.motion_elapsed += min(max(now-self.motion_clock,0),0.05)
        self.motion_clock = now
        q,progress = self.motion.at(self.motion_elapsed)
        opening = float(self.arm.data.qpos[self.arm.pushrod]/self.arm.gripper_max)
        self.arm.forward(q,unit="rad",gripper=opening)
        self.sync_state(False)
        self.show_motion_status(progress)
        if progress >= 1:
            self.motion_timer.stop()
            self.motion = None
            self.goal_q = q.copy()
            self.sync_state()
            self.status.setText("已完成线性运动" + (" · 目标已限幅" if self.target_clipped else ""))
            self.viewport.message = self.status.text()
        self.viewport.refresh()

    def motion_failure(self,message):
        self.stop_motion(align=False)
        self.viewport.target_ok = False
        self.status.setText(message+"\n保留当前姿态")
        self.status.setStyleSheet("color:#ff8f99;")
        self.viewport.message = self.status.text()
        self.sync_state()
        self.update_inputs()
        self.viewport.refresh()

    def stop_motion(self,align=True):
        if self.tracking is not None and self.tracking.status == "running":
            self.stop_tracking()
        self.cancel_drag()
        self.motion_timer.stop()
        self.motion = None
        self.viewport.planned_path = np.empty((0,3))
        self.goal_q = self.arm.data.qpos[self.arm.q_indices].copy()
        if align:
            self.align_target()
            self.status.setText("已停止，保留当前运动位置")
            self.viewport.message = self.status.text()
            self.sync_state()
            self.viewport.update()

    def toggle_workspace(self,visible):
        self.viewport.show_workspace = visible
        self.viewport.update()

    def reset_block(self):
        if self.tracking is not None and self.tracking.status == "running":
            return
        self.stop_motion()
        joint = self.arm.model.joint("block_free").id
        qadr,vadr = self.arm.model.jnt_qposadr[joint],self.arm.model.jnt_dofadr[joint]
        self.arm.data.qpos[qadr:qadr+7] = self.arm.model.key_qpos[0,qadr:qadr+7]
        self.arm.data.qvel[vadr:vadr+6] = 0
        mujoco.mj_forward(self.arm.model,self.arm.data)
        self.status.setText("方块已放回操作台取物区")
        self.viewport.message = self.status.text()
        self.viewport.refresh()

    def queue_drag(self, target):
        self.pending_target,clipped = self.planner.clamp_target(target)
        self.viewport.target = self.pending_target.copy()
        self.target_clipped |= clipped
        self.update_inputs()
        self.viewport.update()
        if not self.drag_timer.isActive():
            self.drag_timer.start()

    def process_drag(self):
        if self.pending_target is not None:
            self.viewport.target = self.pending_target
            self.pending_target = None
            self.solve_target()

    def finish_drag(self):
        had_pending = self.pending_target is not None
        self.process_drag()
        self.drag_timer.stop()
        # Keep drag solutions local/continuous rather than switching IK branches.
        if not had_pending and self.motion is None:
            self.solve_target()

    def cancel_drag(self):
        self.pending_target = None
        self.drag_timer.stop()

    def align_target(self):
        if self.tracking is not None and self.tracking.status == "running":
            return
        self.cancel_drag()
        self.motion_timer.stop()
        self.motion = None
        self.goal_q = self.arm.data.qpos[self.arm.q_indices].copy()
        self.target_clipped = False
        self.sync_state()
        self.viewport.target = self.arm.position().copy()
        self.viewport.target_ok = True
        self.update_inputs()
        self.status.setText("目标点已对齐当前 TCP")
        self.viewport.message = self.status.text()
        self.status.setStyleSheet("color:#70ddb5;")
        self.viewport.refresh()

    def home(self):
        if self.tracking is not None and self.tracking.status == "running":
            return
        self.cancel_drag()
        try:
            plan = self.planner.joint(self.arm.home_q,duration=self.duration_input.value(),reject_contacts=self.reject_contacts.isChecked())
            self.viewport.target = plan.xyz[-1].copy()
            self.start_motion(plan)
        except MotionError as exc:
            self.motion_failure(str(exc))

    def select_example(self,index):
        name = ("feasible_pose","position_loop","joint_curve","pick_place")[index]
        self.loaded_trajectory = self.root_path/"trajectories"/(name+".json")
        self.trajectory_label.setText(self.loaded_trajectory.name)

    def load_trajectory_file(self):
        path,_ = QFileDialog.getOpenFileName(self,"载入预定轨迹",str(self.root_path/"trajectories"),"Trajectory (*.json *.csv)")
        if path:
            self.loaded_trajectory = Path(path)
            self.trajectory_label.setText(self.loaded_trajectory.name)

    def show_tracking_monitor(self):
        if self.tracking_monitor is None:
            from .tracking_plots import make_live_window
            self.tracking_monitor = make_live_window(self)
        self.tracking_monitor.show()
        self.tracking_monitor.raise_()
        if self.tracking is not None:
            self.tracking_monitor.refresh(self.tracking)

    def tracking_controls(self,active):
        widgets = (self.sliders+self.angle_inputs+self.xyz_inputs+
            [self.solve_button,self.duration_input,self.reject_contacts,self.example_select,
             self.physics_medium,self.time_scale_input,self.start_tracking_button,self.load_trajectory_button])
        widgets.append(self.align_button)
        for widget in widgets:
            widget.setEnabled(not active)
        self.viewport.target_editable = not active
        self.home_action.setEnabled(not active)
        self.reset_block_action.setEnabled(not active)

    def start_tracking(self,path=None,output=None,show_monitor=True):
        self.stop_motion()
        self.arm.model.opt.density,self.arm.model.opt.viscosity = self.water_parameters if self.physics_medium.currentIndex() else (0,0)
        path = Path(path) if path is not None else self.loaded_trajectory
        try:
            reference = load_reference(self.arm,path,self.time_scale_input.value())
            self.tracking = TrackingSession(self.arm,reference)
        except (ValueError,KeyError,OSError) as exc:
            self.status.setText("轨迹预检查失败："+str(exc))
            self.viewport.message = self.status.text()
            self.viewport.target_ok = False
            self.viewport.update()
            return False
        self.tracking_output = Path(output) if output is not None else self.root_path/"outputs"/f"tracking_{datetime.now():%Y%m%d_%H%M%S_%f}"
        self.tracking_ticks = 0
        self.viewport.target_ok = True
        times = np.linspace(0,reference.duration,min(500,int(reference.duration/.02)+2))
        self.viewport.planned_path = np.array([reference.sample(t)["position"] for t in times])
        self.tracking_controls(True)
        self.status.setText("动力学跟踪已启动；CSV 与曲线将在完成/停止后导出。")
        self.viewport.message = self.status.text()
        if show_monitor:
            self.show_tracking_monitor()
        self.tracking_timer.start()
        return True

    def advance_tracking(self):
        if self.tracking is None or self.tracking.status != "running":
            return
        try:
            # Fixed dt physics substeps; slow rendering slows wall time, not simulation time.
            for _ in range(max(1,round(.033/self.arm.model.opt.timestep))):
                self.tracking.step()
                if self.tracking.status != "running":
                    break
        except RuntimeError as exc:
            self.status.setText("物理跟踪失败："+str(exc))
        row = self.tracking.last
        if row is not None:
            self.goal_q = row["q_des_rad"].copy()
            self.viewport.target = row["position_des_m"].copy()
            self.sync_state(False)
            self.update_inputs()
            phase = PHASE_LABELS.get(row['phase'],row['phase'])
            self.status.setText(f"物理跟踪 {self.tracking.time:.2f}/{self.tracking.end_time:.2f} s · {phase}\n"
                                f"位置误差 {row['position_error_m']*1000:.3f} mm · 姿态误差 {np.rad2deg(row['orientation_error_rad']):.3f}°")
            self.viewport.message = self.status.text()
            self.telemetry_label.setText("实际关节力矩 / N·m\n"+" / ".join(f"{v:.3f}" for v in row["tau_actual_nm"]))
        self.tracking_ticks += 1
        if self.tracking_monitor is not None and self.tracking_ticks % 8 == 0:
            self.tracking_monitor.refresh(self.tracking)
        self.viewport.refresh()
        if self.tracking.status != "running":
            self.finish_tracking()

    def finish_tracking(self,plots=True):
        self.tracking_timer.stop()
        self.tracking_controls(False)
        if self.tracking is None:
            return
        self.tracking.export(self.tracking_output,plots=plots)
        self.arm.data.ctrl[:] = 0
        if self.tracking_monitor is not None:
            self.tracking_monitor.refresh(self.tracking)
        summary = self.tracking.summary()
        self.status.setText(f"物理跟踪 {summary['status']} · 位置 RMS {summary.get('position_rmse_m',0)*1000:.3f} mm\n"
                            + (summary.get("failure") or ""))
        if "pick_place" in summary:
            task = summary["pick_place"]
            self.status.setText(self.status.text()+f"\n抓取放置{'成功' if task['success'] else '未完成'} · 抬升 {task['lift_height_m']*1000:.1f} mm · 落点误差 {task['place_xy_error_m']*1000:.1f} mm")
        self.viewport.message = self.status.text()
        self.telemetry_label.setText("已导出：\n"+str(self.tracking_output))
        self.goal_q = self.arm.data.qpos[self.arm.q_indices].copy()
        self.sync_state()

    def stop_tracking(self,plots=True):
        if self.tracking is not None and self.tracking.status == "running":
            self.tracking.stop()
            self.finish_tracking(plots)

    def closeEvent(self, event):
        self.stop_tracking()
        self.drag_timer.stop()
        self.motion_timer.stop()
        self.tracking_timer.stop()
        self.viewport.close_renderer()
        super().closeEvent(event)


def main():
    app = QApplication.instance() or QApplication([])
    window = KinematicsWindow()
    window.show()
    return app.exec()
