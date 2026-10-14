# 更换机械臂对象与导入新路径

## 1. 修改当前模型

- `config/alpha5.json`：初始关节角、安装高度、积分步长、关节阻尼/armature、水参数。
- `config/scene.json`：台面、方块尺寸/质量/摩擦、取放标记、相机。
- `vendor/reach_description/`：当前模型的 URDF/Xacro、网格和原始限位。
- `scripts/build_model.py`：模型展开、网格转换、接触几何、执行器、夹爪联动、TCP 和场景生成。

运行 `scripts/build_model.py` 后重开 GUI。改变 home 后运行 `scripts/make_trajectories.py` 重建三个基础曲线示例；抓取序列需另外核对取放点、腕部方向、夹爪开度和所有阶段姿态。

## 2. 换成另一款机械臂

当前不是任意自由度模型的通用加载器。建议按以下顺序适配：

1. 准备新机械臂有权使用的 URDF/MJCF、网格及真实尺寸、质量、质心、惯性、关节限位和驱动力上限。在独立模型目录中保留来源和许可证。
2. 更新生成器或直接生成新的 MJCF：明确基座、`tool_tip`、各执行器、默认 keyframe、夹爪联动及碰撞结构。`ArmKinematics(model_path=...)` 可以加载 MJCF，但模型仍须符合当前接口所查找的名称和结构。
3. 修改 `uvms/kinematics.py` 的 `ARM_JOINTS`、基座/TCP、夹爪与地址映射；检查 FK、雅可比、位置 IK 和完整位姿 IK。当前若干输入和数值算法固定为四维，需要随自由度一起改。
4. 修改 `uvms/dynamics.py` 的独立坐标和映射矩阵。当前是四转动关节 + 推杆，两个夹指按倍率 51 联动；其他夹爪需重新定义约束与广义力单位。
5. 同步 `uvms/motion.py`、`trajectory.py`、`tracking.py` 中的维度、关节/执行器名称、增益与限幅，以及 `gui.py`、`tracking_plots.py` 的控件、曲线和日志列。
6. 更新配置、工作区域、示例轨迹和测试。先验证模型编译、惯性/限位、FK/IK 和空气刚体跟踪，再验证物体接触、水动力和电机模型。

`uvms/pick_place.py` 与 `trajectories/pick_place.json` 使用当前夹指、场景物体和四关节阶段参数；换模型后应重新设计抓取姿态和允许接触对。现有 Alpha 5 素材许可证限定 Reach Robotics 产品，不应直接复用到其他机械臂。

## 3. 导入新的位置/姿态/关节路径

在 GUI 的“动力学轨迹”页点击“载入 JSON / CSV”，然后选择环境并开始。也可以：

```powershell
.\.venv\Scripts\python.exe scripts/track.py trajectories/my_path.json --medium air --time-scale 2
```

JSON 位置轨迹示例（世界系，m、s）：

```json
{
  "mode": "position",
  "frame": "world",
  "points": [
    {"t": 0, "position": [0.11302337, -0.02909579, 0.59750556]},
    {"t": 4, "position": [0.11000000, -0.03000000, 0.60000000]}
  ]
}
```

| 模式 | JSON 点字段 | CSV 表头 |
| --- | --- | --- |
| `position` | `t`、`position: [X,Y,Z]` | `time_s,x_m,y_m,z_m` |
| `pose` | 另加 `quaternion: [w,x,y,z]` | `time_s,x_m,y_m,z_m,qw,qx,qy,qz` |
| `joint` | `t`、`q: [E,D,C,B]`；顶层 `angle_unit: rad/deg` | `time_s,q_e_rad,q_d_rad,q_c_rad,q_b_rad` |

时间必须从 0 开始且严格递增；CSV 采用世界系，关节 CSV 为弧度。JSON 支持 world/base。姿态必须符合机械臂自由度；四自由度 Alpha 5 无法跟随任意六维位姿。

公式曲线可通过 `InputTrajectory.from_curve(position_fn=..., quaternion_fn=...)` 采样输入，见 [完整接口示例](trajectory_tracking.md)。`scripts/make_trajectories.py` 演示从已知关节曲线的 FK 生成可实现位姿。

预检查会拒绝不可达、超限、碰撞或名义驱动力超限的轨迹。先从小幅、平滑的空气轨迹开始；需要减速时增大时间缩放。完成/停止后的物理量反馈与对比图保存在 `outputs/`。
