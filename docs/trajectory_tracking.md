# 轨迹跟踪、刚体动力学与物理量输出

当前提供两种独立运行方式：原来的关节/末端操作是预检查后的运动学插值；新增的“动力学轨迹”通过理想电机广义力输入和 `mj_step` 推进物理状态。动力学跟踪过程中不把 `qpos` 写成期望角度。

## 图形界面

1. 双击项目根目录 `kinematics.cmd`，打开右侧 **动力学轨迹** 标签。
2. 先选“完整位姿曲线 · 可实现示例”和“空气 · 刚体基准”，时间缩放为 1。
3. 点击“预检查并开始力矩跟踪”。需要时自动先以关节五次时间曲线接近轨迹起点，再执行文件里的时间轨迹，最后保持终点 0.5 秒。
4. 曲线窗口显示期望/实际关节角度、末端位置、末端四元数、请求/实际关节力矩。主窗口显示当前实际位置、目标位置及误差。
5. 点击“载入 JSON / CSV”使用自定义轨迹。时间缩放大于 1 表示降低运行速度；例如 2 将 6 秒轨迹变成 12 秒，不改变路径。
6. 顶部“停止运动”暂停物理积分，保留当前状态并导出部分日志。停止不是实机急停。完成、停止、仿真故障都会保存可用数据；预检查未通过则不执行。

跟踪期间禁用运动学滑块、目标拖动和返回初始姿态，防止两种模式同时改写状态；仍可旋转/缩放相机、查看曲线。完成/停止后恢复控件。固定物理步长 0.002 秒，GUI 每次刷新执行多个物理子步；渲染或绘图变慢时，仿真墙钟运行速度可低于实时，日志时间以物理仿真为准。

## 命令行与示例

```powershell
# 完整位姿轨迹，空气刚体基准
.\.venv\Scripts\python.exe scripts\track.py trajectories\feasible_pose.json --medium air

# 指定位置闭合曲线
.\.venv\Scripts\python.exe scripts\track.py trajectories\position_loop.json --medium air

# 关节空间曲线
.\.venv\Scripts\python.exe scripts\track.py trajectories\joint_curve.json --medium air

# 加入当前基础流体阻力
.\.venv\Scripts\python.exe scripts\track.py trajectories\feasible_pose.json --medium water

# 缩放时间或指定输出目录
.\.venv\Scripts\python.exe scripts\track.py my_path.json --time-scale 2 --output outputs\my_run
```

三个示例源于 `scripts/make_trajectories.py`：`feasible_pose.json` 从已知可行的四关节平滑曲线做 FK 得到 301 个位姿点；`position_loop.json` 是 TCP 附近的闭合位置曲线；`joint_curve.json` 输入四个关节的角度曲线。示例用于软件验证，不是对实机性能的证明。

## 输入约定

支持 `position`、`pose`、`joint` 三种模式。所有时间从 0 开始，严格递增，单位秒；位置单位米。JSON 的 `frame` 可为 `world` 或 `base`（原始 `base_link`）。CSV 使用世界坐标系。输入时长默认最多 120 秒，不包括自动接近和终点保持。

位置曲线的最小 JSON 示例：

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

完整位姿模式改为 `"mode": "pose"`，每个点添加 `"quaternion": [w,x,y,z]`。四元数必须非零，程序会归一化；四元数表示 TCP 相对所选坐标系的旋转。`q` 和 `−q` 表示同一姿态，插值/角度误差处理此等价性。没有指定 `mode` 时，含 quaternion 自动识别为 pose，含 q 自动识别为 joint，否则为 position。

**Alpha 5 只有四个机械臂转动关节，无法任意指定六维位姿。** 完整位姿逆解同时约束位置和姿态，失败时拒绝整条轨迹，不会自动忽略姿态；位置模式保持腕部初始角度，日志里的“期望姿态”是参考关节解的 FK 结果，并非独立下达的姿态指令。两个完整位姿端点分别可达，也不能保证它们之间的位置/姿态插值可达。建议从密集、连续的可实现位姿点或已知关节曲线开始。

对应 CSV 表头：

```text
# 位置曲线（实际 CSV 不写这一行注释）
time_s,x_m,y_m,z_m

# 完整位姿曲线
time_s,x_m,y_m,z_m,qw,qx,qy,qz

# 关节曲线，顺序 E/D/C/B，单位弧度
time_s,q_e_rad,q_d_rad,q_c_rad,q_b_rad
```

关节 JSON 示例点用 `"q": [E,D,C,B]`，顶层 `"mode":"joint"`、`"angle_unit":"rad"` 或 `"deg"`。连续曲线接口允许在 Python 中直接提供函数，再生成采样轨迹：

```python
import numpy as np
from uvms.kinematics import ArmKinematics
from uvms.trajectory import InputTrajectory, CompiledTrajectory
from uvms.tracking import TrackingSession

arm = ArmKinematics("air")
p0 = arm.position().copy()
source = InputTrajectory.from_curve(
    duration=6,
    position_fn=lambda t: p0 + np.array([0.002*np.sin(np.pi*t/6)**2, 0, 0]),
    # 可加 quaternion_fn=lambda t: [w,x,y,z]，启用完整位姿约束
    period=0.02,
)
reference = CompiledTrajectory(arm, source)
session = TrackingSession(arm, reference)
session.run()
session.export("outputs/function_curve")
```

位置/关节输入使用分段三次 Hermite 插值，首尾速度置零；姿态使用最短弧四元数 SLERP。笛卡尔曲线按默认 0.01 秒采样做连续初值 IK，再用关节 Hermite 参考计算角度、速度、加速度。轨迹结点及中间点进行可达性、碰撞代理体、限位、速度、加速度和名义驱动力检查。不同插值段可有加速度不连续，未实现 jerk 限制；急剧变化的轨迹需平滑或放慢时间。

预定轨迹不会像单目标操作那样自动裁剪位置或改变时间；超出任务区域、不可实现姿态、过快或名义力矩超限时会明确拒绝。可以增加 `--time-scale`；自动接近阶段使用 `approach_min_duration_s` 及速度/加速度下界，需要调整接近阶段时可修改 `config/tracking.json`。离散检查不是连续碰撞可行性的数学证明。

## 刚体动力学模型与控制器

模型使用现有 MJCF 连杆质量、质心、惯性、关节 armature、重力和碰撞结构。动力学接口提供质量矩阵、偏置力、重力、科氏/离心项以及正逆动力学。夹爪两个从动关节遵循 `jaw=51×pushrod`，通过常数映射 `T` 得到五个独立广义坐标：四个关节角度与一个推杆位移。

`M_r=Tᵀ M T`、`b_r=Tᵀ b`。五个坐标的单位分别是 rad/rad/rad/rad/m，广义力分别是 N·m/N·m/N·m/N·m/N。模型含原始数值 armature，未单独部署电压、电流、反电动势或电机热模型。

```python
from uvms.dynamics import RigidBodyDynamics

dyn = RigidBodyDynamics(arm)
terms = dyn.terms(arm.home_q, qd=[0,0,0,0])
# terms['mass'] 为 5×5，bias/gravity/coriolis_centrifugal/passive 为 5 维
force = dyn.inverse(arm.home_q, [0,0,0,0], [0.1,0,0,0])
# force['torque_nm'] 为四关节力矩，gripper_force_n 为推杆力
acc = dyn.forward(arm.home_q, [0,0,0,0], force['torque_nm'], force['gripper_force_n'])
```

正逆动力学接口是理想线性夹爪约束下的约化刚体关系；实际模拟还保留 MuJoCo 软等式约束和接触力，两者会存在数值/模型差异。`bias=gravity+coriolis_centrifugal`，passive 可包含关节阻尼和当前流体作用，是否补偿由接口参数/控制器配置选择。

物理跟踪使用计算力矩控制：

`a_cmd = a_des + Kp(q_des−q_actual) + Kd(v_des−v_actual)`

`u_requested = M_r a_cmd + b_r − passive_r`（启用被动力补偿时）

最后按模型电机上限进行限幅：四关节 ±9 N·m，推杆 ±10 N。实际积分完全由 MuJoCo `mj_step` 执行。控制器参数见 `config/tracking.json`；限位/速度设置见 `config/motion.json`。运行中出现非有限状态、MuJoCo 警告、超过阈值的碰撞代理体穿透或关节限位超调会中止并导出部分数据。实际状态不通过直接裁剪 qpos/qvel 来伪造限幅。

## 输出数据

每次运行在 `outputs/tracking_…/` 或指定目录保存：

| 文件 | 内容 |
| --- | --- |
| `telemetry.csv` | 可直接用表格软件打开的物理量时间序列，UTF-8 BOM |
| `telemetry.npz` | 保留 N×4/N×3 数组的 NumPy 数据 |
| `summary.json` | 运行状态、误差统计、最大力矩、限幅次数、环境参数、时间缩放 |
| `controller.json`、`trajectory.json` | 本次控制器和已缩放时间的输入轨迹 |
| `joint_tracking.png` | 期望/实际四关节角度 |
| `tcp_position.png` | 期望/实际 TCP XYZ |
| `tcp_orientation.png` | 期望/实际 TCP 四元数分量 |
| `joint_torque.png` | 请求/实际四关节力矩 |

CSV/NPZ 包括：时间、轨迹阶段、期望/实际关节角度/速度/加速度、逆动力学前馈力矩、未限幅请求力矩、限幅命令力矩、实际广义驱动力矩、偏置/被动/约束力矩、期望/实际末端位置和四元数、位置/姿态误差、夹爪位置与推杆力、接触数量、限幅标记。

平台方块场景还输出 `object_position_m`、`object_quaternion_wxyz` 和 `robot_contact_count`。总接触数包含正常方块–桌面接触；机器人碰撞中止阈值只检查涉及机械臂本体的接触。规划时使用当前方块位置快照，实际运行时方块由物理引擎自由积分；预检查不会预测抓取过程中的物体接触运动。

抓取序列另输出 `grasp_position_act_m`（两夹指中间的夹持参考点，与 TCP 不同）、`grasp_contact_sides`（有正法向力的夹指侧数，0/1/2）、`grasp_normal_force_n`（两侧法向接触力之和）和 `grasp_contacts.png`（物体高度、推杆力、接触力及双侧接触随时间变化）。`compiled_sequence.json` 记录考虑速度约束后真正执行的各段时长；`summary.json` 的 `pick_place` 给出抬升高度、放置 XY 误差、最终物体位置、释放和桌面支撑状态。

## 抓取放置序列

GUI 的第四个示例为 `trajectories/pick_place.json`。命令行：

```powershell
.\.venv\Scripts\python.exe scripts/track.py trajectories/pick_place.json --medium air --output outputs/pick_place_air
.\.venv\Scripts\python.exe scripts/track.py trajectories/pick_place.json --medium water --output outputs/pick_place_water
```

其 `mode` 为 `pick_place`，每个 `stages` 条目包含 `phase`、`duration_s`、`gripper` 和可选的四关节 `q`（rad；省略则保持上一段角度）。`gripper` 是 0–1 的开度参考，1 为最大张开。`grip_force_n=6` 表示建立双侧接触后的闭爪推杆力幅值，施加 -6 N 以持续夹紧；松爪恢复开度控制，从测得的实际夹持开度平滑张开，避免参考从零追赶时额外挤压方块。关节与开度每段用 `10u³−15u⁴+6u⁵` 插值，阶段接点的速度、加速度为零。速度/加速度上限会自动延长用户给定时长。关节段并不保证 TCP 严格沿笛卡尔直线；需要空间直线时使用原有笛卡尔输入接口。

序列预检查初始物体位置、夹持中心与取放目标的对齐、关节/TCP 工作范围、碰撞、速度/加速度及名义电机力。物体由 free joint 真实积分，运行中不使用焊接、附着、轨迹位置覆写。允许接触仅为 close/lift/transfer/lower/release 阶段的两个接触垫与方块；实际穿透大于 1 mm、其他机械臂碰撞、物体偏离夹持点超过 30 mm，会中止。完成时必须抬升超过 40 mm、落点 XY 误差小于 10 mm、中心高度误差小于 2 mm、夹指已释放且存在桌面支撑，否则记录 failed。

抓取时机械臂与推杆反馈分别计算，避免夹指被物体阻挡时开度反馈耦合到关节参考加速度；四关节使用 Kp=900、Kd=60，搬运阶段补偿配置方块质量在夹持中心的重力。接触力和滑动仍由物理引擎确定，这个补偿没有部署完整在线负载辨识。载荷、夹指接触垫和摩擦参数均为当前模拟场景的参数。

当前基座和初始机械臂姿态保持原设置，操作台位置/高度、方块位置/朝向调整到侧向夹持分支。夹掌碰撞改用网格凸包，原夹指粗包围盒改为两块接触垫，联动和摩擦约束加强；接触参数见 `scripts/build_model.py`。四自由度腕部能力限制了可行取放姿态，需要结合实际装配和夹指尺寸进一步核验。

空气和当前基础水阻力模型均完成约 58.35 秒、5836 行的序列：方块抬升约 56.6 mm，搬运约 132.7 mm，放置 XY 误差约 0.0053 mm，释放后由桌面支撑。预检查及 GUI 启动、完成、导出均已测试。全程没有驱动力限幅，关节力矩最大约 1.49 N·m；实际夹持推杆力为 6 N。这些结果来自已知模型上的控制和接触仿真。

字段单位写在名称中，如 `q_act_rad_E`、`tau_actual_nm_D`、`position_des_m_X`、`quaternion_act_wxyz_w`。关节顺序始终 E/D/C/B。`tau_actual_nm` 是从 MuJoCo 广义驱动力读取的模拟执行器力矩，gear=1，因此与限幅命令一致；它不是硬件传感器读数，也不是包含所有外力的总关节负载。`tau_constraint_nm` 和 `tau_passive_nm` 单列表示其他作用。

日志默认每 0.01 秒记录，物理积分步长 0.002 秒。同一行中的状态、参考和力矩在同一 `time_s` 下计算，运动中的命令作用于随后的物理时间步；最后一行是终止时的状态/控制计算快照，不再推进后续积分。普通曲线 `phase` 分 `approach / tracking / settling`，抓取序列为上述九个任务阶段及 settling；`trajectory_time_s` 在进入跟踪阶段后才沿用户轨迹推进。误差摘要主要针对 tracking 阶段。位置模式/关节模式的姿态误差相对参考关节 FK 定义，不能解释为独立姿态指令跟踪精度。

## 验证结果与当前边界

当前模型更新后，空气/水下完整位姿示例均为 6 秒跟踪 + 0.5 秒终点保持，651 行日志，位置 RMS 约 **0.00198 mm**，姿态 RMS 约 **0.000970°**，没有力矩限幅。水下位置闭合曲线原时间尺度同样完成，位置 RMS 约 **0.00333 mm**，无力矩限幅。最新日志见 `outputs/validation_air_updated`、`outputs/validation_water_updated`、`outputs/validation_water_position_updated`。旧版本水下较大误差/力矩超限结果来自 TCP 占位惯性触发的异常流体阻力，不应作为当前模型性能；本次将无实体 TCP 的流体系数置零后重新验证。

这些是同一已知模型上的仿真结果，不能作为真实机械臂精度。当前水下模型仅是来源几何的基础流体近似，没有浮力、附加质量或实机标定的水动力/电机参数。后续水下定制模型、控制器鲁棒性、参数辨识、噪声/延迟等仍需继续验证。

运行测试：`.\.venv\Scripts\python.exe -m unittest discover -s tests -v`。覆盖完整位姿可行/不可行逆解、动力学矩阵和正逆关系、真实物理步进、CSV/NPZ 对齐、自动接近、力矩限幅、图形界面跟踪和实时曲线，以及原有运动学/路径检查。

MuJoCo 质量矩阵及物理积分 API 依据 [官方函数文档](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html)，偏置/被动/约束广义力约定见 [官方计算说明](https://mujoco.readthedocs.io/en/stable/computation/index.html)。
