# 水下电动机械臂仿真基础工程

第一阶段已搭建：Windows + Python 3.12.14 + MuJoCo 3.14.0，模型采用 **Reach Alpha 5 五功能水下电动机械臂**。当前是固定基座机械臂模型，用于后续运动学、动力学和自定义电机/水动力算法开发。

现已加入完整位姿逆运动学、约化刚体正逆动力学、预定轨迹的力矩跟踪以及物理量日志/实时曲线。完整说明见 [轨迹跟踪与输出数据](trajectory_tracking.md)。图形界面右侧“动力学轨迹”标签可加载 JSON/CSV，默认提供已知可实现的完整位姿示例。Alpha 5 四个转动关节不能任意实现六维末端位姿，输入轨迹会检查此约束。

用户最初提及“日本 SR5”，随后允许以可获得的最佳公开素材选型；本工程选择有原始结构、网格和惯性数据的 Alpha 5，不将其标记为日本 SR5。结构资料来自独立维护者的 Reach 描述包，而非经实机标定的完整数字孪生。

## 平台方块场景

当前默认场景为“固定基座机械臂 + 操作台 + 可运动方块”。为满足四自由度机械臂的侧向夹持路径，操作台调整为 0.42×0.30 m、台面高度约 0.27468 m；橙色方块仍为边长 0.03 m、质量 0.06 kg。初始世界位置约 `[-0.01906, -0.27705, 0.29028] m`，绕世界 Z 转角 1 rad。黄色标线标识取料点；绿色区的放置中心约 `[0.10463, -0.32514, 0.28968] m`。方块带六自由度 free joint，通过接触、摩擦、重力自由运动。

配置在 `config/scene.json`。修改后运行 `.\.venv\Scripts\python.exe scripts\build_model.py`，再重新打开 GUI。顶部“重置方块”可把方块放回取物区；动力学运行期间禁用此按钮。视角已按操作台与机械臂共同取景，工作区点云可通过顶部开关显示。

桌面/桌腿和方块均参与碰撞；正常方块–桌面接触不被当作机械臂碰撞失败，机械臂与桌面/方块的接触仍检测。规划探针会复制当前方块状态，运动学输入不会重置方块位置/速度。控制器仍按名称访问机械臂关节，新增 free joint 不占用机器人五路电机。

自动抓取放置：双击 `kinematics.cmd` → “动力学轨迹” → 选择“抓取方块 → 绿色放置区” → 点击“预检查并开始力矩跟踪”。阶段为张爪、接近、下降、闭爪、抬升、搬运、下降放置、松爪、撤离，默认约 58.35 秒（含终点保持）。每段采用五次时间插值，起止速度/加速度为零；关节/TCP 速度和加速度要求会延长阶段时间。四关节和夹爪由电机广义力控制，方块通过物理接触搬运。重复运行前点击“重置方块”。当前机械臂姿态若不能通过接近路径预检查，先返回初始姿态。

序列参数在 `trajectories/pick_place.json`，包括各阶段关节角度、夹爪开度、时长、夹持力和控制器增益；实际采用的时间表输出到 `compiled_sequence.json`。更改取放位置时须同时调整场景、阶段姿态与预期取放坐标，预检查会核验夹持中心是否对齐目标。原有运动学和 JSON/CSV 轨迹接口继续可用。

空气和基础水阻力环境均已验证：抬升约 **56.6 mm**、搬运约 **132.7 mm**、最终 XY 落点误差约 **0.0053 mm**，方块释放后由桌面支撑。结果保存在 `outputs/pick_place_air`、`outputs/pick_place_water`。运行时只允许闭爪/搬运/释放阶段的夹指接触垫–方块接触，实际允许穿透不超过 1 mm；其他机械臂碰撞仍中止。方块远离夹持中心超过 30 mm 或最终抬升/放置/释放验证失败，会停止并输出失败结果。这些精度只描述当前已知参数仿真。

命令行运行：`.\.venv\Scripts\python.exe scripts/track.py trajectories/pick_place.json --medium air --output outputs/pick_place_air`。水环境使用 `--medium water`。

## 直接运行

在项目目录的 PowerShell 中运行，无需激活虚拟环境：

```powershell
# 静态交互预览：旋转、缩放查看模型，关闭窗口退出
.\.venv\Scripts\python.exe scripts\simulate.py --medium water --viewer

# 模型/环境检查及初始姿态截图
.\.venv\Scripts\python.exe scripts\simulate.py --medium air --render
.\.venv\Scripts\python.exe scripts\simulate.py --medium water --render

# 修改 config/alpha5.json 后重新生成
.\.venv\Scripts\python.exe scripts\build_model.py
```

也可双击 `preview.cmd` 打开水下场景静态预览。预览不施加控制器，也不推进物理时间。`--viewer` 的交互窗口入口已提供，当前验证覆盖离屏 OpenGL 渲染，未人工验证交互窗口操作。

检查结果和截图保存在 `outputs/check_air.json`、`outputs/check_water.json`、`outputs/arm_air.png`、`outputs/arm_water.png`。`--seconds 2` 是默认短时检查时长；运行时施加零输入，机械臂在重力作用下运动和接触，不保持初始姿态。

## 运动学输入接口

双击 `kinematics.cmd` 打开 **原生图形界面**，或运行：

```powershell
.\.venv\Scripts\python.exe scripts\gui.py
```

界面同时包含关节滑块、当前 TCP 坐标、目标 XYZ 输入和三维拖动手柄：

- **关节空间**：拖动四个 `axis_e / axis_d / axis_c / axis_b` 滑块，设置限位内的目标角度，再从当前角度线性运动到目标。角度单位为度；滑块旁的数字框可精确输入目标角度。运动期间滑块显示目标角度，TCP 数值显示当前实际位置；目标点显示终点。
- **位置逆解**：填写 X、Y、Z（米），点击“求解并更新姿态”。支持世界/基座坐标显示；目标位置不因切换坐标系改变。先逐点检查从当前 TCP 到目标的空间直线路径，通过后播放运动，更新 TCP 数值和运动进度。
- **三维目标拖动**：左键拖动橙色点，在当前相机视平面内移动目标；拖动红 X、绿 Y、蓝 Z 手柄，沿对应世界轴移动。手柄始终使用世界轴，与输入框坐标系选择无关。视角使某轴接近正对相机时，可先右键旋转以露出该轴手柄。
- **视图操作**：右键旋转相机，滚轮缩放。顶部“停止运动”停在当前实际位置；“线性返回初始姿态”按关节插值返回。“目标点对齐当前 TCP”会停止运动并对齐目标。小窗口时右侧面板可滚动，运动状态也显示在三维视图顶部。
- **失败状态**：直线路径上的任一点无法求逆解、发生关节突变、离开任务范围，或勾选碰撞检查后出现穿透，目标点变红，保留当前实际姿态。端点可达并不表示连接端点的直线可行；界面不会自动切换其他路径。拖动采用当前姿态初值、局部逐点逆解和约 33 ms 的输入合并周期，以减少分支跳变。实际刷新率取决于显卡、渲染和求解耗时。

GUI 使用 PySide6 6.10.2 和现有 MuJoCo 模型离屏渲染，依赖已加入环境锁定文件。本页“关节 / 末端”操作包含运动学路径检查和定时插值播放；“动力学轨迹”页则以理想电机力矩输入推进物理积分并输出跟踪数据。TCP/关节限位和碰撞代理体定义沿用下文约定。运动学页默认检查整条关节/末端路径的碰撞代理体；取消勾选可关闭该模式的碰撞拒绝，但仍检查限位和任务范围。切换该选项时停止当前运动。腕部 `axis_b` 不影响 TCP 位置，运动学拖动/位置求解时保持其当前角度。

### 限幅、工作区域和线性运动

参数文件为 `config/motion.json`，修改后重新打开图形界面生效。界面的“线性运动 · 限幅”提供期望运动时间设置；配置中的速度上限若要求更长时间，会自动延长实际运动时间。默认值：

| 项目 | 默认配置 |
| --- | --- |
| 关节角度范围 | 从 MJCF 模型读取 E/D/C/B 的实际限位 |
| 关节速度上限 | E/D/C/B 分别 20/20/20/25 °/s |
| TCP 速度上限 | 0.04 m/s |
| 期望运动时间 | 1.2 s，界面可调 |
| 任务范围，世界系 X | −0.35 ～ 0.68 m |
| 任务范围，世界系 Y | −0.50 ～ 0.51 m |
| 任务范围，世界系 Z | 0.03 ～ 0.88 m |
| Cartesian 路径采样间隔 | 不超过 1 mm |
| 关节路径采样间隔 | 每个关节不超过 1° |

关节目标在规划接口中截断到关节限位。XYZ 输入框约束任务包围盒；拖动或规划接口的目标超出任务盒/最大伸展球时投影到二者交集，并提示“目标已限幅”。最大伸展球半径由原始连杆链长度计算，是保守外界而非完整可达区域；仍须进行 IK 和路径检查，不能把限幅后的目标直接视为可达。

蓝色点云采样 E/D/C 关节范围，腕部固定为初始角度，筛选出任务范围内且碰撞代理体无穿透的 TCP 点。顶部可切换点云显示。它只是模型下的离散可行位置示意，不是填充体积或严格可达边界；换腕部角度、夹爪、碰撞几何或模型参数后可行性会改变，执行检查始终使用当前状态。

关节运动使用 `q(t)=(1-s)q_start+s q_goal`。末端运动先将 `p(s)=(1-s)p_start+s p_goal` 分段并逐点使用连续初值 IK；再在相邻 IK 关节解间线性插值播放。中点检查 TCP 相对目标直线的偏差不超过 0.05 mm，IK 点位置容差为 0.01 mm；它是离散检查的近似直线，不是解析直线。蓝线显示预检查路径，视图顶部显示进度。拖动中更新目标会从当前实际姿态重新规划，不跳到新终点。

速度上限用于计算运动学线性路径的最小运动时间：检查分段关节速度以及端点/中点 Jacobian 得到的 TCP 速度，实际时长取期望时间与速度所需时间中的较大值。GUI 每帧沿该路径推进；长时间界面阻塞后限制单次播放时间推进，避免突然跳到终点。这些上限是仿真起步设置，不是实机额定值。该线性操作未限制加速度/加加速度，起止处存在速度突变；新增动力学轨迹模式采用独立参考曲线、加速度预检查和力矩控制器。

预检查覆盖每个离散点及相邻关节插值中点的范围/碰撞检查，不是连续几何无碰撞的数学证明。运动规划使用独立 MuJoCo data；规划失败或采样工作区不会改变显示中的机械臂姿态。停止操作冻结当前运动学姿态，不等同于实机急停。

Python 规划接口：

```python
from uvms.kinematics import ArmKinematics
from uvms.motion import MotionPlanner

arm = ArmKinematics()
planner = MotionPlanner(arm)
plan = planner.cartesian([0.11, -0.03, 0.60], duration=1.2)
q_mid, progress = plan.at(plan.duration / 2)
arm.forward(q_mid, unit="rad")
# planner.joint([...]) 的目标角度单位为弧度；失败抛出 MotionError。
```

已用 Qt 自动化事件测试滑块鼠标拖动和键盘调整、XYZ 求解按钮、目标点拖动、三轴手柄拖动、坐标切换和失败保留姿态；截图检查覆盖中文字体、实际机械臂网格和控件布局。参考截图：`outputs/kinematics_gui.png`。这些是离屏自动化验证，未由人工逐项操作桌面窗口。

原有终端接口和 Python 接口继续可用：

```powershell
# 正运动学：四个转动关节按 E、D、C、B 顺序，默认单位为度
.\.venv\Scripts\python.exe scripts\kinematics.py --q 30 90 45 30 --render

# 指定世界坐标中的 TCP 位置（米），逆解并预览
.\.venv\Scripts\python.exe scripts\kinematics.py --xyz 0.11 -0.03 0.60 --viewer

# 指定逆解初值、基座坐标、夹爪开合比例
.\.venv\Scripts\python.exe scripts\kinematics.py --xyz 0.10 -0.03 0.20 --frame base --q 30 90 45 30 --gripper 0.5

# 连续输入并更新可视化；去掉 --viewer 可仅使用终端
.\.venv\Scripts\python.exe scripts\kinematics.py --interactive --viewer

# 弧度输入
.\.venv\Scripts\python.exe scripts\kinematics.py --q 0.6 1.5746 0.8 0.6 --unit rad
```

交互模式输入 `q 30 90 45 30`、`xyz 0.11 -0.03 0.60`、`home` 或 `quit`。角度单位和位置坐标系沿用启动时的 `--unit`、`--frame`；夹爪使用启动时 `--gripper` 或默认开合比例。交互逆解以当前关节状态为初值。输入错误和不收敛不会替换当前姿态。

`--frame world` 表示场景世界系（地面 z=0）；`--frame base` 表示原始 `base_link` 系，目前该系原点的世界坐标是 `[0.0374, 0, 0.3725] m`。四个关节的角度遵循模型关节轴右手规则，不是相对 home 的偏移量。位置对应 `tool_tip`/原始 TCP，不是夹爪最远尖端；输入位置单位始终为米。

终端默认显示位置、关节角度和碰撞检查摘要，`--json` 输出完整机器可读结果。完整输出含 `q_deg`、`q_rad`、TCP 位置、旋转矩阵、4×4 齐次变换、3×4 位置雅可比和当前姿态碰撞代理体检查。结果写入 `outputs/kinematics_result.json`；`--render` 将指定姿态保存到 `outputs/kinematics_pose.png`。两者每次覆盖，交互输入更新 JSON，截图仅在启动时生成。

逆运动学采用关节限位内的阻尼最小二乘迭代和确定性多初值搜索，默认位置容差 `1e-5 m`，可用 `--tolerance` 修改。只约束三维位置，不约束姿态；TCP 位于腕部旋转轴上，因此 `axis_b` 保持初值，其他关节解也可能不唯一。返回 `success=false` 时提供最佳候选和误差，进程退出码为 2；数值不收敛不等于已证明目标不可达。输入格式/非有限数/关节越界会明确拒绝。

默认 FK/IK 是运动学映射，允许输出存在代理体接触的姿态，并返回 `collision_free`。逆解加 `--reject-collisions` 可拒绝检测到穿透的最终姿态，但不是路径规划或真实几何无碰撞保证。关节输入直接更新姿态，未进行轨迹插值、速度限制或动力学执行；`--viewer` 不推进物理时间。`--gripper` 为开合比例 0–1，0 关闭、1 最大开合，不是角度或夹爪间距。

Python 接口无需 CLI，适合后续算法调用：

```python
from uvms.kinematics import ArmKinematics

arm = ArmKinematics(medium="water")
fk = arm.forward([30, 90, 45, 30], unit="deg", frame="world", gripper=0.5)
ik = arm.inverse([0.11, -0.03, 0.60], seed=fk["q_deg"], unit="deg", frame="world")
if ik["success"]:
    print(ik["state"]["q_deg"], ik["error_m"])
```

运行全部检查：`.\.venv\Scripts\python.exe -m unittest discover -s tests -v`。目前 34 项测试覆盖实际空气/水下抓取放置、阶段连续性、允许接触范围、GUI 序列启动完成，方块自由运动/落稳、接触分类、物体状态保留/复制和 GUI 重置，以及原有完整位姿 IK、动力学正逆关系、真实物理步进、日志对齐、动力学 GUI/曲线窗口、自动接近、力矩限幅和运动学/路径检查。依赖 MuJoCo 的 [site Jacobian API](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#mj-jacsite)。运动学计算不依赖流体或电机参数；实际跟踪结果会受惯性、阻尼、流体力和控制器影响。

## 文件与环境

| 文件 | 用途 |
| --- | --- |
| `config/alpha5.json` | 初始姿态、安装高度、时间步长、占位阻尼/转子惯量和水参数 |
| `config/scene.json` | 操作台尺寸/位置、自由方块尺寸/质量/摩擦、GUI 相机 |
| `config/motion.json` | 任务区域、速度上限、期望时长和路径采样配置 |
| `uvms/motion.py` | 关节/末端线性路径、限幅、逐点检查和工作区采样 |
| `uvms/dynamics.py` | 质量矩阵、重力/科氏偏置和刚体正逆动力学 |
| `uvms/trajectory.py` | JSON/CSV/函数曲线输入、完整位姿 IK 和轨迹预检查 |
| `uvms/tracking.py` | 计算力矩控制、物理积分及同步遥测日志 |
| `uvms/pick_place.py` | 分阶段抓取放置、预检查和任务完成验证 |
| `config/tracking.json` | 控制器、采样率、加速度与接近/保持阶段配置 |
| `trajectories/` | 三种曲线示例及抓取放置序列 |
| `docs/trajectory_tracking.md` | 轨迹格式、物理量单位、模型方程与验证结果 |
| `models/alpha5.urdf` | 展开后的原始结构，网格已转换为 OBJ，可作为后续工具交换入口 |
| `models/arm_air.xml` | 空气基准场景，关闭流体效应 |
| `models/arm_water.xml` | 全浸没流体阻力基准场景 |
| `models/assets/` | 原始 DAE 转换后的视觉 OBJ 网格 |
| `models/alpha5_imported.xml` | MuJoCo 原始 URDF 导入结果，用于追溯转换 |
| `scripts/build_model.py` | Xacro 展开、网格转换、场景生成 |
| `scripts/simulate.py` | 模型加载、转换一致性检查、预览、短时检查和截图 |
| `vendor/reach_description/` | 原始素材与许可证，已下载到本地 |
| `requirements-lock.txt` | 当前环境所有依赖的固定版本 |

本机原有默认 Python 3.8 不满足此环境要求，已通过可用的 Python 3.12 创建项目内 `.venv`，未修改系统 Python。重新搭建时请指定可用的 Python 3.12：

```powershell
./scripts/setup.ps1 -Python 'C:\path\to\Python312\python.exe'
```

脚本安装固定依赖并重新生成模型。迁移目录后须重建 `.venv` 并重新生成模型，`models/source/` 中间 Xacro 含本机路径；最终 MJCF 和 OBJ 采用相对路径。

## 关节定义与输入

五功能 = 四个机械臂转动关节 + 一个夹爪推杆驱动。机械臂本体保留两个夹爪从动关节，因此其广义坐标/速度各为 7、驱动数 `nu=5`，存在两个夹爪联动约束。当前场景另有一个自由方块，增加 7 个位置坐标（XYZ+四元数）与 6 个速度坐标，因此整个场景 `nq=14`、`nv=13`。不要直接用完整 qpos 当作机械臂角度；通过关节名称/地址访问。

| 驱动顺序 | 关节 | 类型 | 原模型范围 | 输入单位/上限 |
| --- | --- | --- | --- | --- |
| 0 | `axis_e` | 转动 | 0–6.10 rad | N·m / ±9 |
| 1 | `axis_d` | 转动 | −3.49–3.49 rad | N·m / ±9 |
| 2 | `axis_c` | 转动 | 0–3.22 rad | N·m / ±9 |
| 3 | `axis_b` | 转动 | 0–3.22 rad | N·m / ±9 |
| 4 | `axis_a` | 推杆平移 | 原值 0–0.015 m，有效范围见下文 | N / ±10 |

`rs1_130_joint = 51 × axis_a`，`rs1_139_joint = rs1_130_joint`，两个从动关节范围均为 0–0.5 rad。原始推杆上限 0.015 m 与该联动关系冲突，MJCF 场景按 `0.5/51 ≈ 0.00980392 m` 限制推杆，原始 URDF 保留来源值。联动由 MuJoCo 软等式约束实现，运动时允许小量约束残差。

后续可通过 `data.ctrl[:]` 注入上述五路广义力，`data.qfrc_applied` 注入自定义关节附加力，或通过 `data.xfrc_applied` 注入连杆外部力/力矩。`tool_tip` 位于来源模型 TCP 坐标系原点，并不等同于最远夹爪尖端。传感器包含五路驱动的位置/速度和 TCP 世界位置。读状态建议按名称查找关节地址，不将夹爪平移值当作角度。

## 来源、假设与边界

- 官方产品：[Reach Alpha 5](https://reachrobotics.com/products/reach-alpha/reach-alpha-5)。
- 结构来源：[Robotic-Decision-Making-Lab/reach](https://github.com/Robotic-Decision-Making-Lab/reach)，固定下载提交 `5e37b9df49fb1a47fb2c0b02ba641ef854d552f9`。使用 `reach_description/description/alpha_5/`、`config/alpha_5/` 和 `meshes/alpha_5/`。原始厂家/维护者说明和许可证保留在 `vendor/reach_description/`；其素材许可证限定用于 Reach Robotics 产品，不能作为任意机械臂通用素材使用。
- 质量、质心和惯性来自原始 URDF；源文件中若干基座/TCP/推杆链接使用 `0.0001` 的占位质量或惯性。这些值未经过本工程实机标定，后续动力学验证必须核对。驱动力上限也来自 URDF，不作为真实电机额定参数。
- 关节阻尼 `0.02` 和 armature `0.0001` 是数值起步参数，转动/平移关节对应单位不同；不代表真实摩擦、转子惯量或减速器参数。
- 大部分连杆碰撞沿用原始圆柱/盒体。原夹掌圆柱侵占整个夹持空间，现改用夹掌网格的凸包；两夹指采用自定义接触垫（全尺寸 22×4×30 mm，局部倾角 ±0.17 rad，摩擦系数 1.5）。这些是接触仿真近似，尚未标定实物夹指。保留基座轴承对和两夹指对的自接触排除，其余自接触和环境接触继续检测；展示支座关闭碰撞。视觉网格仍不参与碰撞，夹掌另建碰撞网格。夹爪联动 solref=0.002 s，接触 solref=0.005 s、solimp=0.99，impratio=10、noslip_iterations=20，以减少摩擦约束的数值滑移。
- 水下场景采用 `density=1000 kg/m³`、`viscosity=0.001 Pa·s` 和碰撞几何的椭球近似流体力。在几何代理体上计算流体作用，避免视觉网格重复计入。TCP 为无实体坐标系，其流体系数设为零，避免占位惯性生成不合理的惯性椭球阻力。它是全浸没阻力基准，没有自由液面，也不包含浮力、附加质量、海流场、压力效应或经过标定的水动力系数。[MuJoCo 参数文档](https://mujoco.readthedocs.io/en/stable/XMLreference.html#option)
- `motor` 是理想广义力输入，目前不包含电压/电流、反电动势、绕组电感、电机热模型或减速器效率；新增外部计算力矩控制器驱动轨迹跟踪。四个机械臂自由度不能实现任意六维末端姿态，完整位姿轨迹会预检查并拒绝不可实现的输入。

## 本阶段验证

已完成依赖一致性检查、两场景编译、七个广义坐标/五路驱动/两条联动约束检查、质量惯性正值检查、初始姿态限位检查和初始接触检查。将最终 MJCF 的连杆位置、方向、质量及惯性与原始导入 URDF 在相同姿态下比较，并执行两场景各 2 秒零输入运行与 1280×960 图形渲染。

环境与模型基础检查、正/位置逆/完整位姿逆运动学、刚体动力学接口和基础力矩轨迹跟踪已完成，并输出对应的期望/实际物理量及验证结果。真实参数核验、完整 UVMS 整体运行测试、定制水下电机/水动力模型和更复杂控制器仍待后续工作。
