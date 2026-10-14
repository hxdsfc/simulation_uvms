# 机械臂运动学与动力学仿真

基于 **Python 3.12 + MuJoCo 3.14 + PySide6** 的机械臂仿真工作台。当前对象为 **Reach Alpha 5：四个转动关节 + 一个夹爪驱动**，场景包含固定基座、操作台和自由方块。

## 环境配置

已验证平台：Windows、Python 3.12。在项目根目录的 PowerShell 执行：

```powershell
# 创建 .venv、安装固定版本依赖、生成模型
powershell -ExecutionPolicy Bypass -File scripts/setup.ps1 -Python "C:\path\to\Python312\python.exe"
# 或已有 Python 3.12 在 PATH 中时：
powershell -ExecutionPolicy Bypass -File scripts/setup.ps1 -Python python
```

将示例路径换为本机 Python 3.12 的实际路径。需要联网安装依赖、可用的桌面/OpenGL 环境；无需 ROS。依赖版本见 [requirements-lock.txt](requirements-lock.txt)。`.venv`、运行输出和缓存不上传，克隆后需重新配置环境；迁移项目目录后重新运行配置脚本以重建模型。

## 启动入口

| 入口 | 用途 |
| --- | --- |
| **双击 `kinematics.cmd`** | 主图形界面：运动学操作、动力学轨迹、抓取放置、实时曲线 |
| 双击 `preview.cmd` | 静态模型预览，旋转/缩放查看，不推进物理时间 |
| `scripts/track.py` | 无 GUI 执行轨迹并导出数据 |

也可在 PowerShell 运行：

```powershell
.\.venv\Scripts\python.exe scripts/gui.py
.\.venv\Scripts\python.exe scripts/track.py trajectories/pick_place.json --medium air
```

## 操作模式

| 模式 | 操作与执行方式 |
| --- | --- |
| **关节 / 末端** | 四个关节滑块或角度框；输入 XYZ 求逆解；拖动三维目标点/坐标轴。先检查限位、工作区和碰撞，再插值播放；这是运动学操作。 |
| **动力学轨迹** | 选择示例或载入 JSON/CSV，选择空气/基础水阻力环境，点击“预检查并开始力矩跟踪”。通过电机广义力控制、MuJoCo 物理积分执行，显示期望/实际量。 |
| **抓取放置** | 在“动力学轨迹”中选择“抓取方块 → 绿色放置区”。自动接近、闭爪、抬升、搬运、放下、松爪和撤离。重复运行前点击“重置方块”。 |

右键旋转视角、滚轮缩放；顶部“停止运动”暂停当前运动并在动力学模式导出已有数据。“期望 / 实际 / 力矩曲线”打开反馈窗口。时间缩放大于 1 表示放慢轨迹。

## 当前项目情况

- **运动学解算**：正运动学、位置逆运动学、可实现完整位姿的逆运动学、雅可比、坐标系转换、关节限位、工作区与离散路径碰撞检查。
- **动力学控制**：刚体质量矩阵、重力/科氏偏置、正逆动力学；计算力矩控制、驱动力限幅、轨迹跟踪、夹爪开合/夹持力控制。方块通过真实接触和摩擦搬运。
- **物理量反馈**：期望/实际关节角度、速度、末端位置/姿态、关节力矩、推杆力、物体位置和夹持接触。完成/停止后在 `outputs/` 导出 CSV、NPZ、摘要、轨迹及 PNG 曲线。
- **验证**：34 项测试覆盖运动学、动力学、GUI 和空气/基础水阻力下的抓取放置。当前抓取示例抬升约 56.6 mm、搬运约 133 mm，释放后方块稳定落在平台。

当前仍是固定基座、未标定参数的仿真。Alpha 5 的四个转动关节不能实现任意六维位姿；不可行轨迹会被拒绝。水下模型目前提供基础阻力，尚未部署浮力、附加质量、海流及完整电机电气模型。

## 后续修改方向

| 修改目标 | 修改入口 |
| --- | --- |
| 当前机械臂尺寸/参数/安装姿态 | `config/alpha5.json`、来源 URDF/Xacro 与 `scripts/build_model.py`；生成后重新校验惯性、限位和接触 |
| 平台、方块和取放位置 | `config/scene.json`；抓取阶段关节角与目标同时修改 `trajectories/pick_place.json` |
| **更换机械臂对象** | 导入新对象的 URDF/MJCF 和网格，配置关节、执行器、惯性、TCP；随后适配 `uvms/kinematics.py`、`dynamics.py`、控制器和 GUI。当前多处按四关节及夹爪联动编写，增加自由度需要同步修改维度。 |
| **导入新路径** | 在 GUI 载入 JSON/CSV，或参考 `trajectories/` 新建 `position`、`pose`、`joint` 轨迹；时间从 0 开始递增，位置单位 m，四元数顺序 w/x/y/z |
| 控制器、电机和水动力扩展 | `config/tracking.json`、`uvms/tracking.py`、`uvms/dynamics.py`；通过 MuJoCo 广义力/外力接口接入自定义模型 |

修改配置/来源模型后重新生成：

```powershell
.\.venv\Scripts\python.exe scripts/build_model.py
# 修改初始姿态或机械臂后，重新生成三个基础曲线示例
.\.venv\Scripts\python.exe scripts/make_trajectories.py
# 验证
.\.venv\Scripts\python.exe -m unittest discover -s tests -v
```

[替换机械臂与导入路径指南](docs/extending.md) · [轨迹格式、动力学与反馈说明](docs/trajectory_tracking.md) · [模型及运动学接口详解](docs/model_and_interfaces.md)

## 模型来源与使用范围

Alpha 5 素材来自 [Robotic-Decision-Making-Lab/reach](https://github.com/Robotic-Decision-Making-Lab/reach)，固定提交 `5e37b9df49fb1a47fb2c0b02ba641ef854d552f9`。原始模型描述位于 `vendor/reach_description/`，转换网格位于 `models/assets/`。上游许可证限定用于 Reach Robotics 产品，替换其他机械臂时应使用新对象有权使用的素材；参见 [第三方声明](THIRD_PARTY_NOTICES.md) 与 [原始许可证](vendor/reach_description/LICENSE)。
