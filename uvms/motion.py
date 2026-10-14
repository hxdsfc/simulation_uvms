"""Bounded, preflight-checked linear kinematic trajectories (no dynamics)."""
from dataclasses import dataclass
import json
import math
from pathlib import Path
import numpy as np
import mujoco
from .kinematics import ArmKinematics, finite_vector

ROOT = Path(__file__).resolve().parents[1]


class MotionError(ValueError):
    pass


@dataclass
class MotionPlan:
    q: np.ndarray
    xyz: np.ndarray
    duration: float
    kind: str
    clipped: bool

    def at(self, elapsed):
        progress = np.clip(elapsed / self.duration, 0, 1)
        value = progress * (len(self.q)-1)
        index = min(int(value),len(self.q)-2)
        fraction = value-index
        return self.q[index]*(1-fraction)+self.q[index+1]*fraction, float(progress)


class MotionPlanner:
    def __init__(self, arm, config=None):
        self.arm = arm
        self.config = config or json.loads((ROOT / "config/motion.json").read_text(encoding="utf-8"))
        self.low = finite_vector(self.config["cartesian_min_world_m"],3,"workspace minimum")
        self.high = finite_vector(self.config["cartesian_max_world_m"],3,"workspace maximum")
        self.joint_speed = np.deg2rad(finite_vector(self.config["joint_speed_deg_s"],4,"joint speed"))
        if np.any(self.low >= self.high) or np.any(self.joint_speed <= 0):
            raise ValueError("Invalid workspace or speed limits")
        for name in ("tcp_speed_m_s","requested_duration_s","cartesian_step_m","joint_step_deg",
                     "cartesian_line_tolerance_m","max_ik_step_deg"):
            if not math.isfinite(self.config[name]) or self.config[name] <= 0:
                raise ValueError(f"Invalid motion parameter: {name}")
        # Conservative reach bound derived from the actual fixed-base link chain.
        pivot = arm.model.body("m2_1_1_link").id
        self.center = arm.data.xpos[pivot].copy()
        body = arm.model.site_bodyid[arm.site]
        self.radius = float(np.linalg.norm(arm.model.site_pos[arm.site]))
        while body != pivot:
            self.radius += float(np.linalg.norm(arm.model.body_pos[body]))
            body = arm.model.body_parentid[body]
            if body == 0:
                raise ValueError("TCP is not a descendant of the fixed pivot")
        # Independent data prevents planning/sampling from teleporting the display arm.
        self.probe = ArmKinematics(model=arm.model)
        if self.probe.model.nq != arm.model.nq:
            raise ValueError("Planner model mismatch")

    def in_bounds(self, xyz):
        xyz = np.asarray(xyz)
        return (np.all(xyz >= self.low-1e-9) and np.all(xyz <= self.high+1e-9)
                and np.linalg.norm(xyz-self.center) <= self.radius+1e-9)

    def clamp_target(self, xyz):
        original = finite_vector(xyz,3,"target")
        target = original.copy()
        # Alternating projection onto the task box and conservative reach sphere.
        for _ in range(30):
            target = np.clip(target,self.low,self.high)
            delta = target-self.center
            distance = np.linalg.norm(delta)
            if distance > self.radius:
                target = self.center + delta*(self.radius/distance)
        if not self.in_bounds(target):
            raise MotionError("任务区域和机械臂范围无交集，请检查配置")
        return target, not np.allclose(original,target,atol=1e-9,rtol=0)

    def _check(self, q, reject_contacts):
        state = self.probe.forward(q,unit="rad",gripper=self.arm.state()["gripper"])
        if not self.in_bounds(state["position_m"]):
            raise MotionError("路径离开设定工作范围；保留当前姿态")
        if reject_contacts and not state["collision_free"]:
            raise MotionError("路径存在碰撞代理体穿透；保留当前姿态")
        return np.array(state["position_m"])

    def _finish(self, q, xyz, kind, clipped, duration, reject_contacts):
        q,xyz = np.asarray(q),np.asarray(xyz)
        dt_min = float(np.max(np.abs(np.diff(q,axis=0))/self.joint_speed))
        dt_min = max(dt_min,float(np.max(np.linalg.norm(np.diff(xyz,axis=0),axis=1)))/self.config["tcp_speed_m_s"])
        for i in range(len(q)-1):
            mid = (q[i]+q[i+1])/2
            point = self._check(mid,reject_contacts)
            if kind == "cartesian" and np.linalg.norm(point-(xyz[i]+xyz[i+1])/2) > self.config["cartesian_line_tolerance_m"]:
                raise MotionError("该直线路径曲率/奇异性过大，请调整目标或初值")
            # Sample TCP speed at endpoints/midpoint of each joint-linear segment.
            delta = q[i+1]-q[i]
            for sample in (q[i],mid,q[i+1]):
                self.probe._set(sample,self.arm.state()["gripper"])
                dt_min = max(dt_min,float(np.linalg.norm(self.probe.jacobian() @ delta))/self.config["tcp_speed_m_s"])
        requested = self.config["requested_duration_s"] if duration is None else float(duration)
        if not math.isfinite(requested) or requested <= 0:
            raise ValueError("运动时间必须为正数")
        return MotionPlan(q,xyz,max(requested,dt_min*(len(q)-1),0.05),kind,clipped)

    def joint(self, target, duration=None, reject_contacts=True):
        self.probe.copy_environment_from(self.arm)
        target = finite_vector(target,4,"joint target radians")
        bounded = np.clip(target,self.arm.limits[:,0],self.arm.limits[:,1])
        start = self.arm.data.qpos[self.arm.q_indices].copy()
        count = max(2,math.ceil(np.max(np.abs(bounded-start))/np.deg2rad(self.config["joint_step_deg"]))+1)
        q = np.linspace(start,bounded,count)
        xyz = [self._check(value,reject_contacts) for value in q]
        return self._finish(q,xyz,"joint",not np.allclose(target,bounded,atol=1e-12,rtol=0),duration,reject_contacts)

    def cartesian(self, target, duration=None, reject_contacts=True):
        self.probe.copy_environment_from(self.arm)
        target,clipped = self.clamp_target(target)
        start_q = self.arm.data.qpos[self.arm.q_indices].copy()
        start_xyz = self.arm.position().copy()
        count = max(2,math.ceil(np.linalg.norm(target-start_xyz)/self.config["cartesian_step_m"])+1)
        xyz = np.linspace(start_xyz,target,count)
        q = [start_q]
        self._check(start_q,reject_contacts)
        for index,point in enumerate(xyz[1:],1):
            result = self.probe.inverse(point,seed=q[-1],unit="rad",restarts=0,
                                        gripper=self.arm.state()["gripper"],reject_collisions=reject_contacts)
            if not result["success"]:
                raise MotionError(f"直线路径第 {index}/{count-1} 段无可行逆解；保留当前姿态")
            value = np.array(result["state"]["q_rad"])
            if np.max(np.abs(value-q[-1])) > np.deg2rad(self.config["max_ik_step_deg"]):
                raise MotionError("直线路径出现关节突变，请调整目标")
            self._check(value,reject_contacts)
            q.append(value)
        return self._finish(q,xyz,"cartesian",clipped,duration,reject_contacts)

    def workspace_cloud(self, counts=(18,18,16)):
        """Sample feasible TCP positions, not a filled-volume reachability proof."""
        self.probe.copy_environment_from(self.arm)
        points = []
        axes = [np.linspace(lo,hi,num) for (lo,hi),num in zip(self.arm.limits[:3],counts)]
        wrist = self.arm.home_q[3]
        for e in axes[0]:
            for d in axes[1]:
                for c in axes[2]:
                    state = self.probe.forward([e,d,c,wrist],unit="rad")
                    if state["collision_free"] and self.in_bounds(state["position_m"]):
                        points.append(state["position_m"])
        return np.asarray(points,dtype=float).reshape(-1,3)
