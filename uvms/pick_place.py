"""Checked staged pick/place references; the free object moves only by contact."""
import json
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import mujoco
from .kinematics import ArmKinematics
from .motion import MotionPlanner
from .dynamics import RigidBodyDynamics
from .spatial import matrix_quaternion

ROOT = Path(__file__).resolve().parents[1]
PHASE_LABELS = {"open":"张开夹爪","approach":"接近取料点","descend":"下降对齐",
    "close":"闭爪夹持","lift":"抬起方块","transfer":"搬运方块","lower":"下降放置",
    "release":"松爪释放","retreat":"撤离放置点","settling":"等待稳定"}


class PickPlaceTrajectory:
    def __init__(self,arm,payload,time_scale=1,source_file=None):
        if not np.isfinite(time_scale) or time_scale <= 0:
            raise ValueError("Invalid time_scale")
        self.source = SimpleNamespace(mode="pick_place",payload=payload,source_file=source_file,time_scale=time_scale)
        self.config = json.loads((ROOT/"config/tracking.json").read_text())
        self.config.update(payload.get("controller",{}))
        grip_force = float(payload.get("grip_force_n",3))
        if not np.isfinite(grip_force) or not 0 < grip_force <= -arm.model.actuator("motor_axis_a").ctrlrange[0]:
            raise ValueError("Grip force must be positive and within pushrod actuator capacity")
        self.arm = ArmKinematics(model=arm.model)
        self.arm.copy_environment_from(arm)
        self.bounds = MotionPlanner(arm)
        self.initial_q = arm.data.qpos[arm.q_indices].copy()
        self.gripper = float(np.clip(arm.data.qpos[arm.pushrod]/arm.gripper_max,0,1))
        self.grasp_site = arm.model.site("grasp_center").id
        self.pad_ids = {arm.model.geom("pad_"+n).id for n in ("rs1_130_link","rs1_139_link")}
        self.block_geom = arm.model.geom("block_geom").id
        self.table_geom = arm.model.geom("table_top").id
        self.block_body = arm.model.body("grasp_block").id
        self.pick_position = np.array(payload["pickup_position_m"],dtype=float)
        self.place_position = np.array(payload["place_position_m"],dtype=float)
        if self.pick_position.shape != (3,) or self.place_position.shape != (3,) or not np.isfinite(np.r_[self.pick_position,self.place_position]).all():
            raise ValueError("Pickup/place positions must be finite XYZ")
        if np.linalg.norm(self.arm.data.xpos[self.block_body]-self.pick_position) > .003:
            raise ValueError("方块不在取料点；请先点击工具栏‘重置方块’，再运行抓取序列")
        self.segments = []
        if not payload.get("stages"):
            raise ValueError("Pick/place requires timed stages")
        start,q,g = 0.,self.initial_q.copy(),self.gripper
        dyn = RigidBodyDynamics(arm)
        for stage in payload["stages"]:
            phase = stage["phase"]
            if phase not in PHASE_LABELS or phase == "settling":
                raise ValueError("Unknown pick/place phase")
            target = arm.angles(stage.get("q",q),"rad")
            opening = float(stage["gripper"])
            requested = float(stage["duration_s"])*time_scale
            if not np.isfinite(requested) or requested <= 0 or not np.isfinite(opening) or not 0 <= opening <= 1:
                raise ValueError("Stage duration/opening out of range")
            delta = target-q
            peak_tcp = 0.
            for check in np.linspace(q,target,31):
                self.arm.forward(check,unit="rad",gripper=max(g,opening))
                peak_tcp = max(peak_tcp,float(np.linalg.norm(self.arm.jacobian()@delta)))
            duration = max(requested,float(np.max(1.9*np.abs(delta)/self.bounds.joint_speed)),
                1.9*peak_tcp/self.bounds.config["tcp_speed_m_s"],
                float(np.sqrt(6*np.max(np.abs(delta))/self.config["max_joint_acceleration_rad_s2"])))
            self.segments.append(dict(start=start,duration=duration,q=q.copy(),target=target.copy(),g=g,opening=opening,phase=phase))
            start += duration
            q,g = target,opening
        self.duration = start
        self.approach_duration = sum(s["duration"] for s in self.segments if s["phase"] in ("open","approach","descend"))
        if self.duration > self.config["max_duration_s"]:
            raise ValueError("Pick/place duration exceeds configured maximum")
        for phase,position in (("close",self.pick_position),("release",self.place_position)):
            stage = next((s for s in self.segments if s["phase"] == phase),None)
            if stage is None:
                raise ValueError(f"Missing {phase} stage")
            self.arm.forward(stage["target"],unit="rad",gripper=stage["opening"])
            if np.linalg.norm(self.arm.data.site_xpos[self.grasp_site]-position) > .002:
                raise ValueError(f"{phase}: grasp centre does not match configured object target")
        self.max_feedforward = np.zeros(5)
        ranges = arm.model.actuator_ctrlrange[[arm.model.actuator("motor_"+n).id for n in ("axis_e","axis_d","axis_c","axis_b","axis_a")]]
        for time in np.linspace(0,self.duration,int(np.ceil(self.duration/.02))+1):
            ref = self.sample(time)
            state = self.arm.forward(ref["q"],unit="rad",gripper=ref["gripper"])
            if not self.bounds.in_bounds(state["position_m"]):
                raise ValueError(f"{ref['phase']}: reference leaves workspace")
            if any(c.dist < -1e-6 and self.arm.robot_contact(c) and not self.allowed_contact(c,ref["phase"]) for c in self.arm.data.contact):
                raise ValueError(f"t={time:.2f}s {ref['phase']}: reference collision")
            dyn.arm.forward(ref["q"],unit="rad",gripper=ref["gripper"])
            dyn.arm.data.qvel[:] = dyn.mapping@np.r_[ref["qd"],ref["gripper_qd"]*arm.gripper_max]
            mujoco.mj_forward(arm.model,dyn.arm.data)
            terms = dyn.from_data(dyn.arm.data)
            force = terms["mass"]@np.r_[ref["qdd"],ref["gripper_qdd"]*arm.gripper_max]+terms["bias"]
            if self.config["compensate_passive"]:
                force -= terms["passive"]
            if ref["phase"] in ("lift","transfer","lower"):
                jac = np.zeros((3,arm.model.nv))
                mujoco.mj_jacSite(arm.model,dyn.arm.data,jac,None,self.grasp_site)
                force[:4] -= jac[:,arm.dof_indices].T@(arm.model.body_mass[self.block_body]*arm.model.opt.gravity)
            self.max_feedforward = np.maximum(self.max_feedforward,np.abs(force))
            if np.any(force < ranges[:,0]) or np.any(force > ranges[:,1]):
                raise ValueError(f"t={time:.2f}s: nominal actuator limit exceeded: {np.round(force,3).tolist()} (Nm/N); increase time_scale")

    def sample(self,time):
        segment = next((s for s in self.segments if time < s["start"]+s["duration"]),self.segments[-1])
        u = np.clip((time-segment["start"])/segment["duration"],0,1)
        value = np.clip(10*u**3-15*u**4+6*u**5,0,1)
        first = (30*u*u-60*u**3+30*u**4)/segment["duration"]
        second = (60*u-180*u*u+120*u**3)/segment["duration"]**2
        delta = segment["target"]-segment["q"]
        dg = segment["opening"]-segment["g"]
        q = segment["q"]+delta*value
        gripper = segment["g"]+dg*value
        state = self.arm.forward(q,unit="rad",gripper=gripper)
        return dict(q=q,qd=delta*first,qdd=delta*second,gripper=gripper,gripper_qd=dg*first,gripper_qdd=dg*second,
            position=np.array(state["position_m"]),quaternion=matrix_quaternion(state["rotation_matrix"]),
            phase=segment["phase"] if time < self.duration else "settling",trajectory_time=min(time,self.duration))

    def allowed_contact(self,contact,phase):
        pair = {contact.geom1,contact.geom2}
        return phase in ("close","lift","transfer","lower","release") and self.block_geom in pair and bool(pair & self.pad_ids)

    def contact_metrics(self,data):
        touched,normal = set(),0.
        for i,c in enumerate(data.contact):
            pair = {c.geom1,c.geom2}
            if self.block_geom in pair and pair & self.pad_ids:
                force = np.zeros(6)
                mujoco.mj_contactForce(self.arm.model,data,i,force)
                if force[0] > .01:
                    touched.update(pair & self.pad_ids)
                    normal += max(0,float(force[0]))
        return len(touched),normal

    def validate_state(self,data,phase):
        if phase in ("lift","transfer","lower"):
            if np.linalg.norm(data.xpos[self.block_body]-data.site_xpos[self.grasp_site]) > .03:
                raise RuntimeError("抓取失败：方块滑落或未夹持；已停止搬运")

    def task_summary(self,rows,data):
        start = np.array(rows[0]["object_position_m"])
        final = data.xpos[self.block_body].copy()
        lift = max(r["object_position_m"][2] for r in rows)-start[2]
        supported = any({c.geom1,c.geom2} == {self.block_geom,self.table_geom} for c in data.contact)
        released = self.contact_metrics(data)[0] == 0
        error = float(np.linalg.norm(final[:2]-self.place_position[:2]))
        ok = bool(lift > .04 and error < .01 and abs(final[2]-self.place_position[2]) < .002 and supported and released)
        return dict(success=ok,lift_height_m=float(lift),place_xy_error_m=error,
            initial_position_m=start.tolist(),final_position_m=final.tolist(),target_position_m=self.place_position.tolist(),
            released=released,supported_by_table=bool(supported),object_motion="free body; contact/friction only; no attachment or pose overwrite")

    def validate_completion(self,rows,data):
        if not self.task_summary(rows,data)["success"]:
            raise RuntimeError("抓取放置验证未通过：检查抬升高度、落点及释放接触")


def load_reference(arm,path,time_scale=1):
    from .trajectory import InputTrajectory, CompiledTrajectory
    path = Path(path)
    if path.suffix.lower() == ".json":
        payload = json.loads(path.read_text(encoding="utf-8-sig"))
        if payload.get("mode") == "pick_place":
            return PickPlaceTrajectory(arm,payload,time_scale,str(path.resolve()))
    return CompiledTrajectory(arm,InputTrajectory.load(path,time_scale))
