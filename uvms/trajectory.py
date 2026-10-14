"""Timed JSON/CSV trajectories, continuous IK, and checked joint references."""
import csv
import json
import math
from pathlib import Path
import numpy as np
import mujoco
from .kinematics import ArmKinematics
from .spatial import quaternion, matrix_quaternion, rotation_error, slerp
from .motion import MotionPlanner
from .dynamics import RigidBodyDynamics

ROOT = Path(__file__).resolve().parents[1]


class HermiteCurve:
    def __init__(self,times,values):
        self.t = np.asarray(times,dtype=float)
        self.values = np.asarray(values,dtype=float)
        self.derivatives = np.gradient(self.values,self.t,axis=0,edge_order=1)
        self.derivatives[[0,-1]] = 0

    def at(self,time):
        time = np.clip(time,self.t[0],self.t[-1])
        i = min(max(int(np.searchsorted(self.t,time,side="right"))-1,0),len(self.t)-2)
        dt = self.t[i+1]-self.t[i]
        u = (time-self.t[i])/dt
        a,b = self.values[i],self.values[i+1]
        v,w = self.derivatives[i]*dt,self.derivatives[i+1]*dt
        value = (2*u**3-3*u**2+1)*a+(u**3-2*u**2+u)*v+(-2*u**3+3*u**2)*b+(u**3-u**2)*w
        first = ((6*u*u-6*u)*a+(3*u*u-4*u+1)*v+(-6*u*u+6*u)*b+(3*u*u-2*u)*w)/dt
        second = ((12*u-6)*a+(6*u-4)*v+(-12*u+6)*b+(6*u-2)*w)/(dt*dt)
        return value,first,second


class InputTrajectory:
    def __init__(self,payload):
        self.payload = payload
        first = payload.get("points",[{}])[0] if payload.get("points") else {}
        self.mode = payload.get("mode","pose" if "quaternion" in first else "joint" if "q" in first else "position")
        if self.mode not in ("position","pose","joint"):
            raise ValueError("Trajectory mode must be position, pose, or joint")
        self.frame = payload.get("frame","world")
        if self.frame not in ("world","base"):
            raise ValueError("Trajectory frame must be world or base")
        points = payload["points"]
        if len(points) < 2:
            raise ValueError("At least two timed trajectory points are required")
        self.times = np.array([p["t"] for p in points],dtype=float)
        if not np.isfinite(self.times).all() or self.times[0] != 0 or np.any(np.diff(self.times) <= 0):
            raise ValueError("Times must be finite, strictly increasing, and start at 0 seconds")
        key = "q" if self.mode == "joint" else "position"
        size = 4 if self.mode == "joint" else 3
        values = np.asarray([p[key] for p in points],dtype=float)
        if values.shape != (len(points),size) or not np.isfinite(values).all():
            raise ValueError("Invalid trajectory values")
        if self.mode == "joint":
            unit = payload.get("angle_unit","rad")
            if unit not in ("deg","rad"):
                raise ValueError("angle_unit must be deg or rad")
            if unit == "deg":
                values = np.deg2rad(values)
        self.curve = HermiteCurve(self.times,values)
        self.quats = np.array([quaternion(p["quaternion"]) for p in points]) if self.mode == "pose" else None
        self.duration = float(self.times[-1])

    @classmethod
    def from_curve(cls,duration,position_fn,quaternion_fn=None,period=0.02,frame="world"):
        """Sample caller-defined p(t), and optionally quaternion(t), without eval."""
        if not np.isfinite(duration) or duration <= 0 or not np.isfinite(period) or period <= 0:
            raise ValueError("Curve duration and sample period must be finite and positive")
        points = []
        for time in np.linspace(0,duration,max(2,math.ceil(duration/period)+1)):
            point = {"t":float(time),"position":np.asarray(position_fn(float(time)),dtype=float).tolist()}
            if quaternion_fn is not None:
                point["quaternion"] = np.asarray(quaternion_fn(float(time)),dtype=float).tolist()
            points.append(point)
        return cls({"mode":"pose" if quaternion_fn is not None else "position","frame":frame,"points":points})

    @classmethod
    def load(cls,path,time_scale=1):
        path = Path(path)
        if not math.isfinite(time_scale) or time_scale <= 0:
            raise ValueError("time_scale must be finite and positive")
        if path.suffix.lower() == ".csv":
            with path.open(encoding="utf-8-sig",newline="") as handle:
                rows = list(csv.DictReader(handle))
            if not rows:
                raise ValueError("Empty CSV")
            joint_keys = [f"q_{axis}_rad" for axis in "edcb"]
            mode = "joint" if all(k in rows[0] for k in joint_keys) else "position"
            quat_keys = ["qw","qx","qy","qz"]
            if mode != "joint" and any(k in rows[0] for k in quat_keys):
                if not all(k in rows[0] for k in quat_keys):
                    raise ValueError("Pose CSV requires all qw,qx,qy,qz columns")
                mode = "pose"
            points = []
            for row in rows:
                p = {"t":float(row["time_s"])}
                p["q" if mode == "joint" else "position"] = [float(row[k]) for k in (joint_keys if mode == "joint" else ["x_m","y_m","z_m"])]
                if mode == "pose":
                    p["quaternion"] = [float(row[k]) for k in quat_keys]
                points.append(p)
            payload = {"mode":mode,"frame":"world","points":points}
        else:
            payload = json.loads(path.read_text(encoding="utf-8-sig"))
        for point in payload["points"]:
            point["t"] *= time_scale
        result = cls(payload)
        result.source_file = str(path.resolve())
        result.time_scale = time_scale
        return result

    def quaternion_at(self,time):
        index = min(max(int(np.searchsorted(self.times,time,side="right"))-1,0),len(self.times)-2)
        fraction = np.clip((time-self.times[index])/(self.times[index+1]-self.times[index]),0,1)
        return slerp(self.quats[index],self.quats[index+1],fraction)


class CompiledTrajectory:
    def __init__(self,arm,source,config=None):
        self.config = config or json.loads((ROOT / "config/tracking.json").read_text(encoding="utf-8"))
        for key in ("reference_period_s","log_period_s","pose_position_tolerance_m",
                    "pose_orientation_tolerance_rad","max_joint_acceleration_rad_s2","abort_penetration_m","max_duration_s","approach_min_duration_s"):
            if not np.isfinite(self.config[key]) or self.config[key] <= 0:
                raise ValueError(f"Invalid tracking parameter: {key}")
        if not np.isfinite(self.config["settle_time_s"]) or self.config["settle_time_s"] < 0:
            raise ValueError("Invalid settle_time_s")
        self.source = source
        self.arm = ArmKinematics(model=arm.model)
        self.arm.copy_environment_from(arm)
        self.bounds = MotionPlanner(arm)
        self.gripper = float(arm.data.qpos[arm.pushrod]/arm.gripper_max)
        if source.duration > self.config["max_duration_s"]:
            raise ValueError("Trajectory exceeds configured maximum duration")
        self.origin,self.basis = arm._frame(source.frame)
        period = self.config["reference_period_s"]
        if not np.isfinite(period) or period <= 0:
            raise ValueError("Invalid reference sample period")
        grid = np.linspace(0,source.duration,max(2,math.ceil(source.duration/period)+1))
        initial = arm.data.qpos[arm.q_indices].copy()
        q,seed = [],initial
        for time in grid:
            if source.mode == "joint":
                value = source.curve.at(time)[0]
                self.arm.forward(value,unit="rad",gripper=self.gripper)
            else:
                position,quat = self.requested_pose(time)
                if not self.bounds.in_bounds(position):
                    raise ValueError(f"t={time:.3f}s: target leaves task workspace; trajectory is not clipped")
                if source.mode == "pose":
                    result = self.arm.inverse_pose(position,quat,seed=seed,gripper=self.gripper,
                        position_tolerance=self.config["pose_position_tolerance_m"],
                        orientation_tolerance=self.config["pose_orientation_tolerance_rad"],
                        reject_collisions=self.config["reject_reference_contacts"])
                else:
                    result = self.arm.inverse(position,seed=seed,unit="rad",gripper=self.gripper,restarts=0,tolerance=1e-9,
                                              reject_collisions=self.config["reject_reference_contacts"])
                if not result["success"]:
                    raise ValueError(f"t={time:.3f}s: no feasible {source.mode} IK solution; "
                                     "Alpha 5 has four arm DOFs. Adjust the path/posture or use position mode.")
                value = np.array(result["state"]["q_rad"])
            seed = value.copy()
            q.append(value)
        self.reference = source.curve if source.mode == "joint" else HermiteCurve(grid,np.array(q))
        self.initial_q = initial
        self.first_q = np.array(q[0])
        delta = self.first_q-initial
        self.approach_duration = 0.0 if np.max(np.abs(delta)) < 1e-4 else max(self.config["approach_min_duration_s"],
            float(np.max(1.875*np.abs(delta)/self.bounds.joint_speed)),
            float(np.sqrt(6*np.max(np.abs(delta))/self.config["max_joint_acceleration_rad_s2"])))
        if self.approach_duration:
            peak_tcp = 0.0
            for q_check in np.linspace(initial,self.first_q,31):
                self.arm.forward(q_check,unit="rad",gripper=self.gripper)
                peak_tcp = max(peak_tcp,float(np.linalg.norm(self.arm.jacobian()@delta)))
            self.approach_duration = max(self.approach_duration,1.9*peak_tcp/self.bounds.config["tcp_speed_m_s"])
        self.duration = self.approach_duration+source.duration
        # Validate interpolated midpoint references as well as all IK knots.
        check_times = np.unique(np.r_[grid,(grid[:-1]+grid[1:])/2]+self.approach_duration)
        if self.approach_duration:
            check_times = np.r_[np.linspace(0,self.approach_duration,math.ceil(self.approach_duration/period)+1),check_times]
        dyn = RigidBodyDynamics(arm)
        self.max_feedforward = np.zeros(5)
        actuator_ids = [arm.model.actuator("motor_"+n).id for n in ("axis_e","axis_d","axis_c","axis_b","axis_a")]
        ranges = arm.model.actuator_ctrlrange[actuator_ids]
        for time in check_times:
            ref = self.sample(float(time))
            state = self.arm.forward(ref["q"],unit="rad",gripper=self.gripper)
            if not self.bounds.in_bounds(state["position_m"]):
                raise ValueError(f"t={time:.3f}s: reference leaves configured workspace")
            if self.config["reject_reference_contacts"] and not state["collision_free"]:
                raise ValueError(f"t={time:.3f}s: reference collision proxy penetration")
            if np.any(np.abs(ref["qd"]) > self.bounds.joint_speed*1.01):
                raise ValueError(f"t={time:.3f}s: joint speed limit exceeded; increase trajectory time_scale")
            if np.max(np.abs(ref["qdd"])) > self.config["max_joint_acceleration_rad_s2"]:
                raise ValueError(f"t={time:.3f}s: joint acceleration limit exceeded; smooth or slow trajectory")
            if np.linalg.norm(self.arm.jacobian()@ref["qd"]) > self.bounds.config["tcp_speed_m_s"]*1.01:
                raise ValueError(f"t={time:.3f}s: TCP speed limit exceeded; increase time_scale")
            if source.mode != "joint" and ref["phase"] == "tracking":
                ep = np.linalg.norm(np.array(state["position_m"])-ref["position"])
                er = np.linalg.norm(rotation_error(ref["quaternion"],matrix_quaternion(state["rotation_matrix"])))
                if ep > self.config["pose_position_tolerance_m"]*2 or (source.mode == "pose" and er > self.config["pose_orientation_tolerance_rad"]*1.1):
                    raise ValueError(f"t={time:.3f}s: interpolated pose not feasible; supply denser/smoother trajectory points")
            force = dyn.inverse(ref["q"],ref["qd"],ref["qdd"],gripper=self.gripper,
                                compensate_passive=self.config["compensate_passive"])["generalized_force"]
            self.max_feedforward = np.maximum(self.max_feedforward,np.abs(force))
            if np.any(force < ranges[:,0]) or np.any(force > ranges[:,1]):
                raise ValueError(f"t={time:.3f}s: nominal generalized forces {np.round(force,3).tolist()} exceed motor limits (first four Nm, fifth N); slow trajectory")

    def requested_pose(self,time):
        local = self.source.curve.at(time)[0]
        position = self.origin+self.basis@local
        quat = None
        if self.source.mode == "pose":
            matrix = np.empty(9)
            mujoco.mju_quat2Mat(matrix,self.source.quaternion_at(time))
            quat = matrix_quaternion(self.basis@matrix.reshape(3,3))
        return position,quat

    def sample(self,time):
        source_time = np.clip(time-self.approach_duration,0,self.source.duration)
        if time < self.approach_duration:
            u = np.clip(time/self.approach_duration,0,1)
            delta = self.first_q-self.initial_q
            q = self.initial_q+delta*(10*u**3-15*u**4+6*u**5)
            qd = delta*(30*u*u-60*u**3+30*u**4)/self.approach_duration
            qdd = delta*(60*u-180*u*u+120*u**3)/self.approach_duration**2
            phase = "approach"
        else:
            q,qd,qdd = self.reference.at(source_time)
            phase = "tracking" if time <= self.duration else "settling"
            if time >= self.duration:
                qd,qdd = np.zeros(4),np.zeros(4)
        state = self.arm.forward(q,unit="rad",gripper=self.gripper)
        position = np.array(state["position_m"])
        quat = matrix_quaternion(state["rotation_matrix"])
        if phase == "tracking" and self.source.mode != "joint":
            position,requested = self.requested_pose(source_time)
            if requested is not None:
                quat = requested
        return {"q":q,"qd":qd,"qdd":qdd,"position":position,"quaternion":quat,
                "phase":phase,"trajectory_time":float(source_time)}
