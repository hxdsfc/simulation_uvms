"""Torque-controlled MuJoCo stepping and synchronized physical telemetry."""
import csv
import json
from pathlib import Path
import numpy as np
import mujoco
from .dynamics import RigidBodyDynamics
from .spatial import matrix_quaternion, rotation_error


class TrackingSession:
    def __init__(self,arm,reference):
        self.arm,self.reference = arm,reference
        self.model,self.data = arm.model,arm.data
        self.config = reference.config
        self.dynamics = RigidBodyDynamics(arm)
        self.kp,self.kd = np.array(self.config["kp_accel"]),np.array(self.config["kd_accel"])
        if self.kp.shape != (5,) or self.kd.shape != (5,) or not np.isfinite(self.kp).all() or not np.isfinite(self.kd).all() or np.any(self.kp <= 0) or np.any(self.kd <= 0):
            raise ValueError("Invalid controller gains")
        self.ids = [self.model.actuator("motor_"+name).id for name in ("axis_e","axis_d","axis_c","axis_b","axis_a")]
        self.limits = self.model.actuator_ctrlrange[self.ids].copy()
        # Motor gear is 1 in this model. Do not mislabel non-unit gear as joint torque.
        if not np.allclose(self.model.actuator_gear[self.ids,0],1):
            raise ValueError("Tracking requires unit-gear generalized-force actuators")
        if np.max(np.abs(self.data.qpos[arm.q_indices]-reference.initial_q)) > 1e-6:
            raise ValueError("Model moved after trajectory preflight; compile it again")
        self.start_time = float(self.data.time)
        self.end_time = reference.duration+self.config["settle_time_s"]
        self.rows = []
        self.next_log = 0.0
        self.saturation_steps = 0
        self.status = "running"
        self.failure = None
        self.last = None
        self.release_anchor = None
        self.environment = {"density_kg_m3":float(self.model.opt.density),"viscosity_pa_s":float(self.model.opt.viscosity)}

    @property
    def time(self):
        return float(self.data.time-self.start_time)

    def control_and_record(self,force_log=False):
        time = self.time
        ref = self.reference.sample(time)
        if self.reference.source.mode == "pick_place" and ref["phase"] == "release":
            # The pressure-controlled grasp stops at the object, not zero travel.
            # Start opening from that measured position to avoid first squeezing
            # harder while a zero-to-open reference catches up with the jaws.
            if self.release_anchor is None:
                self.release_anchor = float(np.clip(self.data.qpos[self.arm.pushrod]/self.arm.gripper_max,0,1))
            segment = next(s for s in self.reference.segments if s["phase"] == "release")
            u = np.clip((time-segment["start"])/segment["duration"],0,1)
            delta = segment["opening"]-self.release_anchor
            ref["gripper"] = self.release_anchor+delta*np.clip(10*u**3-15*u**4+6*u**5,0,1)
            ref["gripper_qd"] = delta*(30*u*u-60*u**3+30*u**4)/segment["duration"]
            ref["gripper_qdd"] = delta*(60*u-180*u*u+120*u**3)/segment["duration"]**2
        mujoco.mj_forward(self.model,self.data)
        terms = self.dynamics.from_data(self.data)
        q_des = np.r_[ref["q"],ref.get("gripper",self.reference.gripper)*self.arm.gripper_max]
        qd_des = np.r_[ref["qd"],ref.get("gripper_qd",0)*self.arm.gripper_max]
        qdd_des = np.r_[ref["qdd"],ref.get("gripper_qdd",0)*self.arm.gripper_max]
        q_act = self.data.qpos[self.dynamics.q_indices]
        qd_act = self.data.qvel[self.dynamics.indices]
        feedforward = terms["mass"]@qdd_des+terms["bias"]
        if self.config["compensate_passive"]:
            feedforward -= terms["passive"]
        if self.reference.source.mode == "pick_place" and ref["phase"] in ("lift","transfer","lower"):
            # Carry the configured payload weight at the physical grip centre.
            # Contact constraints still determine its actual motion and slip.
            jac = np.zeros((3,self.model.nv))
            mujoco.mj_jacSite(self.model,self.data,jac,None,self.reference.grasp_site)
            feedforward[:4] -= jac[:,self.arm.dof_indices].T @ (self.model.body_mass[self.reference.block_body]*self.model.opt.gravity)
        requested = feedforward+terms["mass"]@(self.kp*(q_des-q_act)+self.kd*(qd_des-qd_act))
        if self.reference.source.mode == "pick_place":
            feedback = self.kp*(q_des-q_act)+self.kd*(qd_des-qd_act)
            requested[:4] = feedforward[:4]+terms["mass"][:4,:4]@feedback[:4]
            requested[4] = feedforward[4]+terms["mass"][4,4]*feedback[4]
            if ref["phase"] in ("lift","transfer","lower") or (ref["phase"] == "close" and self.reference.contact_metrics(self.data)[0] == 2):
                requested[4] = -float(self.reference.source.payload.get("grip_force_n",3.0))
        command = np.clip(requested,self.limits[:,0],self.limits[:,1])
        saturated = bool(np.any(np.abs(command-requested) > 1e-9))
        self.saturation_steps += int(saturated)
        self.data.ctrl[self.ids] = command
        mujoco.mj_forward(self.model,self.data)
        position = self.data.site_xpos[self.arm.site].copy()
        quat = matrix_quaternion(self.data.site_xmat[self.arm.site])
        row = {"time_s":time,"trajectory_time_s":ref["trajectory_time"],"phase":ref["phase"],
            "q_des_rad":ref["q"].copy(),"q_act_rad":q_act[:4].copy(),
            "qd_des_rad_s":ref["qd"].copy(),"qd_act_rad_s":qd_act[:4].copy(),
            "qdd_des_rad_s2":ref["qdd"].copy(),"qdd_act_rad_s2":self.data.qacc[self.arm.dof_indices].copy(),
            "tau_feedforward_nm":feedforward[:4].copy(),"tau_requested_nm":requested[:4].copy(),
            "tau_command_nm":command[:4].copy(),"tau_actual_nm":self.data.qfrc_actuator[self.arm.dof_indices].copy(),
            "tau_bias_nm":self.data.qfrc_bias[self.arm.dof_indices].copy(),
            "tau_passive_nm":self.data.qfrc_passive[self.arm.dof_indices].copy(),
            "tau_constraint_nm":self.data.qfrc_constraint[self.arm.dof_indices].copy(),
            "position_des_m":ref["position"].copy(),"position_act_m":position,
            "quaternion_des_wxyz":ref["quaternion"].copy(),"quaternion_act_wxyz":quat,
            "position_error_m":float(np.linalg.norm(position-ref["position"])),
            "orientation_error_rad":float(np.linalg.norm(rotation_error(ref["quaternion"],quat))),
            "gripper_des_m":float(q_des[4]),"gripper_act_m":float(q_act[4]),
            "gripper_force_requested_n":float(requested[4]),"gripper_force_command_n":float(command[4]),
            "gripper_force_actual_n":float(self.data.actuator_force[self.ids[4]]),
            "contact_count":int(self.data.ncon),"saturated":int(saturated)}
        block_id = mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_BODY,"grasp_block")
        if block_id >= 0:
            row["object_position_m"] = self.data.xpos[block_id].copy()
            row["object_quaternion_wxyz"] = self.data.xquat[block_id].copy()
        row["robot_contact_count"] = sum(self.arm.robot_contact(c) for c in self.data.contact)
        if hasattr(self.reference,"contact_metrics"):
            row["grasp_contact_sides"],row["grasp_normal_force_n"] = self.reference.contact_metrics(self.data)
            row["grasp_position_act_m"] = self.data.site_xpos[self.reference.grasp_site].copy()
        self.last = row
        if time >= self.next_log-1e-9 or force_log:
            if not self.rows or abs(self.rows[-1]["time_s"]-time) > 1e-9:
                self.rows.append(row)
            self.next_log = time+self.config["log_period_s"]
        return row

    def step(self):
        if self.status != "running":
            return
        try:
            self.control_and_record()
            if self.time >= self.end_time-1e-9:
                self.control_and_record(force_log=True)
                if hasattr(self.reference,"validate_completion"):
                    self.reference.validate_completion(self.rows,self.data)
                self.status = "completed"
                return
            # The actual joints are changed exclusively by physics integration.
            mujoco.mj_step(self.model,self.data)
            if not np.isfinite(self.data.qpos).all() or not np.isfinite(self.data.qvel).all():
                raise RuntimeError("Nonfinite physical state")
            mujoco.mj_forward(self.model,self.data)
            if any(w.number for w in self.data.warning):
                raise RuntimeError("MuJoCo simulation warning")
            phase = self.reference.sample(self.time)["phase"]
            if any(self.arm.robot_contact(c) and c.dist < -self.config["abort_penetration_m"] and
                   not (hasattr(self.reference,"allowed_contact") and self.reference.allowed_contact(c,phase) and c.dist >= -.001) for c in self.data.contact):
                raise RuntimeError("Actual collision proxy penetration exceeds threshold")
            if hasattr(self.reference,"validate_state"):
                self.reference.validate_state(self.data,phase)
            if np.any(self.data.qpos[self.arm.q_indices] < self.arm.limits[:,0]-0.02) or np.any(self.data.qpos[self.arm.q_indices] > self.arm.limits[:,1]+0.02):
                raise RuntimeError("Actual joint limit overshoot")
        except Exception as exc:
            self.status,self.failure = "failed",str(exc)
            self.data.ctrl[:] = 0
            raise RuntimeError(str(exc)) from exc

    def run(self):
        while self.status == "running":
            self.step()
        return self.summary()

    def stop(self):
        if self.status == "running":
            self.control_and_record(force_log=True)
            self.status = "stopped"
        self.data.ctrl[:] = 0

    def summary(self):
        if not self.rows:
            return {"status":self.status,"samples":0,"failure":self.failure}
        following = [row for row in self.rows if row["phase"] == "tracking"] or self.rows
        pos = np.array([row["position_error_m"] for row in following])
        ori = np.array([row["orientation_error_rad"] for row in following])
        q = np.array([row["q_act_rad"]-row["q_des_rad"] for row in following])
        result = {"status":self.status,"failure":self.failure,"samples":len(self.rows),
            "mode":self.reference.source.mode,"duration_s":self.time,
            "approach_duration_s":self.reference.approach_duration,"physics_timestep_s":float(self.model.opt.timestep),
            "position_rmse_m":float(np.sqrt(np.mean(pos**2))),"position_max_error_m":float(np.max(pos)),
            "orientation_rmse_rad":float(np.sqrt(np.mean(ori**2))),"orientation_max_error_rad":float(np.max(ori)),
            "joint_rmse_rad":np.sqrt(np.mean(q*q,axis=0)).tolist(),
            "max_actuator_torque_nm":np.max(np.abs([r["tau_actual_nm"] for r in self.rows]),axis=0).tolist(),
            "saturation_control_evaluations":self.saturation_steps,"passive_compensation":self.config["compensate_passive"],
            "environment":self.environment,"input_source":getattr(self.reference.source,"source_file",None),
            "time_scale":getattr(self.reference.source,"time_scale",1),
            "orientation_definition":"requested quaternion" if self.reference.source.mode == "pose" else "FK of reference joint solution; orientation not independently commanded",
            "model":"Alpha 5 source inertias plus configured joint armature; parameters not calibrated",
            "torque_definition":"simulated generalized actuator torque at gear=1; not an observed hardware torque sensor",
            "time_alignment":"states/commands at time_s; stepped rows apply command over next dt; final row is a terminal snapshot",
            "joint_order":["axis_e","axis_d","axis_c","axis_b"]}
        if hasattr(self.reference,"task_summary"):
            result["pick_place"] = self.reference.task_summary(self.rows,self.data)
        return result

    def export(self,directory,plots=True):
        directory = Path(directory)
        directory.mkdir(parents=True,exist_ok=True)
        keys = list(self.rows[0]) if self.rows else []
        arrays = {key:np.array([row[key] for row in self.rows]) for key in keys}
        np.savez_compressed(directory/"telemetry.npz",**arrays)
        fields = []
        for key,value in arrays.items():
            if value.ndim == 1:
                fields.append((key,None,key))
            else:
                suffix = list("XYZ") if "position" in key else list("wxyz") if "quaternion" in key else list("EDCB")
                fields.extend((key,i,f"{key}_{suffix[i]}") for i in range(value.shape[1]))
        with (directory/"telemetry.csv").open("w",encoding="utf-8-sig",newline="") as handle:
            writer = csv.writer(handle)
            writer.writerow([f[2] for f in fields])
            for row in self.rows:
                writer.writerow([row[key] if index is None else row[key][index] for key,index,_ in fields])
        (directory/"summary.json").write_text(json.dumps(self.summary(),indent=2,ensure_ascii=False),encoding="utf-8")
        (directory/"controller.json").write_text(json.dumps(self.config,indent=2),encoding="utf-8")
        (directory/"trajectory.json").write_text(json.dumps(self.reference.source.payload,indent=2),encoding="utf-8")
        if hasattr(self.reference,"segments"):
            stages = [{key:(value.tolist() if isinstance(value,np.ndarray) else value) for key,value in s.items()} for s in self.reference.segments]
            (directory/"compiled_sequence.json").write_text(json.dumps(stages,indent=2),encoding="utf-8")
        if plots and self.rows:
            from .tracking_plots import save_plots
            save_plots(arrays,directory)
        return directory
