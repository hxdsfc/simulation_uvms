"""Position kinematics for the Alpha 5, in SI units internally.

This maps configurations directly; it does not run a trajectory controller.
One instance owns mutable MuJoCo data and should not be shared across threads.
"""
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
ARM_JOINTS = ("axis_e", "axis_d", "axis_c", "axis_b")


def finite_vector(values, size, name):
    result = np.asarray(values, dtype=float)
    if result.shape != (size,) or not np.isfinite(result).all():
        raise ValueError(f"{name} must contain {size} finite numbers")
    return result


class ArmKinematics:
    def __init__(self, medium="water", model_path=None, model=None):
        if medium not in ("air", "water"):
            raise ValueError("medium must be air or water")
        self.model = model if model is not None else mujoco.MjModel.from_xml_path(str(model_path or ROOT / "models" / f"arm_{medium}.xml"))
        self.data = mujoco.MjData(self.model)
        self.joint_ids = np.array([self.model.joint(n).id for n in ARM_JOINTS])
        self.q_indices = self.model.jnt_qposadr[self.joint_ids]
        self.dof_indices = self.model.jnt_dofadr[self.joint_ids]
        self.limits = self.model.jnt_range[self.joint_ids].copy()
        self.site = self.model.site("tool_tip").id
        self.base = self.model.body("base_link").id
        self.robot_body_ids = set()
        for i in range(1,self.model.nbody):
            parent = i
            while parent and parent != self.base:
                parent = int(self.model.body_parentid[parent])
            if parent == self.base:
                self.robot_body_ids.add(i)
        self.robot_dof_indices = []
        self.environment_q_indices,self.environment_dof_indices = [],[]
        for i in range(self.model.njnt):
            kind = self.model.jnt_type[i]
            nq = 7 if kind == mujoco.mjtJoint.mjJNT_FREE else 4 if kind == mujoco.mjtJoint.mjJNT_BALL else 1
            nv = 6 if kind == mujoco.mjtJoint.mjJNT_FREE else 3 if kind == mujoco.mjtJoint.mjJNT_BALL else 1
            qids = list(range(self.model.jnt_qposadr[i],self.model.jnt_qposadr[i]+nq))
            vids = list(range(self.model.jnt_dofadr[i],self.model.jnt_dofadr[i]+nv))
            if self.model.jnt_bodyid[i] in self.robot_body_ids:
                self.robot_dof_indices.extend(vids)
            else:
                self.environment_q_indices.extend(qids)
                self.environment_dof_indices.extend(vids)
        self.pushrod = int(self.model.jnt_qposadr[self.model.joint("axis_a").id])
        self.jaws = [int(self.model.jnt_qposadr[self.model.joint(n).id]) for n in ("rs1_130_joint", "rs1_139_joint")]
        self.gripper_max = float(self.model.joint("axis_a").range[1])
        mujoco.mj_resetDataKeyframe(self.model, self.data, 0)
        mujoco.mj_forward(self.model, self.data)
        self.home_q = self.data.qpos[self.q_indices].copy()
        self.home_gripper = float(self.data.qpos[self.pushrod] / self.gripper_max)

    def angles(self, q, unit):
        if unit not in ("deg", "rad"):
            raise ValueError("unit must be deg or rad")
        q = finite_vector(q, 4, "joint angles")
        q = np.deg2rad(q) if unit == "deg" else q.copy()
        invalid = (q < self.limits[:, 0]) | (q > self.limits[:, 1])
        if np.any(invalid):
            names = ", ".join(np.array(ARM_JOINTS)[invalid])
            raise ValueError(f"Joint limits exceeded: {names}; inspect joint_limits_rad")
        return q

    def _set(self, q, gripper):
        gripper = self.home_gripper if gripper is None else float(gripper)
        if not np.isfinite(gripper) or not 0 <= gripper <= 1:
            raise ValueError("gripper must be a finite opening fraction in [0, 1]")
        self.data.qpos[self.q_indices] = q
        self.data.qpos[self.pushrod] = gripper * self.gripper_max
        self.data.qpos[self.jaws] = 51 * self.data.qpos[self.pushrod]
        self.data.qvel[self.robot_dof_indices] = 0
        mujoco.mj_forward(self.model, self.data)

    def robot_contact(self,contact):
        return any(int(self.model.geom_bodyid[geom]) in self.robot_body_ids for geom in (contact.geom1,contact.geom2))

    def copy_environment_from(self,source):
        if source.model is not self.model:
            raise ValueError("Environment copy requires the same model")
        self.data.qpos[self.environment_q_indices] = source.data.qpos[self.environment_q_indices]
        self.data.qvel[self.environment_dof_indices] = source.data.qvel[self.environment_dof_indices]
        mujoco.mj_forward(self.model,self.data)

    def _frame(self, frame):
        if frame == "world":
            return np.zeros(3), np.eye(3)
        if frame == "base":
            return self.data.xpos[self.base].copy(), self.data.xmat[self.base].reshape(3,3).copy()
        raise ValueError("frame must be world or base")

    def position(self, frame="world"):
        origin, rotation = self._frame(frame)
        return rotation.T @ (self.data.site_xpos[self.site] - origin)

    def jacobian(self, frame="world"):
        _, rotation = self._frame(frame)
        jacp = np.zeros((3, self.model.nv))
        mujoco.mj_jacSite(self.model, self.data, jacp, None, self.site)
        return rotation.T @ jacp[:, self.dof_indices]

    def state(self, frame="world"):
        origin, rotation = self._frame(frame)
        tcp_rotation = rotation.T @ self.data.site_xmat[self.site].reshape(3,3)
        transform = np.eye(4)
        transform[:3, :3] = tcp_rotation
        transform[:3, 3] = self.position(frame)
        contacts = []
        for contact in self.data.contact:
            if self.robot_contact(contact) and contact.dist < -1e-6:
                contacts.append({"body1": self.model.body(self.model.geom_bodyid[contact.geom1]).name,
                                 "body2": self.model.body(self.model.geom_bodyid[contact.geom2]).name,
                                 "penetration_m": float(-contact.dist)})
        q = self.data.qpos[self.q_indices]
        return {"joint_order": list(ARM_JOINTS), "q_rad": q.tolist(), "q_deg": np.rad2deg(q).tolist(),
                "joint_limits_rad": self.limits.tolist(), "frame": frame, "tcp_site": "tool_tip",
                "position_m": self.position(frame).tolist(), "rotation_matrix": tcp_rotation.tolist(),
                "transform": transform.tolist(), "position_jacobian_m_per_rad": self.jacobian(frame).tolist(),
                "gripper": float(self.data.qpos[self.pushrod] / self.gripper_max),
                "collision_free": not contacts, "penetrating_contacts": contacts,
                "collision_scope": "configured collision proxies and exclusions; pose only"}

    def forward(self, q, unit="deg", frame="world", gripper=None):
        """Set four arm angles, return TCP pose; gripper is opening fraction 0..1."""
        self._frame(frame)
        self._set(self.angles(q, unit), gripper)
        return self.state(frame)

    def inverse(self, xyz, seed=None, unit="deg", frame="world", gripper=None,
                tolerance=1e-5, max_iterations=150, restarts=12, reject_collisions=False):
        """Bounded damped least-squares position IK with deterministic restarts.

        Success constrains position, not orientation. Failure restores input state
        and reports its best candidate; non-convergence does not prove unreachable.
        """
        target = finite_vector(xyz, 3, "target xyz (metres)")
        self._frame(frame)
        if not np.isfinite(tolerance) or tolerance <= 0:
            raise ValueError("tolerance must be finite and positive")
        if not isinstance(max_iterations, int) or max_iterations <= 0:
            raise ValueError("max_iterations must be a positive integer")
        if not isinstance(restarts, int) or restarts < 0:
            raise ValueError("restarts must be a nonnegative integer")
        if unit not in ("deg", "rad"):
            raise ValueError("unit must be deg or rad")
        initial = self.data.qpos[self.q_indices].copy() if seed is None else self.angles(seed, unit)
        saved = self.data.qpos.copy()
        saved_velocity = self.data.qvel.copy()
        rng = np.random.default_rng(42)
        starts = [initial] + [rng.uniform(self.limits[:,0], self.limits[:,1]) for _ in range(restarts)]
        # TCP lies on the wrist roll axis: its angle is unconstrained by position.
        for q in starts:
            q[3] = initial[3]
        best_error, best_state, evaluations = float("inf"), None, 0
        succeeded = False
        try:
            for attempt, q in enumerate(starts, 1):
                for iteration in range(max_iterations):
                    self._set(q, gripper)
                    error = target - self.position(frame)
                    distance = float(np.linalg.norm(error))
                    evaluations += 1
                    if distance < best_error:
                        best_error, best_state = distance, self.state(frame)
                    if distance <= tolerance:
                        state = self.state(frame)
                        if not reject_collisions or state["collision_free"]:
                            succeeded = True
                            return {"success": True, "status": "converged", "target_m": target.tolist(),
                                    "error_m": distance, "tolerance_m": tolerance, "attempts": attempt,
                                    "iterations": evaluations, "state": state, "task": "position_only"}
                        break
                    jac = self.jacobian(frame)
                    damping = 0.002
                    step = jac.T @ np.linalg.solve(jac @ jac.T + damping**2 * np.eye(3), error)
                    # Preserve input wrist roll exactly for a position-only task.
                    step[3] = 0
                    step *= min(1.0, 0.2 / max(float(np.max(np.abs(step))), 1e-12))
                    accepted = False
                    for scale in (1.0, 0.5, 0.25, 0.1, 0.025):
                        candidate = np.clip(q + scale * step, self.limits[:,0], self.limits[:,1])
                        self._set(candidate, gripper)
                        if np.linalg.norm(target - self.position(frame)) < distance - 1e-12:
                            q = candidate
                            accepted = True
                            break
                    if not accepted:
                        break
            return {"success": False, "status": "no_acceptable_solution_found", "target_m": target.tolist(),
                    "error_m": best_error, "tolerance_m": tolerance, "attempts": len(starts),
                    "iterations": evaluations, "best_candidate": best_state, "task": "position_only",
                    "reason": "Solver did not find a limit-compliant solution satisfying the requested position/contact criteria."}
        finally:
            # Successful return leaves the accepted pose; any failure/exception restores it.
            if not succeeded:
                self.data.qpos[:] = saved
                self.data.qvel[:] = saved_velocity
                mujoco.mj_forward(self.model, self.data)

    def inverse_pose(self, xyz, quat_wxyz, seed=None, unit="rad", frame="world", gripper=None,
                     position_tolerance=1e-5, orientation_tolerance=1e-3,
                     max_iterations=200, restarts=0, reject_collisions=True):
        """Overdetermined four-DOF pose IK; never silently relax orientation."""
        from .spatial import quaternion, matrix_quaternion, rotation_error
        target = finite_vector(xyz,3,"position (m)")
        target_quat = quaternion(quat_wxyz)
        _,basis = self._frame(frame)
        if any(not np.isfinite(v) or v <= 0 for v in (position_tolerance,orientation_tolerance)):
            raise ValueError("Pose tolerances must be finite and positive")
        if not isinstance(max_iterations,int) or max_iterations < 1 or not isinstance(restarts,int) or restarts < 0:
            raise ValueError("Invalid IK iteration budget")
        initial = self.angles(seed,unit) if seed is not None else self.data.qpos[self.q_indices].copy()
        saved,velocity = self.data.qpos.copy(),self.data.qvel.copy()
        rng = np.random.default_rng(42)
        starts = [initial]+[rng.uniform(self.limits[:,0],self.limits[:,1]) for _ in range(restarts)]
        succeeded,best = False,None
        best_cost = float("inf")
        def error_vector():
            rotation = basis.T @ self.data.site_xmat[self.site].reshape(3,3)
            return target-self.position(frame),rotation_error(target_quat,matrix_quaternion(rotation))
        try:
            for q in starts:
                for iteration in range(max_iterations):
                    self._set(q,gripper)
                    ep,er = error_vector()
                    cost = float(ep@ep+0.01*(er@er))
                    if cost < best_cost:
                        best_cost,best = cost,(float(np.linalg.norm(ep)),float(np.linalg.norm(er)))
                    if np.linalg.norm(ep) <= position_tolerance and np.linalg.norm(er) <= orientation_tolerance:
                        state = self.state(frame)
                        if not reject_collisions or state["collision_free"]:
                            succeeded = True
                            return {"success":True,"state":state,"position_error_m":float(np.linalg.norm(ep)),
                                    "orientation_error_rad":float(np.linalg.norm(er)),"task":"full_pose"}
                        break
                    jp,jr = np.zeros((3,self.model.nv)),np.zeros((3,self.model.nv))
                    mujoco.mj_jacSite(self.model,self.data,jp,jr,self.site)
                    jac = np.vstack((basis.T @ jp[:,self.dof_indices],0.1*(basis.T @ jr[:,self.dof_indices])))
                    error = np.r_[ep,0.1*er]
                    step = np.linalg.solve(jac.T@jac+1e-6*np.eye(4),jac.T@error)
                    step *= min(1,0.15/max(np.max(np.abs(step)),1e-12))
                    accepted = False
                    for scale in (1,.5,.25,.1,.025):
                        candidate = np.clip(q+step*scale,self.limits[:,0],self.limits[:,1])
                        self._set(candidate,gripper)
                        p,r = error_vector()
                        if p@p+0.01*(r@r) < cost-1e-15:
                            q,accepted = candidate,True
                            break
                    if not accepted:
                        break
            return {"success":False,"position_error_m":best[0],"orientation_error_rad":best[1],
                    "reason":"Four arm joints cannot realize this pose within the requested tolerances."}
        finally:
            if not succeeded:
                self.data.qpos[:],self.data.qvel[:] = saved,velocity
                mujoco.mj_forward(self.model,self.data)
