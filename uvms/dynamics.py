"""Rigid-body mass/bias model, reduced through the two linear gripper mimics."""
import numpy as np
import mujoco
from .kinematics import ArmKinematics, finite_vector


class RigidBodyDynamics:
    def __init__(self, arm):
        self.arm = ArmKinematics(model=arm.model)
        self.source_arm = arm
        self.model = arm.model
        self.indices = np.r_[arm.dof_indices,self.model.jnt_dofadr[self.model.joint("axis_a").id]]
        self.q_indices = np.r_[arm.q_indices,arm.pushrod]
        self.mapping = np.zeros((self.model.nv,5))
        self.mapping[self.indices,np.arange(5)] = 1
        for name in ("rs1_130_joint","rs1_139_joint"):
            self.mapping[self.model.jnt_dofadr[self.model.joint(name).id],4] = 51

    def from_data(self,data):
        mass = np.empty((self.model.nv,self.model.nv))
        mujoco.mj_fullM(self.model,data,mass)
        basis = self.mapping
        return {"mass":basis.T@mass@basis,"bias":basis.T@data.qfrc_bias,
                "passive":basis.T@data.qfrc_passive,"full_mass":mass}

    def terms(self,q,qd=None,gripper=None):
        self.arm.copy_environment_from(self.source_arm)
        q = self.arm.angles(q,"rad")
        qd = np.zeros(4) if qd is None else finite_vector(qd,4,"joint velocity rad/s")
        self.arm._set(q,gripper)
        data = self.arm.data
        data.qvel[:] = self.mapping @ np.r_[qd,0]
        mujoco.mj_forward(self.model,data)
        result = self.from_data(data)
        velocity = data.qvel.copy()
        data.qvel[:] = 0
        mujoco.mj_forward(self.model,data)
        result["gravity"] = self.mapping.T@data.qfrc_bias
        result["coriolis_centrifugal"] = result["bias"]-result["gravity"]
        data.qvel[:] = velocity
        mujoco.mj_forward(self.model,data)
        return result

    def inverse(self,q,qd,qdd,gripper=None,compensate_passive=False):
        terms = self.terms(q,qd,gripper)
        acceleration = np.r_[finite_vector(qdd,4,"joint acceleration rad/s²"),0]
        tau = terms["mass"]@acceleration+terms["bias"]
        if compensate_passive:
            tau -= terms["passive"]
        return {"torque_nm":tau[:4],"gripper_force_n":float(tau[4]),"generalized_force":tau}

    def forward(self,q,qd,torque_nm,gripper_force_n=0,gripper=None,include_passive=False):
        terms = self.terms(q,qd,gripper)
        force = np.r_[finite_vector(torque_nm,4,"torque Nm"),float(gripper_force_n)]
        if not np.isfinite(force).all():
            raise ValueError("Force must be finite")
        rhs = force-terms["bias"]+(terms["passive"] if include_passive else 0)
        return np.linalg.solve(terms["mass"],rhs)
