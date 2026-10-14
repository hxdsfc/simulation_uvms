import csv
import json
from pathlib import Path
import tempfile
import unittest
import numpy as np
import mujoco
from uvms.kinematics import ArmKinematics
from uvms.spatial import matrix_quaternion,rotation_error
from uvms.dynamics import RigidBodyDynamics
from uvms.trajectory import InputTrajectory,CompiledTrajectory
from uvms.tracking import TrackingSession

ROOT = Path(__file__).resolve().parents[1]


class TrackingTests(unittest.TestCase):
    def setUp(self):
        self.arm = ArmKinematics("air")

    def test_pose_ik_and_unrealizable_orientation(self):
        target = self.arm.forward(self.arm.home_q+np.array([.02,.015,-.01,.04]),unit="rad")
        quat = matrix_quaternion(target["rotation_matrix"])
        result = self.arm.inverse_pose(target["position_m"],quat,seed=self.arm.home_q)
        self.assertTrue(result["success"])
        self.assertLess(result["position_error_m"],1e-5)
        self.assertLess(result["orientation_error_rad"],1e-3)
        saved = self.arm.data.qpos.copy()
        # Arbitrarily rotating the TCP in world X at fixed position is overconstrained.
        twist = np.array([np.cos(.3),np.sin(.3),0,0])
        matrix = np.empty(9)
        mujoco.mju_quat2Mat(matrix,twist)
        impossible = matrix_quaternion(matrix.reshape(3,3)@np.array(target["rotation_matrix"]))
        failed = self.arm.inverse_pose(target["position_m"],impossible,seed=result["state"]["q_rad"])
        self.assertFalse(failed["success"])
        np.testing.assert_array_equal(saved,self.arm.data.qpos)

    def test_rigid_dynamics_mass_and_inverse_forward_identity(self):
        dynamics = RigidBodyDynamics(self.arm)
        q,qd,qdd = self.arm.home_q,np.array([.05,.04,-.03,.01]),np.array([.1,-.1,.08,.02])
        terms = dynamics.terms(q,qd)
        np.testing.assert_allclose(terms["mass"],terms["mass"].T,atol=1e-12)
        self.assertGreater(np.linalg.eigvalsh(terms["mass"]).min(),0)
        np.testing.assert_allclose(terms["bias"],terms["gravity"]+terms["coriolis_centrifugal"])
        inverse = dynamics.inverse(q,qd,qdd)
        actual = dynamics.forward(q,qd,inverse["torque_nm"],inverse["gripper_force_n"])
        np.testing.assert_allclose(actual,np.r_[qdd,0],atol=1e-10)

    def test_physics_follows_full_pose_and_exports_synchronized_data(self):
        source = InputTrajectory.load(ROOT/"trajectories/feasible_pose.json")
        saved = self.arm.data.qpos.copy()
        reference = CompiledTrajectory(self.arm,source)
        np.testing.assert_array_equal(self.arm.data.qpos,saved)
        session = TrackingSession(self.arm,reference)
        session.step()
        self.assertAlmostEqual(self.arm.data.time,float(self.arm.model.opt.timestep))
        # Actual state is integrated and is not overwritten with the reference q.
        self.assertFalse(np.array_equal(self.arm.data.qpos[self.arm.q_indices],reference.sample(session.time)["q"]))
        result = session.run()
        self.assertEqual(result["status"],"completed")
        self.assertLess(result["position_rmse_m"],1e-4)
        self.assertLess(result["orientation_rmse_rad"],1e-3)
        rows = session.rows
        self.assertTrue(all(r["time_s"] < rows[i+1]["time_s"] for i,r in enumerate(rows[:-1])))
        np.testing.assert_allclose([r["tau_actual_nm"] for r in rows],[r["tau_command_nm"] for r in rows],atol=1e-10)
        self.assertGreater(np.max(np.abs([r["tau_actual_nm"] for r in rows])),.1)
        with tempfile.TemporaryDirectory() as directory:
            session.export(directory,plots=False)
            with open(Path(directory)/"telemetry.csv",encoding="utf-8-sig",newline="") as handle:
                values = list(csv.DictReader(handle))
            self.assertEqual(len(values),len(rows))
            self.assertIn("quaternion_des_wxyz_w",values[0])
            self.assertIn("tau_actual_nm_D",values[0])
            with np.load(Path(directory)/"telemetry.npz") as archive:
                self.assertEqual(archive["q_act_rad"].shape,(len(rows),4))

    def test_position_mode_and_csv_inputs(self):
        trajectory = InputTrajectory.load(ROOT/"trajectories/position_loop.json")
        reference = CompiledTrajectory(self.arm,trajectory)
        self.assertEqual(reference.source.mode,"position")
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/"example.csv"
            p = self.arm.position()
            path.write_text("time_s,x_m,y_m,z_m\n"+f"0,{p[0]},{p[1]},{p[2]}\n2,{p[0]+.001},{p[1]},{p[2]}\n",encoding="utf-8")
            csv_source = InputTrajectory.load(path)
            self.assertEqual(csv_source.mode,"position")
            self.assertEqual(csv_source.duration,2)
        with self.assertRaises(ValueError):
            InputTrajectory({"points":[{"t":0,"position":[0,0,0]},{"t":0,"position":[0,0,0]}]})

    def test_auto_approach_and_saturation_is_logged(self):
        q = self.arm.home_q.copy()
        q[0] += .04
        self.arm.forward(q,unit="rad")
        source = InputTrajectory.load(ROOT/"trajectories/feasible_pose.json")
        reference = CompiledTrajectory(self.arm,source)
        self.assertGreater(reference.approach_duration,0)
        session = TrackingSession(self.arm,reference)
        # Inject an initial velocity disturbance, not a reference teleport.
        self.arm.data.qvel[self.arm.dof_indices] = [30,30,30,30]
        row = session.control_and_record(force_log=True)
        self.assertTrue(row["saturated"])
        self.assertTrue(np.all(np.abs(row["tau_actual_nm"]) <= 9+1e-10))
        session.stop()
        self.assertEqual(session.status,"stopped")
        np.testing.assert_array_equal(self.arm.data.ctrl,np.zeros(5))


if __name__ == "__main__":
    unittest.main()
