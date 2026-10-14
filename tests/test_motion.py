import unittest
import numpy as np
from uvms.kinematics import ArmKinematics
from uvms.motion import MotionPlanner, MotionError


class MotionTests(unittest.TestCase):
    def setUp(self):
        self.arm = ArmKinematics()
        self.planner = MotionPlanner(self.arm)

    def test_joint_interpolation_and_speed_limits(self):
        start = self.arm.home_q.copy()
        target = start+np.deg2rad([30,2,1,5])
        saved = self.arm.data.qpos.copy()
        plan = self.planner.joint(target,duration=0.1)
        np.testing.assert_array_equal(self.arm.data.qpos,saved)
        middle,progress = plan.at(plan.duration/2)
        np.testing.assert_allclose(middle,(start+target)/2,atol=1e-12)
        self.assertEqual(progress,0.5)
        np.testing.assert_allclose(plan.at(plan.duration)[0],target)
        speeds = np.abs(np.diff(plan.q,axis=0))/(plan.duration/(len(plan.q)-1))
        self.assertTrue(np.all(speeds <= self.planner.joint_speed+1e-10))
        self.assertGreaterEqual(plan.duration,1.5)

    def test_cartesian_line_and_replayed_tcp_speed(self):
        start = self.arm.position().copy()
        target = start+np.array([0.012,-0.01,0.008])
        plan = self.planner.cartesian(target,duration=0.1)
        fractions = np.linspace(0,1,81)
        positions = []
        for fraction in fractions:
            q,_ = plan.at(plan.duration*fraction)
            state = self.arm.forward(q,unit="rad")
            xyz = np.array(state["position_m"])
            self.assertLessEqual(np.linalg.norm(xyz-(start+(target-start)*fraction)),5e-5)
            self.assertTrue(state["collision_free"])
            positions.append(xyz)
        speeds = np.linalg.norm(np.diff(positions,axis=0),axis=1)/(plan.duration/80)
        self.assertTrue(np.all(speeds <= self.planner.config["tcp_speed_m_s"]*1.005))

    def test_amplitude_clamps_and_invalid_inputs(self):
        target = self.arm.home_q.copy()
        target[0] = 100
        plan = self.planner.joint(target,reject_contacts=False)
        self.assertTrue(plan.clipped)
        self.assertEqual(plan.q[-1,0],self.arm.limits[0,1])
        for point in ([10,10,10],[-10,-10,-10],[0,0,-5]):
            bounded,clipped = self.planner.clamp_target(point)
            self.assertTrue(clipped)
            self.assertTrue(self.planner.in_bounds(bounded))
        with self.assertRaises(ValueError):
            self.planner.joint([float("nan"),0,0,0])

    def test_reachable_endpoint_can_have_infeasible_straight_path(self):
        target_q = self.arm.home_q.copy()
        target_q[0] += 3
        target = self.planner.probe.forward(target_q,unit="rad")
        self.assertTrue(target["collision_free"])
        saved = self.arm.data.qpos.copy()
        with self.assertRaises(MotionError):
            self.planner.cartesian(target["position_m"])
        np.testing.assert_array_equal(self.arm.data.qpos,saved)

    def test_collision_path_rejected_without_moving_live_model(self):
        saved = self.arm.data.qpos.copy()
        with self.assertRaises(MotionError):
            self.planner.joint([1,0,2.9,0.6])
        np.testing.assert_array_equal(self.arm.data.qpos,saved)

    def test_workspace_samples_and_bounds_leave_model_unchanged(self):
        saved = self.arm.data.qpos.copy()
        cloud = self.planner.workspace_cloud(counts=(8,8,8))
        self.assertGreater(len(cloud),50)
        self.assertTrue(all(self.planner.in_bounds(point) for point in cloud))
        np.testing.assert_array_equal(self.arm.data.qpos,saved)


if __name__ == "__main__":
    unittest.main()
