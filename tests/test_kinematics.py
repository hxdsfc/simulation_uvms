import unittest
import numpy as np
from uvms.kinematics import ArmKinematics


class KinematicsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.arm = ArmKinematics()

    def test_jacobian_matches_finite_difference(self):
        q = np.array([0.8, 1.3, 0.9, 0.6])
        for frame in ("world", "base"):
            self.arm.forward(q, unit="rad", frame=frame)
            jac = self.arm.jacobian(frame)
            for i in range(4):
                step = np.eye(4)[i] * 1e-6
                plus = self.arm.forward(q + step, unit="rad", frame=frame)["position_m"]
                minus = self.arm.forward(q - step, unit="rad", frame=frame)["position_m"]
                np.testing.assert_allclose((np.array(plus)-minus)/2e-6, jac[:,i], atol=1e-8)

    def test_frame_conversion_and_units(self):
        degrees = [30, 90, 45, 30]
        world = self.arm.forward(degrees)
        base = self.arm.forward(np.deg2rad(degrees), unit="rad", frame="base")
        rotation = self.arm.data.xmat[self.arm.base].reshape(3,3)
        origin = self.arm.data.xpos[self.arm.base]
        np.testing.assert_allclose(rotation @ base["position_m"] + origin, world["position_m"], atol=1e-12)
        np.testing.assert_allclose(world["q_rad"], base["q_rad"])

    def test_reachable_round_trips_and_limits(self):
        rng = np.random.default_rng(10)
        samples = [self.arm.home_q, np.array([0.8, -1.0, 2.0, 0.6])]
        samples.extend(rng.uniform(self.arm.limits[:,0]+0.05, self.arm.limits[:,1]-0.05, size=(24,4)))
        for i, q in enumerate(samples):
            frame = "world" if i % 2 else "base"
            target = self.arm.forward(q, unit="rad", frame=frame)["position_m"]
            result = self.arm.inverse(target, seed=self.arm.home_q, unit="rad", frame=frame)
            self.assertTrue(result["success"], (q, result))
            self.assertLessEqual(result["error_m"], 1e-5)
            solved = np.array(result["state"]["q_rad"])
            self.assertTrue(np.all(solved >= self.arm.limits[:,0]))
            self.assertTrue(np.all(solved <= self.arm.limits[:,1]))
            self.assertEqual(solved[3], self.arm.home_q[3])
            check = self.arm.forward(solved, unit="rad", frame=frame)
            self.assertLessEqual(np.linalg.norm(np.array(check["position_m"])-target), 1e-5)

    def test_failure_restores_state(self):
        self.arm.forward([30,90,45,30])
        saved = self.arm.data.qpos.copy()
        result = self.arm.inverse([10,10,10])
        self.assertFalse(result["success"])
        np.testing.assert_array_equal(self.arm.data.qpos, saved)
        self.assertIn("best_candidate", result)

    def test_collision_rejection(self):
        # Construct a penetrating pose using the configured proxies.
        q = [1.0, 0.0, 2.9, 0.6]
        state = self.arm.forward(q, unit="rad")
        self.assertFalse(state["collision_free"])
        accepted = self.arm.inverse(state["position_m"], seed=q, unit="rad", restarts=0)
        self.assertTrue(accepted["success"])
        rejected = self.arm.inverse(state["position_m"], seed=q, unit="rad", restarts=0, reject_collisions=True)
        self.assertFalse(rejected["success"])

    def test_invalid_inputs_and_gripper_coupling(self):
        for q in ([0,0,0], [float("nan"),0,0,0], [400,0,0,0]):
            with self.assertRaises(ValueError):
                self.arm.forward(q)
        for xyz in ([1,2], [float("inf"),0,0]):
            with self.assertRaises(ValueError):
                self.arm.inverse(xyz)
        for value in (-0.1, 1.1, float("nan")):
            with self.assertRaises(ValueError):
                self.arm.forward([30,90,45,30], gripper=value)
        self.arm.forward([30,90,45,30], gripper=1)
        np.testing.assert_allclose(self.arm.data.qpos[self.arm.jaws], 0.5, atol=1e-9)


if __name__ == "__main__":
    unittest.main()
